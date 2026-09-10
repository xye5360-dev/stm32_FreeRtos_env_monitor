/**
 * @file    env_monitor.c
 * @brief   基于FreeRTOS的智能环境监测系统 —— 核心实现
 * @author  Mosongping
 * @date    2025-07
 *
 * ■ 架构（4个任务 + 4种IPC，面试口述版）：
 *   EnvSensorTask   传感器采集任务  (2000ms)  读DHT11温湿度/光敏/超声波/红外
 *   EnvDisplayTask  OLED显示任务    (1000ms)  从队列取最新帧实时刷新屏幕
 *   EnvAlarmTask    阈值报警任务    (500ms)   判断超标 + 去抖 + 蜂鸣器/LED声光报警
 *   EnvUARTTask     串口上报任务    (3000ms)  用 printf 把环境帧发到上位机
 *
 * ■ IPC 机制（对应简历上的关键词）：
 *   Queue   队列       传感器任务写最新一帧，显示/报警/上报三个任务读同一帧
 *   Mutex   互斥量     保护 OLED 总线（多个任务都要刷屏，防止数据交叉）
 *   Timer   软件定时器 心跳计时，同时作为报警任务的"节拍"来源
 *   Notify  任务通知   软件定时器回调里用任务通知"轻量唤醒"报警任务，
 *                      避免报警任务自己 while 空转、也避免用队列传一个无意义帧
 *
 * ■ 关键设计点（面试会深挖，提前想好怎么答）：
 *   1. 为什么队列深度只有 1？    -> 环境数据是"最新值"而非"历史流"，
 *                                   丢掉旧帧无所谓，用 xQueueOverwrite 覆盖比阻塞更好
 *   2. 为什么用互斥量不用信号量？ -> 互斥量带"优先级继承"，低优先级任务持锁时
 *                                   会把优先级临时抬到跟高优先级一样，防止"优先级反转"
 *   3. 为什么去抖是 3 次？        -> 温度在阈值附近会自然波动，1次会误报；
 *                                   3次=连续3个周期(1.5s)仍超标才信，物理上站得住
 *   4. 为什么 DHT11 用关中断？    -> 单总线要求微秒级时序，任务切换会打断它，
 *                                   用 taskENTER_CRITICAL 临时关中断保护时序窗口
 *   5. 为什么报警用定时器+通知，不自己轮询？ -> 解耦"采集/显示"与"报警"的节奏，
 *                                   定时器是系统时钟，不随任务调度漂移，报警节拍稳定
 */

#include "env_monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "cmsis_os.h"

#include "driver_oled.h"
#include "driver_dht11.h"
#include "driver_led.h"
#include "driver_active_buzzer.h"
#include "driver_light_sensor.h"
#include "driver_ir_obstacle.h"
#include "driver_ultrasonic_sr04.h"
#include "driver_uart.h"

#include <stdio.h>
#include <string.h>

/* ─────────────────────────────────────────
 * 一、IPC 对象 / 全局状态
 * ───────────────────────────────────────── */
static QueueHandle_t       g_xQueueEnvData;    /* 环境数据队列(深度1,只留最新) */
static SemaphoreHandle_t   g_xMutexOLED;       /* OLED总线互斥量(优先级继承)    */
static TimerHandle_t       g_xTimerAlarm;      /* 报警节拍软件定时器             */
static TaskHandle_t        g_xAlarmTaskHandle; /* 报警任务句柄(定时器回调要通知它)*/
static volatile uint32_t   g_ulSensorTick = 0; /* 传感器采集计数(显示在屏幕)     */
static volatile uint32_t   g_ulAlarmTick  = 0; /* 报警任务被唤醒次数(调试用)     */

/* 光照最高/最低记录，用来判断串口曲线是否合理(可选调试) */
static volatile uint16_t   g_usLightMax = 0;
static volatile uint16_t   g_usLightMin = 4095;

/* ═══════════════════════════════════════════
 * 二、OLED 总线互斥量封装
 *   OLED 被多个任务共用(显示任务/报警任务都要刷屏)，
 *   必须用同一个互斥量串行化访问，否则两条 I2C/SPI 写会交错。
 * ═══════════════════════════════════════════ */
static inline void OLED_Lock(void)   { xSemaphoreTake(g_xMutexOLED, portMAX_DELAY); }
static inline void OLED_Unlock(void) { xSemaphoreGive(g_xMutexOLED); }

/* ═══════════════════════════════════════════
 * 三、声光报警动作封装（供报警任务调用）
 * ═══════════════════════════════════════════ */
static void Buzzer_Beep(uint8_t times, uint16_t on_ms, uint16_t off_ms)
{
    for (uint8_t i = 0; i < times; i++) {
        ActiveBuzzer_Control(1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        ActiveBuzzer_Control(0);
        if (i < times - 1)
            vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

static void LED_Flash(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++) {
        Led_Control(LED_GREEN, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        Led_Control(LED_GREEN, 0);
        if (i < times - 1)
            vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ═══════════════════════════════════════════
 * 四、状态 → OLED状态行文本 映射（报警任务用）
 *   ⚠ 每行最多 16 字符(128/8)，超过 OLED_PrintString 会自动换页 → 乱码。
 *   所以状态文本一律控制在 16 字符以内。
 * ═══════════════════════════════════════════ */
static const char *SysStatusToText(sys_status_t st)
{
    switch (st) {
        case SYS_TEMP_HIGH:         return "TEMP HIGH!!    ";
        case SYS_TEMP_LOW:          return "TEMP LOW!!     ";
        case SYS_HUMIDITY_HIGH:     return "HUMI HIGH!!    ";
        case SYS_HUMIDITY_LOW:      return "HUMI LOW!!     ";
        case SYS_LIGHT_DARK:        return "LIGHT DARK     ";
        case SYS_OBSTACLE_DETECTED: return "OBSTACLE!!     ";
        case SYS_NORMAL:
        default:                    return "Normal         ";
    }
}

/* ═══════════════════════════════════════════
 * 五、核心判断：当前这一帧环境数据处于什么状态
 *   ⚠ 关键：每个传感器【独立】判断，一个读失败绝不屏蔽其它报警。
 *   之前 DHT11 读失败 → 温度无效 → 直接 return 正常，连障碍物都被忽略，
 *   这就是"有障碍物却显示 Normal"的根因。现在改成逐项独立判断。
 * ═══════════════════════════════════════════ */
static sys_status_t EvaluateStatus(const env_data_t *env)
{
    /* 温度：无效(-999)则跳过，不影响其它判断 */
    if (env->temperature != SENSOR_INVALID_VALUE) {
        if (env->temperature >= TEMP_HIGH_THRESHOLD)
            return SYS_TEMP_HIGH;
        if (env->temperature <= TEMP_LOW_THRESHOLD)
            return SYS_TEMP_LOW;
    }

    /* 湿度：无效则跳过 */
    if (env->humidity != SENSOR_INVALID_VALUE) {
        if (env->humidity >= HUMIDITY_HIGH_THRESHOLD)
            return SYS_HUMIDITY_HIGH;
        if (env->humidity <= HUMIDITY_LOW_THRESHOLD)
            return SYS_HUMIDITY_LOW;
    }

    /* 红外避障：0=有障碍物，1=无障碍。这个是最可靠的，必须能独立用 */
    if (env->obstacle == 0)
        return SYS_OBSTACLE_DETECTED;

    /* 光照 ADC：过低视为暗 */
    if (env->light > 0 && env->light < LIGHT_DARK_THRESHOLD)
        return SYS_LIGHT_DARK;

    return SYS_NORMAL;
}

/* ═══════════════════════════════════════════
 * 六、任务1：EnvSensorTask —— 多传感器数据采集
 *   周期：2000ms
 * ═══════════════════════════════════════════ */
static void EnvSensorTask(void *params)
{
    env_data_t env;
    int  hum = 0, temp = 0, err = 0, attempt = 0;
    uint32_t val32 = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    (void)params;

    /* DHT11 上电后需要 ~1s 稳定，否则第一次读必失败。先等一会儿再进主循环 */
    vTaskDelay(pdMS_TO_TICKS(1500));

    while (1) {
        /* 初始化该帧为"全部无效"，读失败的字段会保持无效标记 */
        memset(&env, 0, sizeof(env_data_t));
        env.temperature = SENSOR_INVALID_VALUE;
        env.humidity    = SENSOR_INVALID_VALUE;

        /* ── DHT11 温湿度：单总线需要微秒级时序，关中断保护。
         *   读失败自动重试最多3次，成功即停；3次全失败才标无效。
         *   ⚠ 重试之间必须让出总线(不能连着读)，否则 DHT11 会拒绝应答。 */
        err = -1;
        for (attempt = 0; attempt < 3; attempt++) {
            if (attempt > 0)
                vTaskDelay(pdMS_TO_TICKS(20));   /* 重试前让总线恢复 */
            taskENTER_CRITICAL();
            err = DHT11_Read(&hum, &temp);
            taskEXIT_CRITICAL();
            if (err == 0)
                break;
        }

        if (err == 0) {
            env.temperature = temp;
            env.humidity    = hum;
        } else {
            /* 3次全失败：保留无效标记，界面显示 "--"，不误报 */
            printf("[ENV] DHT11 read fail (3 tries)\r\n");
        }

        /* ── 光敏传感器 (ADC) ──
         *   硬件是"光敏电阻+分压"：光越强 → ADC 原始值越低。
         *   这里做一次反转(4095 - raw)，让显示符合直觉：光越强数值越大。
         *   反转后：强光≈4095，黑暗≈0。 */
        if (LightSensor_Read(&val32) == 0) {
            env.light = (uint16_t)(4095 - val32);   /* 反转：光越强值越大 */
            if (env.light > g_usLightMax) g_usLightMax = env.light;
            if (env.light < g_usLightMin) g_usLightMin = env.light;
        }

        /* ── 超声波HC-SR04 ── */
        if (SR04_Read(&val32) == 0) {
            env.distance = (uint16_t)val32;
        }

        /* ── 红外避障：0=有障碍物 → obstacle 字段记为 0 ── */
        env.obstacle = (IRObstacle_Read() == 0) ? 0 : 1;

        /* ── 把这一帧"覆盖"进队列（只留最新，不排队） ── */
        xQueueOverwrite(g_xQueueEnvData, &env);

        g_ulSensorTick++;

        /* 固定周期 + 状态记录(采集计数同步到屏幕) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/* ═══════════════════════════════════════════
 * 七、任务2：EnvDisplayTask —— OLED 实时显示
 *   周期：1000ms
 * ═══════════════════════════════════════════ */
static void EnvDisplayTask(void *params)
{
    env_data_t env;
    char buff[20];
    TickType_t xLastWakeTime = xTaskGetTickCount();

    (void)params;

    /* ⚠ OLED 坐标系：128x64 页地址模式，8 个页(0~7)。
     *   每个字符占 2 页(16px 高)，所以只能放 4 行，页号必须是偶数 0/2/4/6。
     *   用奇数页(1/3/5/7)字符会从半页开始写，和相邻行重叠 → 乱码。
     *   每行最多 16 字符(128/8)，超了 OLED_PrintString 会强行换页，又会乱。 */

    /* 首屏：先画一个稳定的静态框架（4 行偶数页） */
    OLED_Lock();
    OLED_Clear();
    OLED_PrintString(0, 0, "T:--'C  H:--%");
    OLED_PrintString(0, 2, "L:----  Obst:--");
    OLED_PrintString(0, 4, "Dist:---cm");
    OLED_PrintString(0, 6, "Status: Normal");
    OLED_Unlock();

    while (1) {
        if (xQueuePeek(g_xQueueEnvData, &env, pdMS_TO_TICKS(DISPLAY_PERIOD_MS)) == pdPASS) {
            OLED_Lock();

            /* 行1(页0)：温度/湿度（无效则显示 "--"） */
            if (env.temperature != SENSOR_INVALID_VALUE) {
                sprintf(buff, "T:%3d'C H:%3d%%", env.temperature, env.humidity);
            } else {
                sprintf(buff, "T:--'C H:--%%  ");
            }
            OLED_ClearLine(0, 0);
            OLED_PrintString(0, 0, buff);

            /* 行2(页2)：光照 + 障碍物（0=有障碍 YES，1=无障碍 NO） */
            sprintf(buff, "L:%-4d Obst:%s", env.light, env.obstacle == 0 ? "Y" : "N");
            OLED_ClearLine(0, 2);
            OLED_PrintString(0, 2, buff);

            /* 行3(页4)：超声波距离，0 显示为 -- */
            if (env.distance > 0) {
                sprintf(buff, "Dist:%-3dcm", env.distance);
            } else {
                sprintf(buff, "Dist:---cm ");
            }
            OLED_ClearLine(0, 4);
            OLED_PrintString(0, 4, buff);

            OLED_Unlock();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

/* ═══════════════════════════════════════════
 * 八、软件定时器回调 —— 报警任务的"节拍源"
 *   500ms 触发一次。回调里只做一个轻量动作：
 *   发送任务通知唤醒 EnvAlarmTask，不在此处做重逻辑
 *   （软件定时器回调运行在定时器守护任务里，不能阻塞）。
 * ═══════════════════════════════════════════ */
static void AlarmTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    /* 回调运行在"定时器守护任务"里，必须快、不能阻塞。
     * 只用任务通知轻量唤醒报警任务：不传数据，只做计数触发。 */
    if (g_xAlarmTaskHandle != NULL) {
        xTaskNotifyGive(g_xAlarmTaskHandle);
    }
}

/* ═══════════════════════════════════════════
 * 九、任务3：EnvAlarmTask —— 阈值报警 + 去抖 + 声光报警
 *   周期：500ms（由软件定时器任务通知驱动）
 * ═══════════════════════════════════════════ */
static void EnvAlarmTask(void *params)
{
    env_data_t   env;
    sys_status_t cur  = SYS_NORMAL;
    sys_status_t last = SYS_NORMAL;
    uint8_t      alarm_count = 0;    /* 连续异常计数(去抖) */
    uint8_t      alarm_active = 0;   /* 当前是否处于"报警态"：0=正常 1=已报警 */
    TickType_t   xLastWakeTime = xTaskGetTickCount();

    (void)params;

    while (1) {
        /* 等待软件定时器的任务通知（500ms 一次）。
         * pdTRUE=清空计数(信号量式)，只关心"被叫醒"这个事件。 */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ALARM_PERIOD_MS));

        /* 取最新一帧；取不到说明还没采集数据，跳过本次 */
        if (xQueuePeek(g_xQueueEnvData, &env, pdMS_TO_TICKS(ALARM_PERIOD_MS)) != pdPASS)
            continue;

        cur = EvaluateStatus(&env);
        g_ulAlarmTick++;

        /* ═══ 去抖核心逻辑 ═══
         *   ALARM_DEBOUNCE=3：模拟量(温湿度/光照)要"连续3个周期(1.5s)都异常"才信，
         *   防止在阈值边沿反复抖。
         *   ⚠ 障碍物是二值突变量(0/1)，不需要去抖——手一挡就该立刻报，
         *      否则去抖3次要1.5s，手移开就不触发。所以障碍物直接视为已达标。 */
        if (cur == SYS_OBSTACLE_DETECTED) {
            alarm_count = ALARM_DEBOUNCE;   /* 立即触发报警 */
        } else if (cur != SYS_NORMAL) {
            if (cur == last) {
                alarm_count++;
            } else {
                alarm_count = 1;
            }
        } else {
            alarm_count = 0;
        }

        /* ── 进入报警态：去抖达标 + 当前还没在报 ── */
        if (alarm_count >= ALARM_DEBOUNCE && alarm_active == 0) {
            alarm_active = 1;

            OLED_Lock();
            OLED_ClearLine(0, 6);
            OLED_PrintString(0, 6, SysStatusToText(cur));
            OLED_Unlock();

            Buzzer_Beep(3, 100, 80);
            LED_Flash(5);

            printf("[ENV][ALARM] status=%d temp=%d hum=%d light=%u dist=%u obst=%d\r\n",
                   cur, env.temperature, env.humidity,
                   (unsigned)env.light, (unsigned)env.distance, env.obstacle);
        }

        /* ── 恢复正常：清除状态行 + 关闭声光 ── */
        if (alarm_count == 0 && alarm_active == 1) {
            alarm_active = 0;

            OLED_Lock();
            OLED_ClearLine(0, 6);
            OLED_PrintString(0, 6, SysStatusToText(SYS_NORMAL));
            OLED_Unlock();

            ActiveBuzzer_Control(0);
            Led_Control(LED_GREEN, 0);
            printf("[ENV][ALARM] back to normal\r\n");
        }

        last = cur;

        /* 固定节拍（防止任务通知丢失时的兜底自走） */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ALARM_PERIOD_MS));
    }
}

/* ═══════════════════════════════════════════
 * 十、任务4：EnvUARTTask —— 串口上报
 *   周期：3000ms
 *   ⚠ 这里就是原工程"没写完整"的地方：原代码拼好 buff 却空转。
 *   本版本直接 printf 真实发送（driver_uart.c 已重定义 fputc，
 *   所以 printf 会走 USART1 发出）。
 * ═══════════════════════════════════════════ */
static void EnvUARTTask(void *params)
{
    env_data_t   env;
    sys_status_t status;
    TickType_t   xLastWakeTime = xTaskGetTickCount();

    (void)params;

    while (1) {
        if (xQueuePeek(g_xQueueEnvData, &env, pdMS_TO_TICKS(UART_REPORT_PERIOD_MS)) == pdPASS) {
            /* 用与报警任务同一套判断逻辑，算出当前状态(0~6) */
            status = EvaluateStatus(&env);

            /* 每 3 秒【固定】发一帧完整状态：
             *   温湿度(无效为-999) + 光照 + 距离 + 障碍物(0=有/1=无) + 当前状态
             *   这样无需等报警/正常事件，上位机每 3 秒都会收到一帧，便于持续观察趋势。 */
            printf("[ENV] T:%d,H:%d,L:%u,D:%u,O:%d,status=%d\r\n",
                   env.temperature,
                   env.humidity,
                   (unsigned)env.light,
                   (unsigned)env.distance,
                   env.obstacle,
                   (int)status);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(UART_REPORT_PERIOD_MS));
    }
}

/* ═══════════════════════════════════════════
 * 十一、任务入口：EnvMonitorTask
 *   初始化 IPC/外设 → 创建4个子任务 → 进入主监控循环
 * ═══════════════════════════════════════════ */
void EnvMonitorTask(void *params)
{
    (void)params;
    TaskHandle_t xHandle;

    /* ── 1. 创建 IPC 对象 ── */
    g_xMutexOLED   = xSemaphoreCreateMutex();          /* 互斥量：OLED 总线 */
    g_xQueueEnvData = xQueueCreate(QUEUE_DEPTH, sizeof(env_data_t)); /* 队列：环境帧 */
    configASSERT(g_xMutexOLED != NULL);
    configASSERT(g_xQueueEnvData != NULL);

    /* ── 2. 外设初始化 ── */
    OLED_Lock();
    OLED_Init();
    OLED_Clear();
    OLED_Unlock();

    DHT11_Init();
    Led_Init();
    ActiveBuzzer_Init();
    LightSensor_Init();
    IRObstacle_Init();
    SR04_Init();
    UART_Init();

    /* ⚠ 蜂鸣器/绿灯都是"低电平有效"：
     *   驱动初始化后 GPIO 默认为低电平 = 蜂鸣器直接响。
     *   必须主动置为"不响/不亮"，否则一开机就嗡嗡叫。 */
    ActiveBuzzer_Control(0);
    Led_Control(LED_GREEN, 0);

    OLED_Clear();
    OLED_PrintString(0, 0, "Sys Starting...");

    /* ── 3. 创建软件定时器：500ms 报警节拍 ── */
    g_xTimerAlarm = xTimerCreate("AlarmTmr",
                                 pdMS_TO_TICKS(ALARM_PERIOD_MS),
                                 pdTRUE,          /* 自动重载 */
                                 NULL,
                                 AlarmTimerCallback);
    configASSERT(g_xTimerAlarm != NULL);
    xTimerStart(g_xTimerAlarm, 0);

    /* ── 4. 创建4个子任务
     *   优先级：传感器采集 > 显示 = 报警 > 串口上报
     *   栈大小(Word)：够用即可，不必过小免得溢出
     *   ⚠ 必须检查返回值！之前堆太小(8000B)导致后两个任务创建失败
     *     xTaskCreate 返回 errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY，
     *     而代码没检查，任务静默消失(症状: UART 一直不打印)。
     *   现在堆已增大到 14000B，此处仍保留检查，失败时可在串口看到。 */
    if (xTaskCreate(EnvSensorTask,  "Sensor",  320, NULL, osPriorityAboveNormal, NULL) != pdPASS)
        printf("[ENV][ERR] Sensor task create failed!\r\n");
    if (xTaskCreate(EnvDisplayTask, "Display", 320, NULL, osPriorityNormal,      NULL) != pdPASS)
        printf("[ENV][ERR] Display task create failed!\r\n");
    if (xTaskCreate(EnvAlarmTask,   "Alarm",   320, NULL, osPriorityNormal,      &g_xAlarmTaskHandle) != pdPASS)
        printf("[ENV][ERR] Alarm task create failed!\r\n");
    if (xTaskCreate(EnvUARTTask,    "UART",    320, NULL, osPriorityBelowNormal, NULL) != pdPASS)
        printf("[ENV][ERR] UART task create failed!\r\n");

    OLED_ClearLine(0, 2);
    OLED_PrintString(0, 2, "System Ready!   ");
    OLED_ClearLine(0, 4);
    OLED_PrintString(0, 4, "4 Tasks Running");

    /* ── 5. 主监控循环：本任务自我降级，只定期看栈水位 ── */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
        {
            UBaseType_t n = uxTaskGetStackHighWaterMark(NULL);
            (void)n; /* 调试器里查看栈余量 */
        }
#endif
    }
}