/**
 * @file    freertos.c
 * @brief   FreeRTOS 入口 —— 智能环境监测系统
 * @author  Mosongping
 * @date    2025-07
 *
 * 本文件是 CubeMX 自动生成的 freeertos.c 的"替换版"。
 * 作用：系统上电后，由 MX_FREERTOS_Init() 创建环境监测主任务 EnvMonitorTask，
 *       真正的 4 个子任务、队列、互斥量、软件定时器都在 env_monitor.c 里创建，
 *       这里只负责"点火"，不重复初始化外设（否则 OLED 会被初始化两次）。
 *
 * ⚠ 使用方式（替换到你的工程）：
 *   把这个文件覆盖到  .../02_nwatch_game_freertos/Core/Src/freertos.c
 *   然后记得把  nwatch/env_monitor.c 加入 Keil 工程，否则链接不到 EnvMonitorTask。
 *
 * ■ 本文件额外补了模板缺的几个符号（否则链接报错，都是模板残留不是你的错）：
 *   1. configureTimerForRunTimeStats / getRunTimeCounterValue
 *      -> FreeRTOSConfig.h 开了 configGENERATE_RUN_TIME_STATS ，
 *         FreeRTOS 强制要求这两个函数，这里给空实现。
 *   2. GetI2C / PutI2C + g_xI2CMutex
 *      -> driver_mpu6050.c 和 nwatch/draw.c 用这俩保护 I2C 总线，
 *         这里补一个互斥量实现（环境监测本身不用，但它俩要链接）。
 */

/* 驱动头文件 */
#include "driver_led.h"
#include "driver_oled.h"
#include "driver_dht11.h"
#include "driver_light_sensor.h"
#include "driver_ir_obstacle.h"
#include "driver_ultrasonic_sr04.h"
#include "driver_active_buzzer.h"
#include "driver_uart.h"

/* FreeRTOS 头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* CubeMX / CMSIS 头文件 */
#include "main.h"
#include "cmsis_os.h"
#include "env_monitor.h"

#include <stdio.h>   /* vApplicationIdleHook 里用了 printf */

/* ── 环境监测主任务（定义在 env_monitor.c） ── */
extern void EnvMonitorTask(void *params);
void StartDefaultTask(void *argument);

/* ─────────────────────────────────────────
 * 补：模板残留符号 / I2C 总线互斥量
 *   driver_mpu6050.c 和 nwatch/draw.c 都 extern 引用了 GetI2C/PutI2C，
 *   必须在这里提供定义，否则链接报 L6218E。
 * ───────────────────────────────────────── */
static SemaphoreHandle_t g_xI2CMutex;   /* 保护 I2C 总线的互斥量 */

void GetI2C(void)
{
    xSemaphoreTake(g_xI2CMutex, portMAX_DELAY);
}

void PutI2C(void)
{
    xSemaphoreGive(g_xI2CMutex);
}

/* ─────────────────────────────────────────
 * 补：运行时统计所需函数
 *   FreeRTOSConfig.h 定义了
 *     portCONFIGURE_TIMER_FOR_RUN_TIME_STATS = configureTimerForRunTimeStats
 *     portGET_RUN_TIME_COUNTER_VALUE        = getRunTimeCounterValue
 *   开了 configGENERATE_RUN_TIME_STATS 后 FreeRTOS 会引用它们，不给就链接报错。
 *   这里给空实现；若想真用运行时统计，可换成基于 DWT/SysTick 的计数。
 * ───────────────────────────────────────── */
__weak void configureTimerForRunTimeStats(void)
{
}

__weak unsigned long getRunTimeCounterValue(void)
{
    return 0;
}

/* ── Idle Hook：改为空实现（重要！） ──
 *   原来的实现会调用 vTaskGetRunTimeStats 在串口里疯狂打印任务统计表，
 *   只要 CPU 一空闲就刷一张表，几秒就把串口助手刷爆导致"卡死"，
 *   也把环境数据帧全部淹没。这里改成空实现，保留该函数符号避免链接报错，
 *   但不再向串口打印任何内容。串口只输出 env_monitor.c 里的干净数据帧。 */
void vApplicationIdleHook(void)
{
    /* 空实现：不做任何事，避免空闲时刷屏打统计表 */
}

/**
 * @brief   FreeRTOS 初始化入口
 *          由 main() 调用，只做"点火"：创建 I2C 互斥量 + 环境监测主任务。
 */
void MX_FREERTOS_Init(void)
{
    /* 补：创建 I2C 总线互斥量（给 driver_mpu6050.c / draw.c 的 GetI2C/PutI2C 用） */
    g_xI2CMutex = xSemaphoreCreateMutex();
    configASSERT(g_xI2CMutex != NULL);

    /* 环境监测主任务：内部会初始化所有外设 + 4个子任务 */
    xTaskCreate(EnvMonitorTask, "EnvMonitor", 512, NULL,
                osPriorityNormal, NULL);

    /* 默认任务：等系统起来后再删除自己（用于启动阶段收尾） */
    xTaskCreate(StartDefaultTask, "Default", 128, NULL,
                osPriorityIdle, NULL);
}

/**
 * @brief   默认任务 —— 启动后即自我删除
 *           实际功能全部由 env_monitor.c 里的 4 个子任务负责。
 */
void StartDefaultTask(void *argument)
{
    (void)argument;

    /* 给系统一点启动时间 */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 自我删除，释放资源 */
    vTaskDelete(NULL);
}
