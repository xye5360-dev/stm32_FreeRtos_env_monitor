/**
 * @file    env_monitor.h
 * @brief   基于FreeRTOS的智能环境监测系统 —— 头文件
 * @author  Mosongping
 * @date    2025-07
 *
 * 本文件是"环境监测_完整版"的总头文件。
 * 所有阈值、数据结构、状态枚举都在这里集中定义，
 * 其它源文件(env_monitor.c / freertos.c)只要 include 这一个就能拿到全部接口。
 *
 * 说明：本头文件保持与原工程接口兼容 (env_data_t / sys_status_t / EnvMonitorTask)，
 *      可以直接替换原 nwatch/env_monitor.h，不破坏其它依赖。
 */

#ifndef __ENV_MONITOR_H
#define __ENV_MONITOR_H

#include "cmsis_os.h"

/* ═══════════════════════════════════════════
 * 一、报警阈值定义（面试常被追问，集中放这里好改）
 * ═══════════════════════════════════════════ */
#define TEMP_HIGH_THRESHOLD        35      /* 高温报警阈值 (°C)  */
#define TEMP_LOW_THRESHOLD         5       /* 低温报警阈值 (°C)  */
#define HUMIDITY_HIGH_THRESHOLD    80      /* 高湿报警阈值 (%)   */
#define HUMIDITY_LOW_THRESHOLD     20      /* 低湿报警阈值 (%)   */
#define LIGHT_DARK_THRESHOLD       500     /* 光线过暗阈值 (ADC) */
#define OBSTACLE_DISTANCE_CM       15      /* 障碍物报警距离 (cm)*/

/* ═══════════════════════════════════════════
 * 二、系统运行参数
 * ═══════════════════════════════════════════ */
#define SENSOR_PERIOD_MS           2000    /* 传感器采集周期 2s  */
#define DISPLAY_PERIOD_MS          1000    /* OLED显示周期  1s  */
#define ALARM_PERIOD_MS            500     /* 报警检测周期  500ms */
#define UART_REPORT_PERIOD_MS      3000    /* 串口上报周期  3s   */
#define ALARM_DEBOUNCE            3       /* 去抖次数：连续3次才报警 (1.5s 窗口) */
#define QUEUE_DEPTH               1       /* 队列深度：只保留最新一帧 */
#define SENSOR_INVALID_VALUE     (-999)  /* 无效数据的标记值，界面显示成 "--" */

/* ═══════════════════════════════════════════
 * 三、环境数据结构体（队列里传递的就是它）
 * ═══════════════════════════════════════════ */
typedef struct {
    int      temperature;   /* 温度 (°C)。无效时为 SENSOR_INVALID_VALUE */
    int      humidity;      /* 湿度 (%)。无效时为 SENSOR_INVALID_VALUE */
    uint16_t light;         /* 光照强度 (ADC 原始值 0~4095) */
    uint16_t distance;      /* 超声波距离 (cm)。0=无有效目标 */
    uint8_t  obstacle;      /* 红外避障：0=有障碍物，1=无障碍 */
} env_data_t;

/* ═══════════════════════════════════════════
 * 四、系统状态枚举（OLED 状态行 + 报警调度共用）
 * ═══════════════════════════════════════════ */
typedef enum {
    SYS_NORMAL          = 0,   /* 一切正常 */
    SYS_TEMP_HIGH,             /* 高温 */
    SYS_TEMP_LOW,              /* 低温 */
    SYS_HUMIDITY_HIGH,         /* 高湿 */
    SYS_HUMIDITY_LOW,          /* 低湿 */
    SYS_LIGHT_DARK,            /* 光线过暗 */
    SYS_OBSTACLE_DETECTED      /* 检测到障碍物 */
} sys_status_t;

/* ═══════════════════════════════════════════
 * 五、全局函数声明
 * ═══════════════════════════════════════════ */
void EnvMonitorTask(void *params);   /* 主任务：初始化全部资源，再创建4个子任务 */

#endif /* __ENV_MONITOR_H */
