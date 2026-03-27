/**
 * @file sys_timestamp.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 时间计算相关支持库
 * @version 0.1
 * @date 2025-08-16 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/**
 * 要求: 使能并绑定一个定时器, 开外部中断, PSC分频到1MHz, ARR为3600000000
 * 保证arr计数器1us增一次, 1h触发一次外部中断
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_timestamp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_Timestamp SYS_Timestamp = {0};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化时间戳
 *
 * @param __TIM_Manage_Object 绑定的定时器, psc后为1MHz, arr要求是3 600 000 000, arr至少32位的寄存器
 */
void Timestamp_Init(TIM_HandleTypeDef *htim)
{
    SYS_Timestamp.TIM_Handler = htim;
}

/**
 * @brief TIM定时器回调函数, 每3600s调用一次
 *
 */
void Timestamp_TIM_3600s_PeriodElapsedCallback(void)
{
    SYS_Timestamp.TIM_Overflow_Count++;
}

/**
 * @brief 计算当前时间戳, 单位微秒
 *
 * @return uint64_t 当前时间戳
 */
static uint64_t Timestamp_Calculate(void)
{
    uint32_t arr_counter = 0U;

    if (SYS_Timestamp.TIM_Handler != NULL)
    {
        arr_counter = SYS_Timestamp.TIM_Handler->Instance->CNT;
    }

    return (uint64_t)(SYS_Timestamp.TIM_Overflow_Count) * 3600000000ULL + (uint64_t)arr_counter;
}

uint64_t Timestamp_Get_Current_Timestamp(void)
{
    return Timestamp_Calculate();
}

float Timestamp_Get_Now_Second(void)
{
    return (float)Timestamp_Calculate() / 1000000.0f;
}

float Timestamp_Get_Now_Millisecond(void)
{
    return (float)Timestamp_Calculate() / 1000.0f;
}

uint64_t Timestamp_Get_Now_Microsecond(void)
{
    return Timestamp_Calculate();
}

/**
 * @brief 延迟指定秒数
 *
 * @param Second 延迟秒数
 */
void Timestamp_Delay_Second(uint32_t second)
{
    volatile uint64_t start_time = Timestamp_Get_Current_Timestamp();

    while ((uint64_t)(second) * 1000000ULL + start_time > Timestamp_Get_Current_Timestamp())
    {
    }
}

/**
 * @brief 延迟指定毫秒数
 *
 * @param Millisecond 延迟毫秒数
 */
void Timestamp_Delay_Millisecond(uint32_t millisecond)
{
    volatile uint64_t start_time = Timestamp_Get_Current_Timestamp();

    while ((uint64_t)(millisecond) * 1000ULL + start_time > Timestamp_Get_Current_Timestamp())
    {
    }
}

/**
 * @brief 延迟指定微秒数
 *
 * @param Microsecond 延迟微秒数
 */
void Timestamp_Delay_Microsecond(uint32_t microsecond)
{
    volatile uint64_t start_time = Timestamp_Get_Current_Timestamp();

    while ((uint64_t)(microsecond) + start_time > Timestamp_Get_Current_Timestamp())
    {
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/