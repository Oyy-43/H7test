/**
 * @file sys_timestamp.h
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

#ifndef BSP_TIMESTAMP_H
#define BSP_TIMESTAMP_H

/* Includes ------------------------------------------------------------------*/

#include "tim.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "robot_def.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef struct Struct_Timestamp
{
    TIM_HandleTypeDef *TIM_Handler;
    uint32_t TIM_Overflow_Count;
} Struct_Timestamp;

/* Exported variables --------------------------------------------------------*/

#ifdef CPP_ENABLE
extern "C"
{
#endif

extern Struct_Timestamp SYS_Timestamp;

void Timestamp_Init(TIM_HandleTypeDef *htim);
void Timestamp_TIM_3600s_PeriodElapsedCallback(void);
uint64_t Timestamp_Get_Current_Timestamp(void);
float Timestamp_Get_Now_Second(void);
float Timestamp_Get_Now_Millisecond(void);
uint64_t Timestamp_Get_Now_Microsecond(void);
void Timestamp_Delay_Second(uint32_t second);
void Timestamp_Delay_Millisecond(uint32_t millisecond);
void Timestamp_Delay_Microsecond(uint32_t microsecond);

#ifdef CPP_ENABLE
}

class Class_Timestamp
{
public:
    inline void Init(TIM_HandleTypeDef *htim)
    {
        Timestamp_Init(htim);
    }

    inline uint64_t Get_Current_Timestamp() const
    {
        return Timestamp_Get_Current_Timestamp();
    }

    inline float Get_Now_Second() const
    {
        return Timestamp_Get_Now_Second();
    }

    inline float Get_Now_Millisecond() const
    {
        return Timestamp_Get_Now_Millisecond();
    }

    inline uint64_t Get_Now_Microsecond() const
    {
        return Timestamp_Get_Now_Microsecond();
    }

    inline void TIM_3600s_PeriodElapsedCallback()
    {
        Timestamp_TIM_3600s_PeriodElapsedCallback();
    }
};

namespace Namespace_SYS_Timestamp
{
    inline void Delay_Second(const uint32_t &second)
    {
        Timestamp_Delay_Second(second);
    }

    inline void Delay_Millisecond(const uint32_t &millisecond)
    {
        Timestamp_Delay_Millisecond(millisecond);
    }

    inline void Delay_Microsecond(const uint32_t &microsecond)
    {
        Timestamp_Delay_Microsecond(microsecond);
    }
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/