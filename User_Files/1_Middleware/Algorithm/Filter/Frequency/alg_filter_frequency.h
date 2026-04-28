/**
 * @file alg_filter_frequency.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 频率滤波器
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-25 1.1 可自定义滤波器阶数
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ALG_FILTER_FREQUENCY_H
#define ALG_FILTER_FREQUENCY_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

extern float FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY;

#define FILTER_FREQUENCY_MAX_ORDER 50U

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 滤波器类型
 *
 */
typedef enum Enum_Filter_Frequency_Type
{
    Filter_Frequency_Type_LOWPASS = 0,
    Filter_Frequency_Type_HIGHPASS,
    Filter_Frequency_Type_BANDPASS,
    Filter_Frequency_Type_BANDSTOP,
} Enum_Filter_Frequency_Type;

/**
 * @brief 频率滤波器实例(纯C版本)
 *
 */
typedef struct Struct_Filter_Frequency
{
    float Value_Constrain_Low;
    float Value_Constrain_High;
    Enum_Filter_Frequency_Type Filter_Frequency_Type;
    float Frequency_Low;
    float Frequency_High;
    float Sampling_Frequency;
    uint32_t Order;
    float System_Function[FILTER_FREQUENCY_MAX_ORDER + 1U];
    float Input_Signal[FILTER_FREQUENCY_MAX_ORDER + 1U];
    uint32_t Signal_Flag;
    float Out;
} Struct_Filter_Frequency;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化滤波器
 * @param __Value_Constrain_Low 滤波器最小值, 全0不限制
 * @param __Value_Constrain_High 滤波器最大值, 全0不限制
 * @param __Filter_Frequency_Type 滤波器类型
 * @param __Frequency_Low 滤波器特征低频, 非高通有效
 * @param __Frequency_High 滤波器特征高频, 非低通有效
 * @param __Sampling_Frequency 滤波器采样频率
 * @param __Order 滤波器阶数, 范围0~FILTER_FREQUENCY_MAX_ORDER
 */
static inline void Filter_Frequency_Init(Struct_Filter_Frequency *__Filter,
                                         const float __Value_Constrain_Low,
                                         const float __Value_Constrain_High,
                                         const Enum_Filter_Frequency_Type __Filter_Frequency_Type,
                                         const float __Frequency_Low,
                                         const float __Frequency_High,
                                         const float __Sampling_Frequency,
                                         uint32_t __Order)
{
    uint32_t i;
    float system_function_sum = 0.0f;
    float omega_low;
    float omega_high;

    if (__Filter == NULL)
    {
        return;
    }

    if (__Order > FILTER_FREQUENCY_MAX_ORDER)
    {
        __Order = FILTER_FREQUENCY_MAX_ORDER;
    }

    memset(__Filter, 0, sizeof(Struct_Filter_Frequency));
    __Filter->Value_Constrain_Low = __Value_Constrain_Low;
    __Filter->Value_Constrain_High = __Value_Constrain_High;
    __Filter->Filter_Frequency_Type = __Filter_Frequency_Type;
    __Filter->Frequency_Low = __Frequency_Low;
    __Filter->Frequency_High = __Frequency_High;
    __Filter->Sampling_Frequency = __Sampling_Frequency;
    __Filter->Order = __Order;

    if (__Filter->Sampling_Frequency <= 0.0f)
    {
        __Filter->Sampling_Frequency = FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY;
    }

    omega_low = 2.0f * PI * __Filter->Frequency_Low / __Filter->Sampling_Frequency;
    omega_high = 2.0f * PI * __Filter->Frequency_High / __Filter->Sampling_Frequency;

    switch (__Filter->Filter_Frequency_Type)
    {
    case Filter_Frequency_Type_LOWPASS:
    {
        for (i = 0; i <= __Filter->Order; i++)
        {
            __Filter->System_Function[i] = omega_low / PI * Basic_Math_Sinc(((float)i - __Filter->Order / 2.0f) * omega_low);
        }

        break;
    }
    case Filter_Frequency_Type_HIGHPASS:
    {
        for (i = 0; i <= __Filter->Order; i++)
        {
            __Filter->System_Function[i] = Basic_Math_Sinc(((float)i - __Filter->Order / 2.0f) * PI) - omega_high / PI * Basic_Math_Sinc(((float)i - __Filter->Order / 2.0f) * omega_high);
        }

        break;
    }
    case Filter_Frequency_Type_BANDPASS:
    {
        for (i = 0; i <= __Filter->Order; i++)
        {
            __Filter->System_Function[i] = omega_high / PI * Basic_Math_Sinc(((float)i - __Filter->Order / 2.0f) * omega_high) - omega_low / PI * Basic_Math_Sinc(((float)i - __Filter->Order / 2.0f) * omega_low);
        }

        break;
    }
    case Filter_Frequency_Type_BANDSTOP:
    {
        for (i = 0; i <= __Filter->Order; i++)
        {
            __Filter->System_Function[i] = Basic_Math_Sinc(((float)i - __Filter->Order / 2.0f) * PI) + omega_low / PI * Basic_Math_Sinc(((float)i - __Filter->Order / 2.0f) * omega_low) - omega_high / PI * Basic_Math_Sinc(((float)i - __Filter->Order / 2.0f) * omega_high);
        }

        break;
    }
    default:
        break;
    }

    for (i = 0; i <= __Filter->Order; i++)
    {
        system_function_sum += __Filter->System_Function[i];
    }

    if (system_function_sum != 0.0f)
    {
        for (i = 0; i <= __Filter->Order; i++)
        {
            __Filter->System_Function[i] /= system_function_sum;
        }
    }
}

/**
 * @brief 滤波器调整值, 周期与采样周期相同
 */
static inline void Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(Struct_Filter_Frequency *__Filter)
{
    uint32_t i;

    if (__Filter == NULL)
    {
        return;
    }

    __Filter->Out = 0.0f;

    for (i = 0; i <= __Filter->Order; i++)
    {
        __Filter->Out += __Filter->System_Function[i] * __Filter->Input_Signal[(__Filter->Signal_Flag + i) % (__Filter->Order + 1U)];
    }
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
static inline float Filter_Frequency_Get_Out(const Struct_Filter_Frequency *__Filter)
{
    if (__Filter == NULL)
    {
        return 0.0f;
    }

    return __Filter->Out;
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
static inline void Filter_Frequency_Set_Now(Struct_Filter_Frequency *__Filter, const float __Now)
{
    float now_value;

    if (__Filter == NULL)
    {
        return;
    }

    now_value = __Now;

    if (__Filter->Value_Constrain_Low != 0.0f || __Filter->Value_Constrain_High != 0.0f)
    {
        now_value = Basic_Math_Constrain(__Now, __Filter->Value_Constrain_Low, __Filter->Value_Constrain_High);
    }

    __Filter->Input_Signal[__Filter->Signal_Flag] = now_value;
    __Filter->Signal_Flag++;

    if (__Filter->Signal_Flag >= (__Filter->Order + 1U))
    {
        __Filter->Signal_Flag = 0U;
    }
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/