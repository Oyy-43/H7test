#ifndef _BSP_POWER_H_
#define _BSP_POWER_H_
/* Includes ------------------------------------------------------------------*/
#include "drv_adc.h"
#include "main.h"
#include "gpio.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Specialized, 板载电源相关, 包括两个24V输出和一个5V输出以及一个电源检测器
 *
 */
typedef struct Power
{
    Struct_ADC_Manage_Object *ADC_Manage_Object;
    uint16_t ADC_Buffer_Offset;
}Power;


/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern Power Bsp_Power;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电源电压
 *
 * @return float 电源电压, 单位V
 */
static inline float BSP_Power_Get_Power_Voltage() 
{
    // 读取ADC值, 偏移量为0
    uint32_t adc_value = Bsp_Power.ADC_Manage_Object->ADC_Data[Bsp_Power.ADC_Buffer_Offset];
    // 计算电压, 假设ADC分辨率为16位,
    float voltage = ((float) (adc_value) / (float) (1 << 16)) * 3.3f * 11.0f;
    // 返回电压值
    return (voltage);
}

/**
 * @brief 设置DC24V_0输出(PC13)
 *
 * @param __DC24_0 true: 输出, false: 不输出
 */
static inline void BSP_Power_Set_DC24_0(bool __Status_PC13)
{
    HAL_GPIO_WritePin(DC24_0__OUTPUT_GPIO_Port, DC24_0__OUTPUT_Pin, __Status_PC13 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 设置DC24V_1输出(PC14)
 *
 * @param __DC24_1 true: 输出, false: 不输出
 */
static inline void BSP_Power_Set_DC24_1(bool __Status_PC14)
{
    HAL_GPIO_WritePin(DC24_1__OUTPUT_GPIO_Port, DC24_1__OUTPUT_Pin, __Status_PC14 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 设置DC5V输出(PC15)
 *
 * @param __DC5 true: 输出, false: 不输出
 */
static inline void BSP_Power_Set_DC5(bool __Status_DC5)
{
    HAL_GPIO_WritePin(DC5__OUTPUT_GPIO_Port, DC5__OUTPUT_Pin, __Status_DC5 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_Power_Init(const bool __DC24_0_Status, const bool __DC24_1_Status, const bool __DC5_Status);






#endif /* _BSP_POWER_H_ */
