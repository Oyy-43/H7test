/**
 * @file drv_adc.c
 * @author Oyy
 * @brief 借鉴于麻神
 * @version 0.1
 * @date 2026-04-05 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_adc.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Struct_ADC_Manage_Object ADC1_Manage_Object = {NULL};
Struct_ADC_Manage_Object ADC2_Manage_Object = {NULL};
Struct_ADC_Manage_Object ADC3_Manage_Object = {NULL};


/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 初始化ADC
 *
 * @param hadc ADC编号
 * @param Sample_Number 一周期被采样的数据数量, 通道数 * 每个通道的传输数
 */
void ADC_Init(ADC_HandleTypeDef *hadc, uint16_t Sample_Number)
{
    HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    if (hadc->Instance == ADC1)
    {
        ADC1_Manage_Object.ADC_Handler = hadc;
        HAL_ADC_Start_DMA(hadc, (uint32_t *) &ADC1_Manage_Object.ADC_Data, Sample_Number);
    }
    else if (hadc->Instance == ADC2)
    {
        ADC2_Manage_Object.ADC_Handler = hadc;
        HAL_ADC_Start_DMA(hadc, (uint32_t *) &ADC2_Manage_Object.ADC_Data, Sample_Number);
    }
    else if (hadc->Instance == ADC3)
    {
        ADC3_Manage_Object.ADC_Handler = hadc;
        HAL_ADC_Start_DMA(hadc, (uint32_t *) &ADC3_Manage_Object.ADC_Data, Sample_Number);
    }
}

