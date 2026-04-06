#ifndef __DRV_ADC_H
#define __DRV_ADC_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define ADC_BUFFER_SIZE 128
/* Exported types ------------------------------------------------------------*/

/**
 * @brief ADC采样信息结构体
 *
 */
typedef struct Struct_ADC_Manage_Object
{
    ADC_HandleTypeDef *ADC_Handler;
    uint16_t ADC_Data[ADC_BUFFER_SIZE];
} Struct_ADC_Manage_Object;


/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern struct Struct_ADC_Manage_Object ADC1_Manage_Object;
extern struct Struct_ADC_Manage_Object ADC2_Manage_Object;
extern struct Struct_ADC_Manage_Object ADC3_Manage_Object;


/* Exported function declarations --------------------------------------------*/

void ADC_Init(ADC_HandleTypeDef *hadc, uint16_t Sample_Number);


#endif /* __DRV_ADC_H */
