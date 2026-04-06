/**
 * @file bsp_power.c
 * @author Oyyp
 * @brief 借鉴于麻神
 * @version 0.1
 * @date 2026-04-05 0.1 5V,24V 输出初始化
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_power.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern Power Bsp_Power;
/* Private function declarations ---------------------------------------------*/
void BSP_Power_Init(const bool __DC24_0_Status, const bool __DC24_1_Status, const bool __DC5_Status)
{
    // Bsp_Power.ADC_Manage_Object = Bsp_Power.ADC_Manage_Object;
    // Bsp_Power.ADC_Buffer_Offset = Bsp_Power.ADC_Buffer_Offset;

    // 默认关闭24V输出
    BSP_Power_Set_DC24_0(__DC24_0_Status);
    BSP_Power_Set_DC24_1(__DC24_1_Status);
    BSP_Power_Set_DC5(__DC5_Status);
}


/* Function prototypes -------------------------------------------------------*/

