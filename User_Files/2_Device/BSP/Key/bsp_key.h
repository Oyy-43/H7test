#ifndef _BSP_KEY_H
#define _BSP_KEY_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum 
{
    BSP_Key_Status_FREE = 0,
    BSP_Key_Status_TRIG_FREE_PRESSED,
    BSP_Key_Status_TRIG_PRESSED_FREE,
    BSP_Key_Status_PRESSED,
}Enum_BSP_Key_Status;

typedef struct
{
    GPIO_PinState Pre_GPIO_State;
    GPIO_PinState Now_GPIO_State;
    Enum_BSP_Key_Status Key_Status;
}BSP_KEY_S;

extern BSP_KEY_S BSP_Key;

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

// 上一次按键状态

// 当前按键状态

/* Exported function declarations --------------------------------------------*/
void BSP_Key_TIM_1ms_Process_PeriodElapsedCallback();
void BSP_Key_TIM_50ms_Read_PeriodElapsedCallback();


#endif // _BSP_KEY_H
