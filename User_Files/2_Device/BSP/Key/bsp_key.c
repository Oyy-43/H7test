/* Includes ------------------------------------------------------------------*/
#include "bsp_key.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
BSP_KEY_S BSP_Key = {
	.Pre_GPIO_State = GPIO_PIN_SET,
	.Now_GPIO_State = GPIO_PIN_SET,
	.Key_Status = BSP_Key_Status_FREE,
};

/* Private variables ---------------------------------------------------------*/
const GPIO_PinState KEY_FREE_STATE = GPIO_PIN_SET;

/* Private function declarations ---------------------------------------------*/
void BSP_Key_Init()
{
    BSP_Key.Pre_GPIO_State = HAL_GPIO_ReadPin(KEY__INPUT_GPIO_Port, KEY__INPUT_Pin);
}

void BSP_Key_TIM_1ms_Process_PeriodElapsedCallback()
{
if (BSP_Key.Pre_GPIO_State == KEY_FREE_STATE)
    {
        if (BSP_Key.Now_GPIO_State == KEY_FREE_STATE)
        {
            BSP_Key.Key_Status = BSP_Key_Status_FREE;
        }
        else
        {
            BSP_Key.Key_Status = BSP_Key_Status_TRIG_FREE_PRESSED;
        }
    }
    else
    {
        if (BSP_Key.Now_GPIO_State == KEY_FREE_STATE)
        {
            BSP_Key.Key_Status = BSP_Key_Status_TRIG_PRESSED_FREE;
        }
        else
        {
            BSP_Key.Key_Status = BSP_Key_Status_PRESSED;
        }
    }

    BSP_Key.Pre_GPIO_State = BSP_Key.Now_GPIO_State;
}

void BSP_Key_TIM_50ms_Read_PeriodElapsedCallback()
{
    BSP_Key.Now_GPIO_State = HAL_GPIO_ReadPin(KEY__INPUT_GPIO_Port, KEY__INPUT_Pin);
}

/* Function prototypes -------------------------------------------------------*/

