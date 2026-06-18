/**
 * @file drv_servo_motor.c
 * @author oyyp
 * @brief 舵机驱动库
 * @version 0.1
 * @date 2026-06-18 0.1 init
 *
 * @copyright Copyright
 *
 */

 /* Includes ------------------------------------------------------------------*/
 #include "drv_servo_motor.h"
 
 
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
// 函数：设置舵机角度
// angle: 期望的角度，范围0-270
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint16_t Channel,uint16_t angle)
{
    if (angle > 270){angle = 270;}
    
    uint16_t pulse = (uint16_t)(50 + ((float)angle / 270.0f) * 200); 
    
    // 调用HAL库函数更新指定通道的CCR值
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse); //TIM_CHANNEL_1
}

/* Function prototypes -------------------------------------------------------*/


 
 