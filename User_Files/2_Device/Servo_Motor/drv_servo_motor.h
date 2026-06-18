#ifndef _DRV_SERVO_MOTOR_H_
#define _DRV_SERVO_MOTOR_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"



/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint16_t Channel,uint16_t angle);

#endif //  _DRV_SERVO_MOTOR_H_