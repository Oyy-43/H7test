#ifndef __CTRL_MOTOR_DM_H
#define __CTRL_MOTOR_DM_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "drv_can.h"
#include "drv_motor_dm.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern DM_Motor_1to4_Instance g_dm_1to4_motors[4];


/* Exported function declarations --------------------------------------------*/
void Motor_DM_InitPID();

#endif /* __CTRL_MOTOR_DM_H */