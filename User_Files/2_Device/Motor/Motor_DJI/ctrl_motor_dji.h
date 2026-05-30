#ifndef _CTRL_MOTOR_DJI_H
#define _CTRL_MOTOR_DJI_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "drv_can.h"
#include "drv_motor_dji.h"
#include "alg_pid.h"
#include "alg_sin.h"
#include "cmsis_os2.h"
#include "drv_uart.h"
#include "alg_transData.h"
#include "crsf.h"
#include "drv_motor_dm.h"
#include "ctrl_motor_dm.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern float Sin_Target;

/* Exported function declarations --------------------------------------------*/
void Motor_DJI_InitPID();
void DJI_Motor_Output();




#endif /* _CTRL_MOTOR_DJI_H */