#ifndef __CTRL_MOTOR_DM_H
#define __CTRL_MOTOR_DM_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crsf.h"
#include "alg_filter_frequency.h"
#include "alg_sin.h"
#include "drv_motor_dm.h"
#include "drv_uart.h"
#include "alg_pid.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include "alg_transData.h"
#include "Lift.h"
#include "robot_def.h"
#include "withPC.h"
#include "arm.h"
#include "get_Weapon.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    float amplitude;
    float phase;
    float offset;
} GravityFeedforward_Config; //重力前馈拟合曲线

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern PID_TypeDef Motor_DM_1_To_4_PID[2];

/* Exported function declarations --------------------------------------------*/

void DM_Motor_Output();
void Motor_DM_Normal_Output();
void Motor_DM_InitPID();

#endif /* __CTRL_MOTOR_DM_H */