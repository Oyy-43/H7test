/**
 * @file chassis.c
 * @author Oyyp
 * @brief 底盘任务编写
 * @version 0.1
 * @date 2026-04-04 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis.h"
#include "ctrl_motor_dm.h"
#include "drv_motor_dm.h"
#include "alg_basic.h"

/* Private macros ------------------------------------------------------------*/


/* Private types -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
void Chassis_Omega_update(float vx, float vy, float vz)
{
  const float inv_mecanum_r = 2.0f / chassis_d;
  const float inv_omni_r = 2.0f / chassis_omni_d;
  const float wz_term = vz * (chassis_a + chassis_b);

  DM_Motor_1to4_Instances[0].Target_Omega = chassis_motor0_dir * ((-vx + vy + wz_term) * inv_mecanum_r);
  DM_Motor_1to4_Instances[1].Target_Omega = chassis_motor1_dir * ((-vx - vy + wz_term) * inv_mecanum_r);
  DM_Motor_1to4_Instances[2].Target_Omega = chassis_motor2_dir * ((vx - vy + wz_term) * inv_mecanum_r);
  DM_Motor_1to4_Instances[3].Target_Omega = chassis_motor3_dir * ((vx + vy + wz_term) * inv_mecanum_r);
//两个全向轮
  DM_Motor_1to4_Instances[4].Target_Omega = chassis_motor4_dir * ((vx - vz * chassis_omni_b) * inv_omni_r);
  DM_Motor_1to4_Instances[5].Target_Omega = chassis_motor5_dir * ((vx + vz * chassis_omni_b) * inv_omni_r);
}

void Chassis_Control()
{
  if(rc_channels.ch[4]<0)
  {
    Chassis_Omega_update(0,0,0);
  }
  else
  {
    Chassis_Omega_update(rc_channels.ch[2]*0.05f, -rc_channels.ch[3]*0.05f, -rc_channels.ch[0]*0.01f);
  }
  
  
}


void Chassis_Task(void *argument)
{
    while (1)
    {
      // Chassis_Control();
      osDelay(1); // 每10ms更新一次
    }
}


/* Function prototypes -------------------------------------------------------*/

