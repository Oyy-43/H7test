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
  const float wz_term = vz * (chassis_a + chassis_b);
  const float inv_omni_r = 2.0f / chassis_omni_d;
  const float wheel4_x_omega = (-vx - vz * chassis_omni_b) * inv_omni_r;
  const float wheel5_x_omega = (vx + vz * chassis_omni_b) * inv_omni_r;

  DM_Motor_1to4_Instances[0].Target_Omega =  ((-vx + vy + wz_term) * inv_mecanum_r);
  DM_Motor_1to4_Instances[1].Target_Omega =  ((-vx - vy + wz_term) * inv_mecanum_r);
  DM_Motor_1to4_Instances[2].Target_Omega =  ((vx - vy + wz_term) * inv_mecanum_r);
  DM_Motor_1to4_Instances[3].Target_Omega =  ((vx + vy + wz_term) * inv_mecanum_r);
  // 4/5号轮驱动X向分量，同时叠加绕中心旋转的切向速度分量
  DM_Motor_1to4_Instances[4].Target_Omega =  wheel4_x_omega;
  DM_Motor_1to4_Instances[5].Target_Omega =  wheel5_x_omega;
}

void Chassis_Control()
{
  if(rc_channels.ch[4]<=0)
  {
    Chassis_Omega_update(0,0,0);
  }
  else
  {
    const float vx_cmd = rc_channels.ch[1] * 0.05f / 10.0f / 8.0f;
    const float vy_cmd = -rc_channels.ch[0] * 0.05f / 10.0f / 8.0f;
    const float wz_cmd = -rc_channels.ch[3] * 0.01f / 15.0f;

    Chassis_Omega_update(vx_cmd, vy_cmd, wz_cmd);
  }
  
}


void Chassis_Task(void *argument)
{
    while (1)
    {
      Chassis_Control();
      osDelay(1); // 每1ms更新一次
    }
}


/* Function prototypes -------------------------------------------------------*/

