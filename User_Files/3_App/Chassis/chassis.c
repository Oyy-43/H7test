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


/* Private macros ------------------------------------------------------------*/
#define Turn_KP 0.15f
#define Turn_KI 0.0f
#define Turn_KD 0.0f
#define Turn_Kf 0.018f 

/* Private types -------------------------------------------------------------*/
//转向控制PID
PID_TypeDef Turn_PID;
Struct_Filter_Frequency Chassis_Vz_Fillter;

/* Private variables ---------------------------------------------------------*/
float test_yaw = 0.0f;
float vz_turn_cmd = 0.0f;
float target_y =0.0f;
/* Private function declarations ---------------------------------------------*/


void Chassis_Turing_Init()
{
  PID_Init(&Turn_PID,1.25f,0.0f,0.0f,Turn_KP, Turn_KI, Turn_KD, Turn_Kf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f,Integral_Limit);
  Filter_Frequency_Init(&Chassis_Vz_Fillter,
      0.0f,
      0.0f,
      Filter_Frequency_Type_LOWPASS,
      10.0f,
      FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f,
      1000.0f,
      1U);

}

void Chassis_Omega_update(float vx, float vy, float vz)
{
  const float inv_mecanum_r = 2.0f / chassis_d;
  const float wz_term = vz * (chassis_a + chassis_b);

  DJI_Motor_Instances[0].Target_Omega =  ((vx - vy + wz_term) * inv_mecanum_r);
  DJI_Motor_Instances[1].Target_Omega =  ((vx + vy + wz_term) * inv_mecanum_r);
  DJI_Motor_Instances[2].Target_Omega =  ((-vx + vy + wz_term) * inv_mecanum_r);
  DJI_Motor_Instances[3].Target_Omega =  ((-vx - vy + wz_term) * inv_mecanum_r);
  // 4/5号轮驱动X向分量，同时叠加绕中心旋转的切向速度分量
  // DJI_Motor_Instances[4].Target_Omega =  wheel4_x_omega;
  // DJI_Motor_Instances[5].Target_Omega =  wheel5_x_omega;
}

void Chassis_Control()
{
  float  vx_cmd = 0.0f, vy_cmd = 0.0f, wz_cmd = 0.0f;

  // 将目标航向限制在 [0, 360) 范围内
  if (Target_Yaw >= 360.0f) Target_Yaw -= 360.0f;
  else if (Target_Yaw < 0.0f) Target_Yaw += 360.0f;

  // 角度误差归一化：将误差限制在 [-180, 180] 范围内
  // 避免陀螺仪 360°→0° 跳变导致 PID 误差突变

  // float yaw_measure = hipnuc_imu_data.eul[2];
  Filter_Frequency_Set_Now(&Chassis_Vz_Fillter, hipnuc_imu_data.eul[2]);
  Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Chassis_Vz_Fillter);
  float yaw_measure =Filter_Frequency_Get_Out(&Chassis_Vz_Fillter);
  float yaw_error = Basic_Math_Modulus_Normalization(Target_Yaw - yaw_measure, 360.0f);
  float effective_target = yaw_measure + yaw_error;

  switch(Robot_Mode)
  {
    case Robot_Mode_Stop:
      vx_cmd = 0.0f;
      vy_cmd = 0.0f;
      wz_cmd = 0.0f;
      vz_turn_cmd = 0.0f;
      Chassis_Omega_update(0.0f, 0.0f, 0.0f);
      return;
    break;
    case Robot_Mode_Manual:
      vx_cmd = rc_channels.ch[1] * 0.05f / 10.0f / 4.0f;
      vy_cmd = -rc_channels.ch[0] * 0.05f / 10.0f / 4.0f;
      // Target_Yaw -= rc_channels.ch[3] * 0.01f / 20.0f;
      if(rc_channels.ch[3] > 2)
      {
        Target_Yaw -= 0.025f;
      }
      else if(rc_channels.ch[3] < -2)
      {
        Target_Yaw += 0.025f;
      }
    break;
    case Robot_Mode_Auto:
      switch (LiftingState_t.state)
      {
        case No_Lifting:
          vx_cmd = PC_frame.cmd_vx;
          vy_cmd = PC_frame.cmd_vy;
          // wz_cmd = PC_frame.cmd_vz;
        break;
        case LiftLevel200_Step1:
        case LiftLevel200_Step2:
        case LiftLevel200_Step3:
        case LiftLevel200_Step4:
        case LiftLevel200_Step5:
        case LiftLevel200_Step6:
        case LiftLevel200_Step7:
          vx_cmd = LiftStand_Speedvx;
          vy_cmd = LiftStand_Speedvy;
          // wz_cmd = LiftStand_Speedvz;
        break;
      }
    break;
  }
  vz_turn_cmd = PID_Calculate(&Turn_PID, yaw_measure, effective_target, 0.001f);
  Chassis_Omega_update(vx_cmd, vy_cmd, -vz_turn_cmd);
}


void Chassis_Task(void *argument)
{ 
    Chassis_Turing_Init();
    while (1)
    {
      Chassis_Control();
      osDelay(1); // 每1ms更新一次
    }
}


/* Function prototypes -------------------------------------------------------*/

