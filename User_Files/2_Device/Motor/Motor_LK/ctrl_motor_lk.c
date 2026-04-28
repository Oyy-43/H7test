/**
 * @file ctrl_motor_lk.c
 * @author Oyyp
 * @brief 翎控电机控制
 * @version 0.1
 * @date 2026-04-16 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "ctrl_motor_lk.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
PID_TypeDef Motor_LK_SPEED_PID[LK_Motor_Num];
PID_TypeDef Motor_LK_POS_PID[LK_Motor_Num];

/* Private variables ---------------------------------------------------------*/
float LK_SPEEDPIDKP[LK_Motor_Num] = {1.0f, 1.0f};
float LK_SPEEDPIDKI[LK_Motor_Num] = {0.001f, 0.001f};
float LK_SPEEDPIDKD[LK_Motor_Num] = {0.0f, 0.0f};
float LK_SPEEDPIDKf[LK_Motor_Num] = {0.0f, 0.0f};
float LK_POSPIDKP[LK_Motor_Num] = {10.0f, 10.0f}; 
float LK_POSPIDKI[LK_Motor_Num] = {0.0f, 0.0f};
float LK_POSPIDKD[LK_Motor_Num] = {0.0f, 0.0f};
float LK_POSPIDKf[LK_Motor_Num] = {0.05f, 0.05f};
float speed_target = 0.0f;   // 目标速度

static float Motor_LK_GetForwardTarget(float feedback_angle, float target_angle)
{
    float feedback_mod;
    float target_mod;
    float delta;

    // Work in [0, 360) for boundary decision, but keep continuous output target.
    feedback_mod = Basic_Math_Modulus_Normalization(feedback_angle, 360.0f);
    if (feedback_mod < 0.0f)
    {
        feedback_mod += 360.0f;
    }

    target_mod = Basic_Math_Modulus_Normalization(target_angle, 360.0f);
    if (target_mod < 0.0f)
    {
        target_mod += 360.0f;
    }

    // Default: shortest-path error.
    delta = Basic_Math_Modulus_Normalization(target_mod - feedback_mod, 360.0f);

    // Only for 330 -> 0 crossing: force forward small-angle motion.
    if ((feedback_mod >= 330.0f) && (target_mod <= 30.0f) && (delta < 0.0f))
    {
        delta += 360.0f;
    }

    return (feedback_angle + delta);
}

/* Private function declarations ---------------------------------------------*/
void Motor_LK_InitPID()
{
    PID_Init(&Motor_LK_SPEED_PID[0], 2048.0f, 100.0f, 0.0f, LK_SPEEDPIDKP[0], LK_SPEEDPIDKI[0], LK_SPEEDPIDKD[0], LK_SPEEDPIDKf[0],10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&Motor_LK_POS_PID[0], 360.0f, 10.0f, 0.0f, LK_POSPIDKP[0], LK_POSPIDKI[0], LK_POSPIDKD[0], LK_POSPIDKf[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&Motor_LK_SPEED_PID[1], 2048.0f, 100.0f, 0.0f, LK_SPEEDPIDKP[1], LK_SPEEDPIDKI[1], LK_SPEEDPIDKD[1], LK_SPEEDPIDKf[1],10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
    PID_Init(&Motor_LK_POS_PID[1], 360.0f, 10.0f, 0.0f, LK_POSPIDKP[1], LK_POSPIDKI[1], LK_POSPIDKD[1], LK_POSPIDKf[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Integral_Limit);
}

void Motor_LK_CalPID()
{
    float pos_target_0;
    float pos_target_1;

    pos_target_0 = Motor_LK_GetForwardTarget(LK_Motor_Instances[0].Rx_Data.CircleAngle, LK_Motor_Instances[0].Target_Angle);
    pos_target_1 = Motor_LK_GetForwardTarget(LK_Motor_Instances[1].Rx_Data.CircleAngle, LK_Motor_Instances[1].Target_Angle);

    LK_Motor_Instances[0].Target_Omega = PID_Calculate(&Motor_LK_POS_PID[0], LK_Motor_Instances[0].Rx_Data.CircleAngle, pos_target_0, 0.001f);
    LK_Motor_Instances[0].Target_Current =(int16_t)PID_Calculate(&Motor_LK_SPEED_PID[0], LK_Motor_Instances[0].Rx_Data.Speed, LK_Motor_Instances[0].Target_Omega, 0.001f);
    LK_Motor_Instances[1].Target_Omega = PID_Calculate(&Motor_LK_POS_PID[1], LK_Motor_Instances[1].Rx_Data.CircleAngle, pos_target_1, 0.001f);
    LK_Motor_Instances[1].Target_Current =(int16_t)PID_Calculate(&Motor_LK_SPEED_PID[1], LK_Motor_Instances[1].Rx_Data.Speed, LK_Motor_Instances[1].Target_Omega, 0.001f);
}
/* Function prototypes -------------------------------------------------------*/

void LKsetOutput(void *argument)
{
    for(;;)
    {   
        Motor_LK_GetFeedback(&LK_Motor_Instances[0]);
        Motor_LK_GetFeedback(&LK_Motor_Instances[1]);
        LK_Motor_Instances[0].Target_Angle = (rc_channels.ch[11]+820)*0.2f; // 直接使用原始目标值，不进行滤波，确保控制环的单一数据来源
        LK_Motor_Instances[1].Target_Angle = (rc_channels.ch[2]+820)*0.2f; // 直接使用原始目标值，不进行滤波，确保控制环的单一数据来源
        Motor_LK_CalPID();
        if(rc_channels.ch[6]<=0)
        {
            Motor_LK_Stop(&LK_Motor_Instances[0]);
            Motor_LK_Stop(&LK_Motor_Instances[1]);
        }
        else
        {
            Motor_LK_TorqueControl(&LK_Motor_Instances[0], LK_Motor_Instances[0].Target_Current);
            Motor_LK_TorqueControl(&LK_Motor_Instances[1], LK_Motor_Instances[1].Target_Current);
        }
        osDelay(1);
    }
}

