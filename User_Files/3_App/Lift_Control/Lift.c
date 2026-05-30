/**
 * @file Lift.c
 * @author name
 * @brief description
 * @version 0.1
 * @date 2026-05-02 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "Lift.h"


/* Private macros ------------------------------------------------------------*/
#define Turning_Time 1000.0f
#define Lift_toMaxSpeed -2.0f
#define LoweringSpeed 0.5f
#define Down_toMinSpeed 1.5f
#define LiftingSpeed -0.5f
#define BlockingDetect_MinTime 50.0f

/* Private types -------------------------------------------------------------*/
FSMstate MeasureMAXMIN_Front_t;
FSMstate MeasureMAXMIN_Back_t;
Event MeasureEvent_Front_t;
Event MeasureEvent_Back_t;  
/* Private variables ---------------------------------------------------------*/

float count;
volatile float Lowering2time = 0.0f;
volatile float Lowering3time = 0.0f;
volatile float Debug_PID_Output = 0.0f;
bool Front_Calibrated = false;
bool Back_Calibrated = false;
Normali_S remote_channel_ch2;
float test_remote_ch2;
/* Private function declarations ---------------------------------------------*/
void MeasureFSM_Init()
{
    FSM_Init(&MeasureMAXMIN_Front_t);
    FSM_Init(&MeasureMAXMIN_Back_t);
    MeasureEvent_Front_t.sig = MeasureEvent_None;
    MeasureEvent_Back_t.sig = MeasureEvent_None;
    Basic_Math_Modulus_Init(&remote_channel_ch2,820,-820);
}

/**
 * @brief 路径移动规划有限状态机运行函数
 * 
 */
void MeasureFSM_Run()
{
    MeasureFSM_Dispatch(&MeasureMAXMIN_Front_t, &MeasureEvent_Front_t, &Motor_DM_1_To_4_PID[0],&DM_Motor_1to4_Instances[0],&Front_Calibrated);
    MeasureFSM_Dispatch(&MeasureMAXMIN_Back_t, &MeasureEvent_Back_t, &Motor_DM_1_To_4_PID[1],&DM_Motor_1to4_Instances[1],&Back_Calibrated);
}


void MeasureFSM_Dispatch(FSMstate *me, Event *e,PID_TypeDef *pid,DM_Motor_1to4_Instance *motor_instance,bool *calibrated)
{
    static float Max1, Max2, Max3, Min1, Min2, Min3;
    MeasureEvent_Generate(me, e, pid);
    switch(me->state)
    {
        case WaitingForStart:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Start:
                me->state = Lifting_to_Max1;
                me->state_time = 0.0f;
                break;
            }
        break;
        case Lifting_to_Max1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                me->state = BlockingatMax1;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMax1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
                Max1 = motor_instance->Rx_Data.Total_Encoder;
                me->state = Lowering1;
                me->state_time = 0.0f;
                break;
            }
        break;
        case Lowering1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_LoweringTimeOut:
                me->state =Lifting_to_Max2;
                me->state_time = 0.0f;
                break;
            }
        break;
        case Lifting_to_Max2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                me->state = BlockingatMax2;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMax2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
                Max2 = motor_instance->Rx_Data.Total_Encoder;
                me->state = Lowering2;
                me->state_time = 0.0f;
                break;
            }
        break;
        case Lowering2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_LoweringTimeOut:
                me->state =Lifting_to_Max3;
                me->state_time = 0.0f;
                break;
            }
        break;
        case Lifting_to_Max3:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                me->state = BlockingatMax3;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMax3:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
                Max3 = motor_instance->Rx_Data.Total_Encoder;
                motor_instance->Encoder_Limit.Max = (Max1+Max2+Max3)/3.0f;
                me->state = Lowering_to_Min1;
                me->state_time = 0.0f;
                pid->ERRORHandler.ERRORCount = 0;
                break;
            }
        break;
        case Lowering_to_Min1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                me->state =BlockingatMin1;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMin1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
                Min1 = motor_instance->Rx_Data.Total_Encoder;
                me->state = Lifting1;
                me->state_time = 0.0f;
                pid->ERRORHandler.ERRORCount = 0;
                break;
            }
        break;
        case Lifting1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_LiftingTimeOut:
                me->state = Lowering_to_Min2;
                me->state_time = 0.0f;
                pid->ERRORHandler.ERRORCount = 0;
                break;
            }
        break;
        case Lowering_to_Min2:
            me->state_time+=1.0f;
            Lowering2time += 1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                me->state =BlockingatMin2;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMin2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
                Min2 = motor_instance->Rx_Data.Total_Encoder;
                me->state = Lifting2;
                me->state_time = 0.0f;
                pid->ERRORHandler.ERRORCount = 0;
                break;
            }
        break;
        case Lifting2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_LiftingTimeOut:
                me->state = Lowering_to_Min3;
                me->state_time = 0.0f;
                pid->ERRORHandler.ERRORCount = 0;
                break;
            }
        break;
        case Lowering_to_Min3:
            Lowering3time += 1.0f;
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                me->state =BlockingatMin3;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMin3:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
                Min3 = motor_instance->Rx_Data.Total_Encoder;
                motor_instance->Encoder_Limit.Min = (Min1+Min2+Min3)/3.0f;
                Basic_Math_Modulus_Init(&motor_instance->Encoder_Limit, motor_instance->Encoder_Limit.Max, motor_instance->Encoder_Limit.Min);
                me->state = done;
                me->state_time = 0.0f;
                break;
            }
        break;
        case done:
            *calibrated = true;
        break;
        default:
        break;
    }
}

bool Blocking_Check(PID_TypeDef *pid)
{
    float target_abs = ABS(pid->Target);

    if (ABS(pid->Output) < ABS(pid->MaxOut) * 0.01f)
    return false;

    if (target_abs < 1e-3f)
    {
        pid->ERRORHandler.ERRORCount = 0;
        return false;
    }

    if ((ABS(pid->Target - pid->Measure) / target_abs) > 0.9f)
    {
        //电机堵转计数
        pid->ERRORHandler.ERRORCount++;
        count++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 1000)
    {
        //电机堵转超过1000次
        return true;
    }
    return false;
}

void MeasureEvent_Generate(FSMstate *me, Event *e,PID_TypeDef *pid)
{
    e->sig = MeasureEvent_None;
    if(me->state==WaitingForStart && ch9_status.Key_Status==CH_Status_TRIG_FREE_PRESSED)
    {
        e->sig = MeasureEvent_Start;
    }
    if(me->state==Lifting_to_Max1 && (Blocking_Check(pid)))
    {
        e->sig = MeasureEvent_Blocking;
    }
    if(me->state==BlockingatMax1)
    {
        e->sig = MeasureEvent_BlockingTimeOut;
    }
    if(me->state==Lowering1 && me->state_time>Turning_Time)
    {
        e->sig = MeasureEvent_LoweringTimeOut;
    }
    if(me->state==Lifting_to_Max2 && (Blocking_Check(pid)))
    {
        e->sig = MeasureEvent_Blocking;
    }
    if(me->state==BlockingatMax2)
    {
        e->sig = MeasureEvent_BlockingTimeOut;
    }
    if(me->state==Lowering2 && me->state_time>Turning_Time)
    {
        e->sig = MeasureEvent_LoweringTimeOut;
    }
    if(me->state==Lifting_to_Max3 && (Blocking_Check(pid)))
    {
        e->sig = MeasureEvent_Blocking;
    }
    if(me->state==BlockingatMax3)
    {
        e->sig = MeasureEvent_BlockingTimeOut;
    }
    if(me->state==Lowering_to_Min1 && (Blocking_Check(pid)))
    {
        e->sig = MeasureEvent_Blocking;
    }
    if(me->state==BlockingatMin1)
    {
        e->sig = MeasureEvent_BlockingTimeOut;
    }
    if (me->state==Lifting1 && me->state_time>Turning_Time)
    {
        e->sig = MeasureEvent_LiftingTimeOut;
    }
    if(me->state==Lowering_to_Min2 && me->state_time > BlockingDetect_MinTime && (Blocking_Check(pid) || me->state_time > Turning_Time))
    {
        // 兜底超时后按堵转流程推进，避免长时间卡在下降状态
        e->sig = MeasureEvent_Blocking;
    }
    if(me->state==BlockingatMin2)
    {
        e->sig = MeasureEvent_BlockingTimeOut;
    }
    if(me->state==Lifting2 && me->state_time>Turning_Time)
    {
        e->sig = MeasureEvent_LiftingTimeOut;
    }
    if(me->state==Lowering_to_Min3 && me->state_time > BlockingDetect_MinTime && (Blocking_Check(pid) || me->state_time > Turning_Time))
    {
        e->sig = MeasureEvent_Blocking;
    }
    if(me->state==BlockingatMin3)
    {
        e->sig = MeasureEvent_BlockingTimeOut;
    }
}

void Lift_Control(FSMstate *me, DM_Motor_1to4_Instance *motor_instance)
{
    switch(me->state)
    {
        case WaitingForStart:
        case BlockingatMax1:
        case BlockingatMax2:
        case BlockingatMax3:
        case BlockingatMin1:
        case BlockingatMin2:
        case BlockingatMin3:
        case done:
            motor_instance->Target_Omega = 0.0f;
        break;
        case Lifting_to_Max1:
        case Lifting_to_Max2:
        case Lifting_to_Max3:
            motor_instance->Target_Omega = Lift_toMaxSpeed;
        break;
        case Lowering1:
        case Lowering2:
            motor_instance->Target_Omega = LoweringSpeed;
        break;
        case Lowering_to_Min1:
        case Lowering_to_Min2:
        case Lowering_to_Min3:
            motor_instance->Target_Omega = Down_toMinSpeed;
        break;
        case Lifting1:
        case Lifting2:
            motor_instance->Target_Omega = LiftingSpeed;
        break;
    }
}

void Lift_Task(void *argument)
{
    MeasureFSM_Init();
    while (1)
    {
      test_remote_ch2 = Basic_Math_Modulus_Return(&remote_channel_ch2, (int32_t)rc_channels.ch[2]);
      Lift_Control(&MeasureMAXMIN_Front_t, &DM_Motor_1to4_Instances[0]);
      Lift_Control(&MeasureMAXMIN_Back_t, &DM_Motor_1to4_Instances[1]);
      osDelay(1); // 每1ms更新一次
    }
}
/* Function prototypes -------------------------------------------------------*/

