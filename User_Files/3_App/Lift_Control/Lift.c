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
#define Down_toMinSpeed -1.5f
#define LiftingSpeed 0.5f
#define BlockingDetect_MinTime 50.0f

#define front_speed 0.5f
#define Down200_Front 200.0f
#define Down200_Back 191.0f
#define Down200_Front_up -50.0f

/* Private types -------------------------------------------------------------*/
FSMstate MeasureMAXMIN_Front_t;
FSMstate MeasureMAXMIN_Back_t;
Event MeasureEvent_Front_t;
Event MeasureEvent_Back_t;  
FSMstate LiftingState_t;
Event LiftingEvent_t;
/* Private variables ---------------------------------------------------------*/
int16_t Last_CH12,Last_CH13;
float count;
volatile float Lowering2time = 0.0f;
volatile float Lowering3time = 0.0f;
volatile float Debug_PID_Output = 0.0f;
bool Front_Calibrated = false;
bool Back_Calibrated = false;
float Lift_HightFront = 0.0f;
float Lift_HightBack = 0.0f;
Normali_S remote_channel_ch2;
float test_remote_ch2;
float LiftStand_Speedvx, LiftStand_Speedvy, LiftStand_Speedvz,Target_Yaw;

/* Private function declarations ---------------------------------------------*/
void MeasureFSM_Init()
{
    FSM_Init(&MeasureMAXMIN_Front_t);
    FSM_Init(&MeasureMAXMIN_Back_t);
    FSM_Init(&LiftingState_t);
    MeasureEvent_Front_t.sig = MeasureEvent_None;
    MeasureEvent_Back_t.sig = MeasureEvent_None;
    LiftingEvent_t.sig = LiftEvent_None;
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

void LiftFSM_Run()
{
    LiftFSM_Dispatch(&LiftingState_t, &LiftingEvent_t);
}

/**
 * @brief 上升机构校准有限状态机状态切换检测函数
 * 
 */
void MeasureFSM_Dispatch(FSMstate *me, Event *e,PID_TypeDef *pid,DM_Motor_1to4_Instance *motor_instance,bool *calibrated)
{
    static float Min1, Min2, Min3;
    MeasureEvent_Generate(me, e, pid);
    switch(me->state)
    {
        case WaitingForStart:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Start:
                // me->state = Lowering_to_Min1;
                // me->state_time = 0.0f;
                motor_instance->Encoder_Limit.Min=motor_instance->Rx_Data.Total_Encoder;
                me->state = done;
                pid->ERRORHandler.ERRORCount = 0;
                break;
            }
        break;
        case Lowering_to_Min1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                Min1 = motor_instance->Rx_Data.Total_Encoder;
                me->state = BlockingatMin1;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMin1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
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
                me->state =Lowering_to_Min2;
                me->state_time = 0.0f;
                pid->ERRORHandler.ERRORCount = 0;
                break;
            }
        break;
        case Lowering_to_Min2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                Min2 = motor_instance->Rx_Data.Total_Encoder;
                me->state = BlockingatMin2;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMin2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
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
                me->state =Lowering_to_Min3;
                me->state_time = 0.0f;
                pid->ERRORHandler.ERRORCount = 0;
                break;
            }
        break;
        case Lowering_to_Min3:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_Blocking:
                Min3 = motor_instance->Rx_Data.Total_Encoder;
                motor_instance->Encoder_Limit.Min = (Min1+Min2+Min3)/3.0f;
                me->state = BlockingatMin3;
                me->state_time = 0.0f;
                break;
            }
        break;
        case BlockingatMin3:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case MeasureEvent_BlockingTimeOut:
                me->state = done;
                me->state_time = 0.0f;
                pid->ERRORHandler.ERRORCount = 0;
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

/**
 * @brief 堵转检测函数，返回true表示堵转
 * 
 */
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
        //电机堵转超过250次
        return true;
    }
    return false;
}
/**
 * @brief 上升机构校准有限状态机事件生成检测函数
 * 
 */
void MeasureEvent_Generate(FSMstate *me, Event *e,PID_TypeDef *pid)
{
    e->sig = MeasureEvent_None;
    if(me->state==WaitingForStart && Calibration_finished)
    {
        e->sig = MeasureEvent_Start;
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

/**
 * @brief 上升机构校准有限状态机运行动作函数
 * 
 */
void Lift_Calibrate(FSMstate *me, DM_Motor_1to4_Instance *motor_instance)
{
    switch(me->state)
    {
        case WaitingForStart:
        case BlockingatMin1:
        case BlockingatMin2:
        case BlockingatMin3:
        case done:
            motor_instance->Target_Omega = 0.0f;
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

/**
 * @brief 上升机构运动有限状态机状态切换函数
 * 
 */
void LiftFSM_Dispatch(FSMstate *me,Event *e)
{
    LiftEvent_Generate(me,e);
    switch(me->state)
    {
        case No_Lifting:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_Lift200_StartEvent:
                me->state = LiftLevel200_Step1;
                break;
                case LiftEvent_Lift400_Step1:
                me->state = LiftLevel400_Step1;
                break;
                case LiftEvent_DownLevel200_StartEvent:
                me->state = DownLevel200_Step1;
                break;
            }
        break;
        case LiftLevel200_Step1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_Lift200_FrontClose:
                me->state = LiftLevel200_Step2;
                break;
            }
        break;
        case LiftLevel200_Step2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_Lift200_HightEvent:
                me->state = LiftLevel200_Step3;
                break;
            }
        break;
        case LiftLevel200_Step3:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_Lift200_DistanceEvent1:
                me->state = LiftLevel200_Step4;
                break;
            }
        break;
        case LiftLevel200_Step4:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_Lift200_HightEvent2:
                me->state = LiftLevel200_Step5;
                break;
            }
        break;
        case LiftLevel200_Step5:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_Lift200_DistanceEvent2:
                me->state = LiftLevel200_Step6;
                break;
            }
        break;
        case LiftLevel200_Step6:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_Lift200_HightEvent3:
                me->state = LiftLevel200_Step7;
                break;
            }
        break;
        case LiftLevel200_Step7:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_Lift200_FrontClose2:
                me->state = No_Lifting;
                break;
            }
        break;
        //以上为上200台阶状态机
//=======================================================================================================//
        case DownLevel200_Step1:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_DownLevel200_Step2Event:
                me->state = DownLevel200_Step2;
                break;
            }
        break;
        case DownLevel200_Step2:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_DownLevel200_Step3Event:
                me->state = DownLevel200_Step3;
                break;
            }
        break;
        case DownLevel200_Step3:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_DownLevel200_Step4Event:
                me->state = DownLevel200_Step4;
                break;
            }
        break;
        case DownLevel200_Step4:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_DownLevel200_Step5Event:
                me->state = DownLevel200_Step5;
                break;
            }
        break;
        case DownLevel200_Step5:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_DownLevel200_Step6Event:
                me->state = DownLevel200_Step6;
                break;
            }
        break;
        case DownLevel200_Step6:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_DownLevel200_Step7Event:
                me->state = DownLevel200_Step7;
                break;
            }
        break;
        case DownLevel200_Step7:
            me->state_time+=1.0f;
            switch(e->sig)
            {
                case LiftEvent_DownLevel200_Step8Event:
                me->state = No_Lifting;
                break;
            }
        break;
}
}

/**
 * @brief 上升机构运动有限状态机事件生成函数
 * 
 */
void LiftEvent_Generate(FSMstate *me,Event *e)
{
    e->sig = LiftEvent_None;
    if(me->state==No_Lifting &&(Front_Calibrated && Back_Calibrated) && (Robot_Mode == Robot_Mode_Auto) && (Last_CH12<0 && rc_channels.ch[12]>0))
    // if(me->state==No_Lifting &&(Front_Calibrated && Back_Calibrated) && (Robot_Mode == Robot_Mode_Auto) && (PC_frame.cmd_lift==1))  //自动后启用
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_Lift200_StartEvent;
    }
    if(me->state==LiftLevel200_Step1 && TFmini_RxData[0].Distance <= 3)
    { 
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_Lift200_FrontClose;
    }
    // if(me->state==LiftLevel200_Step2 && (fabs(DM_Motor_1to4_Instances[0].Target_Length-DM_Motor_1to4_Instances[0].Outch_Length)<0.5)&&(fabs(DM_Motor_1to4_Instances[1].Target_Length-DM_Motor_1to4_Instances[1].Outch_Length)<0.5))
    if(me->state==LiftLevel200_Step2 && LIFT_Height_CHECK(0, 200.0f, 0.3f) && LIFT_Height_CHECK(1, 200.0f, 0.3f))
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig =LiftEvent_Lift200_HightEvent;
    }
    if(me->state==LiftLevel200_Step3 && TFmini_RxData[0].Distance < 100) //感觉改成用光电好一点
    // if(me->state==LiftLevel200_Step3 && (me->state_time >5000))
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_Lift200_DistanceEvent1;
    }
    if(me->state==LiftLevel200_Step4 && LIFT_Height_CHECK(0, Down200_Front_up, 0.3f))
    // if(me->state==LiftLevel200_Step4 && (me->state_time >5000))
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_Lift200_HightEvent2;
    }
    if(me->state==LiftLevel200_Step5 && TFmini_RxData[0].Distance < 70) //感觉改成用光电好一点
    // if(me->state==LiftLevel200_Step5 && (me->state_time >5000))
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_Lift200_DistanceEvent2;
    }
    if(me->state==LiftLevel200_Step6 && LIFT_Height_CHECK(1, 0.0f, 0.3f)) //感觉改成用光电好一点
    // if(me->state==LiftLevel200_Step6 && (me->state_time >5000)) //感觉改成用光电好一点
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_Lift200_HightEvent3;         //检测到后轮收回完毕（检测电机的目标值和当前值是否已经一致）
    }
    if(me->state==LiftLevel200_Step7 && TFmini_RxData[0].Distance < 40) //感觉改成用光电好一点
    // if(me->state==LiftLevel200_Step7 && (me->state_time >5000))
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_Lift200_FrontClose2;         //检测已完全登上台阶，上台阶完毕
    }
    Last_CH12 = rc_channels.ch[12];
    //以上为上200台阶相关事件生成
//**====================================================================================================== */
    if(me->state==No_Lifting &&(Front_Calibrated && Back_Calibrated) && (Robot_Mode == Robot_Mode_Auto) && (Last_CH13<0 && rc_channels.ch[13]>0))
    // if(me->state==No_Lifting &&(Front_Calibrated && Back_Calibrated) && (Robot_Mode == Robot_Mode_Auto) && (PC_frame.cmd_lift==2))  //自动后启用
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        Target_Yaw += 180.0f;
        e->sig = LiftEvent_DownLevel200_StartEvent;
    }
    if(me->state==DownLevel200_Step1 && fabs(Target_Yaw-hipnuc_imu_data.eul[2])<0.1f)
    // if(me->state==DownLevel200_Step1 && me->state_time >= 5000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        // e->sig = LiftEvent_DownLevel200_Step2Event;
    }
    if(me->state==DownLevel200_Step2 && TFmini_RxData[0].Distance > 60)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_DownLevel200_Step3Event;
    }
    if(me->state==DownLevel200_Step3 && LIFT_Height_CHECK(1, Down200_Back, 0.3f)) //感觉改成用光电好一点
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_DownLevel200_Step4Event;
    }
    if(me->state==DownLevel200_Step4 && TFmini_RxData[0].Distance > 100)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_DownLevel200_Step5Event;
    }
    if(me->state==DownLevel200_Step5 && LIFT_Height_CHECK(0, 0.0f, 0.3f)) //感觉改成用光电好一点
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_DownLevel200_Step6Event;
    }
    if(me->state==DownLevel200_Step6 && TFmini_RxData[0].Distance > 150)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_DownLevel200_Step7Event;
    }
    if(me->state==DownLevel200_Step7 && LIFT_Height_CHECK(0, 0.0f, 0.3f) && LIFT_Height_CHECK(1, 0.0f, 0.3f)) //感觉改成用光电好一点
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = LiftEvent_DownLevel200_Step8Event;         //检测已完全下台阶，下台阶完毕
    }
    Last_CH13 = rc_channels.ch[13];
}

/**
 * @brief 上升机构校准有限状态机运行高度设定函数
 * 
 */
void Lift_Set_Target(FSMstate *me)
{
    switch(me->state)
    {
        case No_Lifting:
            LiftStand_Speedvx = 0.0f; // 上台阶过程中禁止底盘移动
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = 0.0f;
            Lift_HightBack = 0.0f;
        break;
        case LiftLevel200_Step1:
            LiftStand_Speedvx = 0.3f;
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            // Lift_HightFront = 37.0f;
            Lift_HightFront = 0.0f;
            // Lift_HightBack = 36.0f;
            Lift_HightBack = 0.0f;
        break;
        case LiftLevel200_Step2:
            LiftStand_Speedvx = 0.0f;
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = Down200_Front; //200
            Lift_HightBack = Down200_Back;  //200
        break;
        case LiftLevel200_Step3:
            LiftStand_Speedvx = 0.3f; //0.0f
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = Down200_Front; //200
            Lift_HightBack = Down200_Back;  //200
        break;
        case LiftLevel200_Step4:
            LiftStand_Speedvx = 0.0f;
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = Down200_Front_up; //-80
            Lift_HightBack = Down200_Back;  //200
        break;
        case LiftLevel200_Step5:
            LiftStand_Speedvx = 0.3f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = Down200_Front_up; //-80
            Lift_HightBack = Down200_Back;  //200
        break;
        case LiftLevel200_Step6:
            LiftStand_Speedvx = 0.0f;
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = Down200_Front_up;  //-80
            Lift_HightBack = 0.0f;     //0.0
        break;
        case LiftLevel200_Step7:
            LiftStand_Speedvx = 0.3f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = Down200_Front_up;
            Lift_HightBack = 0.0f;
        break;
        case DownLevel200_Step1:
            LiftStand_Speedvx = 0.0f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = 0.0f;
            Lift_HightBack = 0.0f;
        break;
        case DownLevel200_Step2:
            LiftStand_Speedvx = -0.3f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = 0.0f;
            Lift_HightBack = 0.0f;
        break;
        case DownLevel200_Step3:
            LiftStand_Speedvx = 0.0f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = 0.0f;
            Lift_HightBack = Down200_Back;
        break;
        case DownLevel200_Step4:
            LiftStand_Speedvx = -0.3f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = 0.0f;
            Lift_HightBack = Down200_Back;
        break; 
        case DownLevel200_Step5:
            LiftStand_Speedvx = 0.0f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = Down200_Front;
            Lift_HightBack = Down200_Back;
        break;
        case DownLevel200_Step6:
            LiftStand_Speedvx = -0.3f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = Down200_Front;
            Lift_HightBack = Down200_Back;
        break;
        case DownLevel200_Step7:
            LiftStand_Speedvx = 0.0f;  //0.6
            LiftStand_Speedvy = 0.0f;
            LiftStand_Speedvz = 0.0f;
            Lift_HightFront = 0.0f;
            Lift_HightBack = 0.0f;
        break;
    }
}

void Lift_Task(void *argument)
{
    MeasureFSM_Init();
    while (1)
    {
      test_remote_ch2 = Basic_Math_Modulus_Return(&remote_channel_ch2, (int32_t)rc_channels.ch[2]);
    //   Lift_Calibrate(&MeasureMAXMIN_Front_t, &DM_Motor_1to4_Instances[0]);
    //   Lift_Calibrate(&MeasureMAXMIN_Back_t, &DM_Motor_1to4_Instances[1]);
      Lift_Set_Target(&LiftingState_t);
      osDelay(1); // 每1ms更新一次
    }
}
/* Function prototypes -------------------------------------------------------*/

