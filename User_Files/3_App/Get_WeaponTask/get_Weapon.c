#/**
 * @file get_Weapon.c
 * @author Oyyp
 * @brief 抓端头状态机
 * @version 0.1
 * @date 2026-06-29 0.1 init
 *
 * @copyright Copyright (c) 2026, Oyyp
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "get_Weapon.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
FSMstate GetWeapon_State_t;
Event GetWeapon_Event_t;
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
void GetWeapon_FSM_Init()
{
    FSM_Init(&GetWeapon_State_t);
    GetWeapon_Event_t.sig = GetWeapon_Event_None;
}


void GetWeapon_FSM_Run()
{
    GetWeapon_Dispatch(&GetWeapon_State_t, &GetWeapon_Event_t);
}

void GetWeapon_Dispatch(FSMstate *me, Event *e)
{
    GetWeapon_Event_Generate(me, e);
    switch (me->state)
    {
    case GetWeapon_Idle:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_Start:
            me->state = GetWeapon_RuntoPosition1;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_RuntoPosition1:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_Position1:
            me->state = GetWeapon_Process0;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process0:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_WeaponDetect:
            me->state = GetWeapon_Process0_5;   //
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process0_5:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_WeaponTimeout:
            me->state = GetWeapon_Process1;   //
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process1:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_HeadDetect:
            me->state = GetWeapon_Process2;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process2:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_ClampDone:
            me->state = GetWeapon_Process2_5;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Interrupt:
            me->state = GetWeapon_Quit;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process2_5:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Height_upDone:
            me->state = GetWeapon_Process3;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Interrupt:
            me->state = GetWeapon_Quit;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process3:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_LiftDone:
            me->state = GetWeapon_Process4;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Interrupt:
            me->state = GetWeapon_Quit;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process4:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_MoveDone:
            me->state = GetWeapon_Process5;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process5:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_RotateDone:
            me->state = GetWeapon_Process6;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process6:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_DockingDone:
            me->state = GetWeapon_Process7;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Process7:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_ReleaseDone:
            me->state = GetWeapon_TurnBack;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_TurnBack:
        me->state_time+=1.0f;
        PC_Transmit_Frame.GetWeapon_FinshFlag = 0x01;
        switch(e->sig)
        {
            case GetWeapon_Event_TurnBackDone:
            me->state = GetWeapon_Done;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Quit:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetWeapon_Event_QuitTimeout:
            me->state = GetWeapon_Process0;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetWeapon_Done:
        me->state_time+=1.0f;
        PC_Transmit_Frame.GetWeapon_FinshFlag = 0x01;
        switch(e->sig)
        {
            case GetWeapon_Event_None:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
            case GetWeapon_Event_Timeout:
            me->state = GetWeapon_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
    }
}

void GetWeapon_Event_Generate(FSMstate *me, Event *e)
{
    e->sig = GetWeapon_Event_None;
    if(me->state == GetWeapon_Idle && PC_frame.GetWeapon_StartFlag==1 && Robot_Mode==Robot_Mode_Auto)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_Start;       
    }
    if(me->state == GetWeapon_RuntoPosition1 &&fabs(PC_frame.Position_MeasureY-PC_frame.Position_Target_Y)<0.1f && 
    fabs(PC_frame.Position_MeasureX-PC_frame.Position_Target_X)<0.1f
    && me->state_time > 1500.0f
)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_Position1;       
    }
    if(me->state == GetWeapon_Process0 && IO_Status[0] == false )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_WeaponDetect;       
    }
    if(me->state == GetWeapon_Process0_5 && me->state_time > 125.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_A1_FLAT, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_WeaponTimeout;
    }
    if(me->state == GetWeapon_Process2_5 && (me->state_time > 500.0f || fabs(DM_Motor_Instances[0].Target_Angle +DM_Motor_Instances[0].Rx_Data.Now_Angle) < 0.1f))
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Height_upDone;
    }
    if(me->state == GetWeapon_Process1 && IO_Status[1] == false )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_HeadDetect;       
    }
    if(me->state == GetWeapon_Process2 && me->state_time > 1000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_ClampDone;       
    }
    if(me->state == GetWeapon_Process3 && me->state_time > 1000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_LiftDone;       
    }
    if(me->state == GetWeapon_Process4 && me->state_time >500.0f
        && fabs(PC_frame.Position_MeasureY-PC_frame.Position_Target_Y)<0.1f
    && fabs(PC_frame.Position_MeasureX-PC_frame.Position_Target_X)<0.1f
)    
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_MoveDone;       
    }
    if(me->state == GetWeapon_Process5 && fabs(hipnuc_imu_data.eul[2]-Target_Yaw)<0.1f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_RotateDone;       
    }
    if(me->state == GetWeapon_Process6 && Banding_Flag>1000 )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6,1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_DockingDone;       
    }
    if(me->state == GetWeapon_Process7 && me->state_time > 3000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_ReleaseDone;       
    }
    if(me->state == GetWeapon_TurnBack && fabs(hipnuc_imu_data.eul[2]-Target_Yaw)<0.1f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_TurnBackDone;       
    }
    // 中断检测：仅在抓取过程中（Process2/2_5/3）检测物体丢失
    // 避免到达目标点后（Process4+）上光电持续导通导致状态机卡死
    if(IO_Status[0] == true &&
       (me->state == GetWeapon_Process2 ||
        me->state == GetWeapon_Process2_5 ||
        me->state == GetWeapon_Process3))
    {
        e->sig = GetWeapon_Event_Interrupt;
    }
    if(me->state == GetWeapon_Quit && me->state_time > 940.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetWeapon_Event_QuitTimeout;       
    }
    if(me->state_time > 15000.0f )
    {
        me->state_time = 0;
        e->sig = GetWeapon_Event_Timeout;       
    }
}

/* Function prototypes -------------------------------------------------------*/
