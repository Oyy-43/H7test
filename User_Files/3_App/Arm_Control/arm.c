/**
 * @file arm.c
 * @author Oyyp
 * @brief description
 * @version 0.1
 * @date 2026-04-12 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "arm.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
arm_angle_s Arm_Autocmd;
FSMstate GetKFS_State_t;
Event GetKFS_Event_t;

/* Private variables ---------------------------------------------------------*/
float test_angle1= 0.0f;
float test_angle=0.0f;
/* Private function declarations ---------------------------------------------*/
void GetKFS_FSM_Init()
{
    FSM_Init(&GetKFS_State_t);
    GetKFS_Event_t.sig = GetKFS_Event_None;
}


void GetKFS_FSM_Run()
{
    GetKFS_Dispatch(&GetKFS_State_t, &GetKFS_Event_t);
}


void GetKFS_Dispatch(FSMstate *me, Event *e)
{
    GetKFS_Event_Generate(me, e);
    switch (me->state)
    {
    case GetKFS_Idle:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Start:
            me->state = GetKFS_Process0;
            me->state_time = 0.0f;
            break;
            case GetKFS_Event_Start2:
            me->state = GetKFS_Down_Process0;
            me->state_time = 0.0f;
            break;
            case GetKFS_Event_Out_Start:
            me->state = GetKFS_Out_Process0;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Process0:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Ready:
            me->state = GetKFS_Process1;
            me->state_time = 0.0f;
            Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_G6, 1.0f, 80); // 发出提示音
            break;
        }
    break;
    case GetKFS_Process1:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_timeout1:
            me->state = GetKFS_Process1_2;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Process1_2:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_timeout2:
            me->state = GetKFS_Process3;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Process2:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_X_suction:
            me->state = GetKFS_Process3;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Process3:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_X_armready:
            me->state = GetKFS_Process4;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Process4:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_timeout3:
            me->state = GetKFS_Done;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Done:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Done:
            me->state = GetKFS_Idle;
            me->state_time = 0.0f;
            break;
        }
    break;
//==========================================切入向下吸取的代码==============================================//
    case GetKFS_Down_Process0:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Ready2:
            me->state = GetKFS_Down_Process1;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Down_Process1:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_timeout4:
            me->state = GetKFS_Down_Process2;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Down_Process2:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_timeout5:
            me->state = GetKFS_Down_Process3;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Down_Process3:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Turn:
            me->state = GetKFS_Process3;
            me->state_time = 0.0f;
            break;
        }
    break;
//==============================================以下为拾取并放置的代码=============================================//
    case GetKFS_Out_Process0:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Out_Ready:
            me->state = GetKFS_Out_Process1;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Out_Process1:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Out_Height:
            me->state = GetKFS_Out_Process2;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Out_Process2:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Out_Timeout:
            me->state = GetKFS_Out_Process3;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Out_Process3:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Out_Ready2:
            me->state = GetKFS_Out_Process4;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Out_Process4:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Out_Ready3:
            me->state = GetKFS_Out_Process5;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Out_Process5:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Out_XReady:
            me->state = GetKFS_Out_Process6;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Out_Process6:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Out_Timeout2:
            me->state = GetKFS_Out_Process7;
            me->state_time = 0.0f;
            break;
        }
    break;
    case GetKFS_Out_Process7:
        me->state_time+=1.0f;
        switch(e->sig)
        {
            case GetKFS_Event_Out_Done:
            me->state = GetKFS_Done;
            me->state_time = 0.0f;
            break;
        }
    break;
    default:
    break;
    }
}

void GetKFS_Event_Generate(FSMstate *me,Event *e)
{
    e->sig = GetKFS_Event_None;
    if (Robot_Mode != Robot_Mode_Auto) return;
    //前伸拾取的开始信号
    if(Robot_Mode == Robot_Mode_Auto && PC_frame.GetKFS_CMD==0x01)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Start;        
    }
    //向下拾取的开始信号
    if(Robot_Mode == Robot_Mode_Auto && PC_frame.GetKFS_CMD==0x02)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Start2;
    }
//==================================================以下为前伸拾取的状态机开始代码====================================
    if(me->state == GetKFS_Process0 && fabs(DM_Motor_Instances[0].Target_Angle + DM_Motor_Instances[0].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[2].Target_Angle - DM_Motor_Instances[2].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[3].Control_Angle - DM_Motor_Instances[3].Rx_Data.Now_Angle) < 0.1f
)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Ready;        
    }
    if(me->state == GetKFS_Process1 && me->state_time >1500.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_timeout1;
    }
    if(me->state == GetKFS_Process1_2 && me->state_time >1000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_timeout2;       
    }
    if(me->state == GetKFS_Process2 && fabs(DM_Motor_Instances[1].Target_Angle - DM_Motor_Instances[1].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[3].Control_Angle - DM_Motor_Instances[3].Rx_Data.Now_Angle) < 0.1f
)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_X_suction;
    }
    if(me->state == GetKFS_Process3 && fabs(DM_Motor_Instances[1].Target_Angle - DM_Motor_Instances[1].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[2].Target_Angle - DM_Motor_Instances[2].Rx_Data.Now_Angle) < 0.5f
)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_X_armready;       
    }
    if(me->state == GetKFS_Process4 && me->state_time > 3000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_timeout3;       
    }
    if(me->state == GetKFS_Done && me->state_time > 2000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Done;       
    }
//================================================以下为向下拾取代码的开始=================================================
    if(me->state == GetKFS_Down_Process0 && ((fabs(DM_Motor_Instances[0].Target_Angle - DM_Motor_Instances[0].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[1].Target_Angle - DM_Motor_Instances[1].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[2].Target_Angle - DM_Motor_Instances[2].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[3].Control_Angle - DM_Motor_Instances[3].Rx_Data.Now_Angle) < 0.1f) || me->state_time > 1500.0f)
    )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Ready2;        
    }
    if(me->state == GetKFS_Down_Process1 && me->state_time > 4000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_timeout4;       
    }
    if(me->state == GetKFS_Down_Process2 && me->state_time > 4000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_timeout5;       
    }
    if(me->state == GetKFS_Down_Process3 && fabs(DM_Motor_Instances[0].Target_Angle + DM_Motor_Instances[0].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[3].Control_Angle - DM_Motor_Instances[3].Rx_Data.Now_Angle) < 0.1f
    )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Turn;        
    }
//===============================================以下为拾取存放的KFS并放置代码=============================================//
    if(me->state == GetKFS_Idle && PC_frame.GetKFS_CMD==0x03)
    {
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_Start;
    }
    if(me->state == GetKFS_Out_Process0 && fabs(DM_Motor_Instances[0].Target_Angle + DM_Motor_Instances[0].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[1].Target_Angle - DM_Motor_Instances[1].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[2].Target_Angle - DM_Motor_Instances[2].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[3].Control_Angle - DM_Motor_Instances[3].Rx_Data.Now_Angle) < 0.1f
    )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_Ready;        
    }
    if(me->state == GetKFS_Out_Process1 && fabs(DM_Motor_Instances[0].Target_Angle + DM_Motor_Instances[0].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[1].Target_Angle - DM_Motor_Instances[1].Rx_Data.Now_Angle) < 0.1f
    )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_Height;        
    }
    if(me->state == GetKFS_Out_Process2 && me->state_time > 2000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_Timeout;        
    }
    if(me->state == GetKFS_Out_Process3 && fabs(DM_Motor_Instances[0].Target_Angle + DM_Motor_Instances[0].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[1].Target_Angle - DM_Motor_Instances[1].Rx_Data.Now_Angle) < 0.1f
        &&fabs(DM_Motor_Instances[2].Target_Angle - DM_Motor_Instances[2].Rx_Data.Now_Angle) < 0.1f
    )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_Ready2;        
    }
    if(me->state == GetKFS_Out_Process4 
        &&fabs(DM_Motor_Instances[3].Control_Angle - DM_Motor_Instances[3].Rx_Data.Now_Angle) < 0.1f
    )
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_Ready3;        
    }
    if(me->state == GetKFS_Out_Process5 && fabs(DM_Motor_Instances[1].Target_Angle - DM_Motor_Instances[1].Rx_Data.Now_Angle) < 0.1f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_XReady;        
    }
    if(me->state == GetKFS_Out_Process6 && me->state_time > 2000.0f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_Timeout2;        
    }
    if(me->state == GetKFS_Out_Process7 && fabs(DM_Motor_Instances[1].Target_Angle - DM_Motor_Instances[1].Rx_Data.Now_Angle) < 0.1f)
    {
        me->state_time = 0;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        e->sig = GetKFS_Event_Out_Done;        
    }
}

void arm_control(FSMstate *me)
{
    switch(me->state)
    {
        case GetKFS_Idle:
        PC_Transmit_Frame.GetKFS_Flag = 0x00;
        case GetKFS_Process0:
        PC_Transmit_Frame.GetKFS_Flag = 0x01;
        BSP_Power_Set_DC24_0(false);
        Arm_Autocmd.Motor0_Height = 2.36f;
        Arm_Autocmd.Motor1_X = 0.0f;
        Arm_Autocmd.Motor2_Arm = 3.40f;
        Arm_Autocmd.Motor3_Suctiom = -0.35f;
        break;
        case GetKFS_Process1:
        PC_Transmit_Frame.GetKFS_Flag = 0x01;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 2.36f;
        Arm_Autocmd.Motor1_X += 0.035f;
        Arm_Autocmd.Motor2_Arm = 3.40f;
        Arm_Autocmd.Motor3_Suctiom = -0.35f;
        break;
        case GetKFS_Process1_2:
        PC_Transmit_Frame.GetKFS_Flag = 0x01;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 2.36f;
        Arm_Autocmd.Motor2_Arm = 3.40f;
        Arm_Autocmd.Motor3_Suctiom = -0.35f;
        break;
        case GetKFS_Process2:
        PC_Transmit_Frame.GetKFS_Flag = 0x01;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 8.36f;
        Arm_Autocmd.Motor1_X = 10.0f;
        Arm_Autocmd.Motor2_Arm = 3.40f;
        Arm_Autocmd.Motor3_Suctiom = -1.7f;
        break;
        case GetKFS_Process3:
        PC_Transmit_Frame.GetKFS_Flag = 0x01;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 8.36f;
        Arm_Autocmd.Motor1_X = 5.0f;
        Arm_Autocmd.Motor2_Arm = 1.9f;      //0.8
        Arm_Autocmd.Motor3_Suctiom = -1.7f;
        break;
        case GetKFS_Process4:
        PC_Transmit_Frame.GetKFS_Flag = 0x01;
        BSP_Power_Set_DC24_0(false);
        Arm_Autocmd.Motor0_Height =8.36f;
        Arm_Autocmd.Motor1_X = 0.0f;
        Arm_Autocmd.Motor2_Arm = 1.9f;       //1.9f
        Arm_Autocmd.Motor3_Suctiom = -1.7f;    //-5.5   -0.35
        break;
        case GetKFS_Down_Process0:
        PC_Transmit_Frame.GetKFS_Flag = 0x02;
        BSP_Power_Set_DC24_0(false);
        Arm_Autocmd.Motor0_Height = 15.0f;
        Arm_Autocmd.Motor1_X = 10.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = 1.25f;  //待修改
        break;
        case GetKFS_Down_Process1:
        PC_Transmit_Frame.GetKFS_Flag = 0x02;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height -= 0.015f;
        Arm_Autocmd.Motor1_X = 10.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = 1.25f;  //待修改
        break;
        case GetKFS_Down_Process2:
        PC_Transmit_Frame.GetKFS_Flag = 0x02;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor1_X = 10.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = 1.25f;  //待修改
        break;
        case GetKFS_Down_Process3:
        PC_Transmit_Frame.GetKFS_Flag = 0x02;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 8.36f;
        Arm_Autocmd.Motor1_X = 10.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = -1.7f;
        break;
        case GetKFS_Done:
        PC_Transmit_Frame.GetKFS_Flag = 0x01;
        BSP_Power_Set_DC24_0(false);
        Arm_Autocmd.Motor0_Height = 2.36f;
        Arm_Autocmd.Motor1_X = 0.0f;
        Arm_Autocmd.Motor2_Arm = 1.9f;
        Arm_Autocmd.Motor3_Suctiom = -0.35f;
        break;
        case GetKFS_Out_Process0:
        PC_Transmit_Frame.GetKFS_Flag = 0x03;
        BSP_Power_Set_DC24_0(false);
        Arm_Autocmd.Motor0_Height = 15.0f;
        Arm_Autocmd.Motor1_X = 5.0f;
        Arm_Autocmd.Motor2_Arm = 1.9f;
        Arm_Autocmd.Motor3_Suctiom = -1.0f;
        break;
        case GetKFS_Out_Process1:
        PC_Transmit_Frame.GetKFS_Flag = 0x03;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 5.0f;
        Arm_Autocmd.Motor1_X = 0.0f;
        Arm_Autocmd.Motor2_Arm = 0.5f;
        Arm_Autocmd.Motor3_Suctiom = -1.0f;
        break;
        case GetKFS_Out_Process2:
        PC_Transmit_Frame.GetKFS_Flag = 0x03;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 5.0f;
        Arm_Autocmd.Motor1_X = 0.0f;
        Arm_Autocmd.Motor2_Arm = 0.5f;
        Arm_Autocmd.Motor3_Suctiom = -1.0f;
        break;
        case GetKFS_Out_Process3:
        PC_Transmit_Frame.GetKFS_Flag = 0x03;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 10.0f;
        Arm_Autocmd.Motor1_X = 5.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = -1.0f;
        break;
        case GetKFS_Out_Process4:
        PC_Transmit_Frame.GetKFS_Flag = 0x03;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 15.0f;
        Arm_Autocmd.Motor1_X = 5.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = -0.35f;
        break;
        case GetKFS_Out_Process5:
        PC_Transmit_Frame.GetKFS_Flag = 0x03;
        BSP_Power_Set_DC24_0(true);
        Arm_Autocmd.Motor0_Height = 15.0f;
        Arm_Autocmd.Motor1_X = 8.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = -0.35f;
        break;
        case GetKFS_Out_Process6:
        PC_Transmit_Frame.GetKFS_Flag = 0x03;
        BSP_Power_Set_DC24_0(false);
        Arm_Autocmd.Motor0_Height = 15.0f;
        Arm_Autocmd.Motor1_X = 8.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = -0.35f;
        break;
        case GetKFS_Out_Process7:
        PC_Transmit_Frame.GetKFS_Flag = 0x03;
        BSP_Power_Set_DC24_0(false);
        Arm_Autocmd.Motor0_Height = 15.0f;
        Arm_Autocmd.Motor1_X = 0.0f;
        Arm_Autocmd.Motor2_Arm = 3.4f;
        Arm_Autocmd.Motor3_Suctiom = -0.35f;
        break;
    }
}
/* Function prototypes -------------------------------------------------------*/
void Arm_Task(void *argument)
{    
    GetKFS_FSM_Init();
    while (1)
    {
        arm_control(&GetKFS_State_t);
        osDelay(1);                  
    }
    
}
