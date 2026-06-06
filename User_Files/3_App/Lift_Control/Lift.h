#ifndef __LIFT_C__
#define __LIFT_C__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "alg_FSM.h"
#include "alg_pid.h"
#include "drv_motor_dm.h"
#include "ctrl_motor_dm.h"
#include "tele_task.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum FSM_MeasureState
{
    WaitingForStart=0,
    Lifting_to_Max1,
    Lifting_to_Max2,
    Lifting_to_Max3,
    BlockingatMax1,
    BlockingatMax2,
    BlockingatMax3,
    Lowering1,
    Lowering2,
    Lowering_to_Min1,
    Lowering_to_Min2,
    Lowering_to_Min3,
    BlockingatMin1,
    BlockingatMin2,
    BlockingatMin3,
    Lifting1,
    Lifting2,
    done,
}FSM_MeasureState;

typedef enum
{
    MeasureEvent_None=0,
    MeasureEvent_Start,
    MeasureEvent_Max1done,
    MeasureEvent_Max2done,
    MeasureEvent_Max3done,
    MeasureEvent_Min1done,
    MeasureEvent_Min2done,
    MeasureEvent_Min3done,
    MeasureEvent_Blocking,
    MeasureEvent_BlockingTimeOut,
    MeasureEvent_LoweringTimeOut,
    MeasureEvent_LiftingTimeOut,
}MeasureEvent;

typedef enum FSM_LiftStatte
{
    No_Lifting=0,
    LiftLevel200,
    LiftLevel400,
    DownLevel200_Step1,
    DownLevel200_Step2,
    DownLevel200_Step3,
    DownLevel200_Step4,
    DownLevel200_Step5,
    DownLevel400_Step1,
    DownLevel400_Step2,
    DownLevel400_Step3,
    DownLevel400_Step4,
    DownLevel400_Step5,
}FSM_LiftStatte;

typedef enum
{
    LiftEvent_None=0,
    LiftEvent_LiftLevel200,
    LiftEvent_LiftLevel400,
    LiftEvent_DownLevel200_Step1,   //车体旋转为后轮朝向
    LiftEvent_DownLevel200_Step2,   //车体向后移动，直到检测到后轮完全离地
    LiftEvent_DownLevel200_Step3,   //车体继续向后移动，直到检测到前轮离地
    LiftEvent_DownLevel200_Step4,   //前轮向下下降
    LiftEvent_DownLevel200_Step5,   //车体后移，并且车身下降
    LiftEvent_DownLevel400_Step1,   //车体旋转为后轮朝向
    LiftEvent_DownLevel400_Step2,   //车体向后移动，直到检测到后轮完全离地
    LiftEvent_DownLevel400_Step3,   //车体继续向后移动，直到检测到前轮离地
    LiftEvent_DownLevel400_Step4,   //前轮向下下降
    LiftEvent_DownLevel400_Step5,   //车体后移，并且车身下降
}LiftEvent;

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern bool Front_Calibrated;
extern bool Back_Calibrated;
extern float test_remote_ch2;
extern float Lift_HightFront, Lift_HightBack;
/* Exported function declarations --------------------------------------------*/
void MeasureFSM_Dispatch(FSMstate *me, Event *e,PID_TypeDef *pid,DM_Motor_1to4_Instance *motor_instance,bool *calibrated);
void MeasureFSM_Run();
void MeasureEvent_Generate(FSMstate *me, Event *e,PID_TypeDef *pid);
void LiftFSM_Dispatch(FSMstate *me,Event *e);
void LiftEvent_Generate(FSMstate *me,Event *e);
void LiftFSM_Run();

#endif /* __LIFT_C__ */
