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
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern bool Front_Calibrated;
extern bool Back_Calibrated;
extern float test_remote_ch2;
/* Exported function declarations --------------------------------------------*/
void MeasureFSM_Dispatch(FSMstate *me, Event *e,PID_TypeDef *pid,DM_Motor_1to4_Instance *motor_instance,bool *calibrated);
void MeasureFSM_Run();
void MeasureEvent_Generate(FSMstate *me, Event *e,PID_TypeDef *pid);

#endif /* __LIFT_C__ */
