#ifndef __LIFT_C__
#define __LIFT_C__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "alg_FSM.h"
#include "alg_pid.h"
#include "drv_motor_dm.h"
#include "ctrl_motor_dm.h"
#include "tele_task.h"
#include "drv_tfmini.h"
#include "withPC.h"
#include "buzzer_music.h"
#include "bsp_buzzer.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum FSM_MeasureState
{
    WaitingForStart=0,
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
    MeasureEvent_Min1done,
    MeasureEvent_Min2done,
    MeasureEvent_Min3done,
    MeasureEvent_Blocking,
    MeasureEvent_BlockingTimeOut,
    MeasureEvent_LiftingTimeOut,
}MeasureEvent;

typedef enum FSM_LiftStatte
{
    No_Lifting=0,
    LiftLevel200_Step1, //向前，直到前轮已靠近着台阶
    LiftLevel200_Step2, //停下，并且前后一同向上抬起
    LiftLevel200_Step3, //向前，直到前轮已经上到台阶上
    LiftLevel200_Step4, //停下，并且把前从动轮抬起
    LiftLevel200_Step5, //向前，直到前小轮到达台阶上
    LiftLevel200_Step6, //停下，把后轮抬起来
    LiftLevel200_Step7, //向前，直到前方距离到达设定值，表示已经完成上台阶
    LiftLevel400_Step1, //zhua，直到前轮已靠近着台阶
    LiftLevel400_Step2, //停下，并且前后一同向上抬起
    LiftLevel400_Step3, //向前，直到后轮已靠近着台阶
    LiftLevel400_Step4, //停下，后轮向上抬起
    LiftLevel400_Step5, //向前，直到前方距离到达设定值
    DownLevel200_Step1, //转弯，车体旋转为后轮朝向
    DownLevel200_Step2, //向后移动，直到检测到后轮完全离地
    DownLevel200_Step3, //停下，后轮向下下降
    DownLevel200_Step4, //继续向后移动，直到检测到前方升降机构离地
    DownLevel200_Step5, //停下，前轮向下下降
    DownLevel200_Step6, //继续向后移动，直到车体完全离开台阶
    DownLevel200_Step7, //停下，前后抬升回0，完成下台阶
    DownLevel400_Step1,
    DownLevel400_Step2,
    DownLevel400_Step3,
    DownLevel400_Step4,
    DownLevel400_Step5,
}FSM_LiftStatte;

typedef enum
{
    LiftEvent_None=0,
    LiftEvent_Lift200_StartEvent,          //按键触发，开始进行上台阶动作
    LiftEvent_Lift200_FrontClose,          //检测到前轮已靠近着台阶
    LiftEvent_Lift200_HightEvent,          //检测到已抬升完毕（检测电机的目标值和当前值是否已经一致）
    LiftEvent_Lift200_DistanceEvent1,      //检测到前方距离到一定值,表示前轮已经上到台阶上
    LiftEvent_Lift200_HightEvent2,         //检测到前轮收回完毕（检测电机的目标值和当前值是否已经一致）  
    LiftEvent_Lift200_DistanceEvent2,      //检测到前方距离到一定值,表示前小轮已经上到台阶上
    LiftEvent_Lift200_HightEvent3,         //检测到后轮收回完毕（检测电机的目标值和当前值是否已经一致）
    LiftEvent_Lift200_FrontClose2,         //检测已完全登上台阶，上台阶完毕
    LiftEvent_Lift400_Step1,
    LiftEvent_Lift400_Step2,
    LiftEvent_Lift400_Step3,
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
extern FSMstate LiftingState_t;
extern float LiftStand_Speedvx, LiftStand_Speedvy, LiftStand_Speedvz,Target_Yaw;
/* Exported function declarations --------------------------------------------*/
void MeasureFSM_Dispatch(FSMstate *me, Event *e,PID_TypeDef *pid,DM_Motor_1to4_Instance *motor_instance,bool *calibrated);
void MeasureFSM_Run();
void MeasureEvent_Generate(FSMstate *me, Event *e,PID_TypeDef *pid);
void LiftFSM_Dispatch(FSMstate *me,Event *e);
void LiftEvent_Generate(FSMstate *me,Event *e);
void LiftFSM_Run();

#endif /* __LIFT_C__ */
