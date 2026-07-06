#ifndef __GET_WEAPON_H
#define __GET_WEAPON_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "alg_FSM.h"
#include "withPC.h"
#include "task_config_and_callback.h"
/* Exported macros -----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef enum FSM_GetWeapon_State
{
    GetWeapon_Idle=0,                       //未开始
    GetWeapon_RuntoPosition1,               //跑到指定位置
    GetWeapon_Process0,                     //往前走，直到检测到武器
    GetWeapon_Process0_5,                   //延迟一小会再向左
    GetWeapon_Process1,                     //往左移动，直到检测到端头
    GetWeapon_Process2,                     //舵机动作1：夹住
    GetWeapon_Process2_5,                   //舵机动作1.5：抬高
    GetWeapon_Process3,                     //舵机动作2：抬起
    GetWeapon_Process4,                     //底盘右移
    GetWeapon_Process5,                     //旋转180°
    GetWeapon_Process6,                     //等待接收对接完成信号
    GetWeapon_Process7,                     //松爪子
    GetWeapon_TurnBack,                     //旋转回原位
    GetWeapon_Quit,                         //拾取的过程中被打断，向外侧退出，超时结束，并且回到pr0
    GetWeapon_Done,                         //完成，删除任务
}FSM_GetWeapon_State;   

typedef enum FSM_GetWeapon_Event
{
    GetWeapon_Event_None=0,                 //不进行抓取端头
    GetWeapon_Event_Start,                  //接收到开始信号，进入RuntoPosition1
    GetWeapon_Event_Position1,              //到达指定位置，进入Process0
    GetWeapon_Event_WeaponDetect,           //检测到武器，进入Process0_5
    GetWeapon_Event_WeaponTimeout,          //向前走超时了
    GetWeapon_Event_HeadDetect,             //检测到端头，进入Process1
    GetWeapon_Event_ClampDone,              //夹紧完成，进入Process2
    GetWeapon_Height_upDone,                //抬高完成，进入Process3
    GetWeapon_Event_MoveDone,               //右移完成，进入Process5
    GetWeapon_Event_LiftDone,               //舵机抬起完成，进入Process4
    GetWeapon_Event_RotateDone,             //旋转完成，进入Process6
    GetWeapon_Event_DockingDone,            //接收对接完成信号，进入松爪子
    GetWeapon_Event_ReleaseDone,            //松爪子完成，进入旋转回原位
    GetWeapon_Event_TurnBackDone,           //旋转回原位完成，进入Done
    GetWeapon_Event_Interrupt,              //抓武器过程中被中断，继续往前走，直到检测到武器
    GetWeapon_Event_QuitTimeout,            //退向外侧超时中断
    GetWeapon_Event_Timeout,                //超时，退回IDLE状态
}FSM_GetWeapon_Event;
/* Exported constants --------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/
extern FSMstate GetWeapon_State_t;
/* Exported function declarations --------------------------------------------*/
void GetWeapon_Event_Generate(FSMstate *me, Event *e);
void GetWeapon_Dispatch(FSMstate *me, Event *e);
void GetWeapon_FSM_Run();

#endif /* __GET_WEAPON_H */
