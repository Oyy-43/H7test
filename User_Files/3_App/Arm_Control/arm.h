#ifndef __ARM_H
#define __ARM_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crsf.h"
#include "drv_motor_dm.h"
#include "bsp_power.h"
#include "alg_FSM.h"
#include "buzzer_music.h"
#include "tele_task.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 控制电机角度的接口结构体
 * 
 */
typedef struct arm_angle_s
{
    float Motor0_Height;
    float Motor1_X;
    float Motor2_Arm;
    float Motor3_Suctiom;
}arm_angle_s;

extern arm_angle_s Arm_Autocmd;
extern FSMstate GetKFS_State_t;

typedef enum FSM_GetKFS_State
{
    GetKFS_Idle=0,                       //未开始
    GetKFS_Process0,                     //第0步，把除了电机1的别的电机角度转到
    GetKFS_Process1,                     //第一步，缓慢前伸，写死几秒钟
    GetKFS_Process1_2,                   //第一点五步，静止不动确保吸上来了
    GetKFS_Process2,                     //第二步，回伸一点，并且吸盘反转
    GetKFS_Process3,                     //第三步，大臂和X转到指定角度
    GetKFS_Process4,                     //第四步，关闭吸盘,等待超时
    GetKFS_Down_Process0,                //第一步，调整高度,X轴前伸,大臂向前,吸盘向下 
    GetKFS_Down_Process1,                //第二步，降低高度,写死几秒钟
    GetKFS_Down_Process2,                //第二点五步，静止不动确保吸上来了
    GetKFS_Down_Process3,                //第三步，抬高高度,吸盘翻转，转到Process3完成后续动作
    GetKFS_Done,                         //完成，回到NONE状态
}FSM_GetKFS_State;

typedef enum FSM_GetKFS_Event
{
    GetKFS_Event_None=0,                 //不进行KFS的拾取
    GetKFS_Event_Start,                  //接收到开始信号，进入pr0，把除电机1以外的电机的角度转到位
    GetKFS_Event_Ready,                  //角度到位信号，说明除了电机1以外的角度都已经到位，进入pr1，开始往前伸x
    GetKFS_Event_timeout1,               //往前伸时的超时信号，进入pr1_2，静止不动一段时间
    GetKFS_Event_timeout2,               //禁止的超时信号，进入pr2，翻转吸盘的同时回收一点X轴
    GetKFS_Event_X_suction,              //X轴和吸盘角度到位，进入pr3，大臂和X转到指定角度
    GetKFS_Event_X_armready,             //大臂和X都转到指定角度，触发角度到位信号，进入pr4     
    GetKFS_Event_timeout3,               //关闭吸盘，等待超时信号，进入NONE状态
    GetKFS_Event_Start2,                 //接收到开始信号，进入Down_pr0，把所有电机的角度转到位
    GetKFS_Event_Ready2,                 //角度到位信号，说明所有的角度都已经到位，进入Down_pr1，开始降低高度
    GetKFS_Event_timeout4,               //降低高度时的超时信号，
    GetKFS_Event_timeout5,               //静止时的超时信号，进入DPR3
    GetKFS_Event_Turn,                   //抬高高度，吸盘翻转，进入PR3
    GetKFS_Event_Done,                   //超时信号，进入NONE状态                                  
}FSM_GetKFS_Event;                       

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void GetKFS_Event_Generate(FSMstate *me,Event *e);
void GetKFS_Dispatch(FSMstate *me, Event *e);
void GetKFS_FSM_Run();

#endif /* __ARM_H */