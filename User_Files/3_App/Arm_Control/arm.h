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
#include "withPC.h"

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

    GetKFS_Out_Process0,                 //第一步，高度抬到对准KFS中间前伸X轴,让吸盘水平,缓等待后，让大臂转到对准KFS，角度到位位后进入Out_Process1
    GetKFS_Out_Process1,                 //第二步，X轴回收,等待若干时间，确保吸上来之后，进入下一步
    GetKFS_Out_Process2,                 //第三步, X轴前伸，缓等待之后，翻转大臂角度一
    GetKFS_Out_Process3,                 //第四步, 大臂角度2
    GetKFS_Out_Process4,                 //第五步，抬高高度，然后X轴前升（可能底盘也需要升高）
    GetKFS_Out_Process5,                 //第六步, 底盘移动或者X轴前伸，等待若干时间,确保KFS放下来之后再进入下一步
    GetKFS_Out_Process6,                 //第七步，X轴回收
    GetKFS_Out_Process7,                 //第八步, X轴回收，底盘下降接下来转到完成状态

    GetKFS_High_Out_Process0,            //第一步，抬高高度,前伸X轴
    GetKFS_High_Out_Process1,            //第二步，大臂和吸盘翻转
    GetKFS_High_Out_Process2,            //第三步，x回收，开启吸盘,等待超时事件
    GetKFS_High_Out_Process2_5,          //第三点五步，静止不动确保吸上来了
    GetKFS_High_Out_Process3,            //第四步，前伸X轴,大臂伸直,底盘抬高
    GetKFS_High_Out_Process4,            //第五步，大臂水平，x轴前伸
    GetKFS_High_Out_Process5,            //第六步，关闭吸盘，等待超时中
    GetKFS_High_Out_Process6,            //第七步，x轴回收，底盘下降，进入完成状态


    GetKFS_Save_Process0,                //第一步，第0步，把除了电机1的别的电机角度转到Save_Process1
    GetKFS_Save_Process1,                //第二步，缓慢前伸，写死几秒钟
    GetKFS_Save_Process1_2,              //第二点五步，静止不动确保吸上来了
    GetKFS_Save_Process2,                //第三步，回伸一点
    GetKFS_Save_Process3,                //第四步，大臂和X转到指定角度

    GetKFS_Save_Down_Process0,           //第一步，调整高度,X轴前伸,大臂向前,吸盘向下
    GetKFS_Save_Down_Process1,           //第二步，降低高度,写死几秒钟
    GetKFS_Save_Down_Process2,           //第三步，静止不动确保吸上来了
    GetKFS_Save_Down_Process3,           //第四步，抬高高度,吸盘翻转为水平
    GetKFS_Save_Down_Process4,           //第五步，大臂和X轴转到指定角度

    GetKFS_High_Process0,                //第一步，抬高高度,调整大臂和吸盘的位置
    GetKFS_High_Process1,                //第二步，缓慢前伸，写死几秒钟
    GetKFS_High_Process1_2,              //第二点五步，静止不动确保吸上来了
    GetKFS_High_Process2,                //第三步，吸盘朝上，回伸一点
    GetKFS_High_Process3,                //第四步，大臂和X转到指定角度
    GetKFS_High_Process4,                //第五步，关闭吸盘，等待超时信号

    GetKFS_Done_Save,                    //完成,手上还吸着一个KFS
    GetKFS_Done,                         //完成，回到NONE状态
}FSM_GetKFS_State;

typedef enum FSM_GetKFS_Event
{
    GetKFS_Event_None=0,                 //不进行KFS的拾取
    GetKFS_Event_Start,                  //接收到开始信号，进入pr0，把除电机1以外的电机的角度转到位
    GetKFS_Event_Ready,                  //角度到位信号，说明除了电机1以外的角度都已经到位，进入pr1，开始往前伸x
    GetKFS_Event_timeout1,               //往前伸时的超时信号，进入pr1_2，静止不动一段时间
    GetKFS_Event_timeout1_2,             //
    GetKFS_Event_timeout2,               //静止时的超时信号，进入pr2，翻转吸盘的同时回收一点X轴
    GetKFS_Event_X_suction,              //X轴和吸盘角度到位，进入pr3，大臂和X转到指定角度
    GetKFS_Event_Save_X_suction,         //X轴和吸盘角度到位，进入Save_pr3，大臂和X转到指定角度
    GetKFS_Event_X_armready,             //大臂和X都转到指定角度，触发角度到位信号，进入pr4     
    GetKFS_Event_timeout3,               //关闭吸盘，等待超时信号，进入NONE状态

    GetKFS_Event_Start2,                 //接收到开始信号，进入Down_pr0，把所有电机的角度转到位
    GetKFS_Event_Ready2,                 //角度到位信号，说明所有的角度都已经到位，进入Down_pr1，开始降低高度
    GetKFS_Event_timeout4,               //降低高度时的超时信号，
    GetKFS_Event_timeout5,               //静止时的超时信号，进入DPR3
    GetKFS_Event_Turn,                   //抬高高度，吸盘翻转，进入PR3

    GetKFS_Event_Out_Start,              //接收到开始信号,进入Out_pr0,把所有电机的角度转到位
    GetKFS_Event_Out_Ready,              //角度到位信号，说明所有的角度都已经到位，进入Out_pr1，开始降低高度
    GetKFS_Event_Out_Height,             //高度下降到可以吸KFS的高度，进入Out_pr2，延迟一段时间
    GetKFS_Event_Out_Timeout,            //延迟一段时间后，进入Out_pr3，抬高一点高度，然后X轴前升，大臂翻转
    GetKFS_Event_Out_Ready2,             //角度到位信号，说明所有的角度都已经到位，进入Out_pr4，吸盘翻转，底盘可能需要抬高一点
    GetKFS_Event_Out_Ready3,             //角度到位信号，说明吸盘的角度都已经到位，进入Out_pr5，X轴往前伸,到位后下一步
    GetKFS_Event_Out_XReady,             //X轴到位信号，说明X轴的角度都已经到位，进入Out_pr6，等待放KFS完成
    GetKFS_Event_Out_Timeout2,           //等待放KFS完成的超时信号，进入Out_pr7，X轴回收，底盘下降接下来转到完成状态   
    GetKFS_Event_Out_Done,               //X轴回收完成，底盘可下降到位，到位后完成

    GetKFS_Event_High_Out_Start,         //接收到开始信号,进入High_Out_pr0,把所有电机的角度转到位
    GetKFS_Event_High_Out_Ready,         //角度到位信号，说明所有的角度都已经到位，进入High_Out_pr1，开始翻转大臂和吸盘
    GetKFS_Event_High_Out_Ready2,        //角度到位信号，说明大臂和吸盘的角度都已经到位，进入High_Out_pr2，X回收，开启吸盘,等待超时事件
    GetKFS_Event_High_Out_Ready3,        //角度到位信号，说明X轴和吸盘的角度都已经到位，进入High_Out_pr2_5，等待超时吸取KFS
    GetKFS_Event_High_Out_Timeout,       //超时溢出信号，等待KFS完全吸到，进入High_Out_pr3，前伸X轴,大臂伸直,底盘抬高
    GetKFS_Event_High_Out_Ready4,        //角度到位信号，说明X轴和大臂的角度都已经到位，进入High_Out_pr4，大臂水平，x轴前伸
    GetKFS_Event_High_Out_Ready5,        //角度到位信号，说明大臂和X轴的角度都已经到位，进入High_Out_pr5，关闭吸盘，等待超时中
    GetKFS_Event_High_Out_timeout2,      //超时溢出信号，等待KFS完全放下，进入High_Out_pr6，x轴回收，底盘下降，进入完成状态
    GetKFS_Event_High_Out_Done,          //x轴回收，说明已完成放置，该转到完成状态了


    GetKFS_Event_High_SucXready,         //吸盘和X轴都转到指定角度，触发角度到位信号，进入Save_pr2

    GetKFS_Event_Start3_Savefront,       //接收到前伸信号，进入Save_pr0，把除了电机1的别的电机角度转到Save_pr1
    GetKFS_Event_Start4_SaveDown,        //接收到下降保留信号，进入Save_Down_pr0，把所有电机的角度转到位
    GetKFS_Event_Start5_HighSuck,        //接收到开始向上吸取KFS信号
    GetKFS_Event_Done,                   //超时信号，进入NONE状态                                  
}FSM_GetKFS_Event;                       

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void GetKFS_Event_Generate(FSMstate *me,Event *e);
void GetKFS_Dispatch(FSMstate *me, Event *e);
void GetKFS_FSM_Run();

#endif /* __ARM_H */