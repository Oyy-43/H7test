/**
 * @file move_task.c
 * @author Oyyp
 * @brief 根据上位机发送的目标位置进行移动规划
 * @version 0.1
 * @date 2026-06-29 0.1 init
 *
 * @copyright Copyright
 *
 */

 /* Includes ------------------------------------------------------------------*/
 #include "move_task.h"
 
 
 /* Private macros ------------------------------------------------------------*/
 
 /* Private types -------------------------------------------------------------*/
PID_TypeDef Move_Pid_X,Move_Pid_Y;
 /* Private variables ---------------------------------------------------------*/
float measure_position[2] = {0.0f, 0.0f}; //测量位置
float target_position[2] = {0.0f, 0.0f}; //目标位置
float Move_Pid_Out[2] = {0.0f, 0.0f}; //PID输出
float move_pid_kp[2] = {0.8f,0.8f};
float move_pid_ki[2] = {0.001f,0.001f};
float move_pid_kd[2] = {0.0f,0.0f};
float move_pid_kf[2] = {0.0f,0.0f};
 /* Private function declarations ---------------------------------------------*/
void Move_Routinue_Pid_Init()
{
    PID_Init(&Move_Pid_X,0.5f,0.1f,0.05f,move_pid_kp[0],move_pid_ki[0],move_pid_kd[0],move_pid_kf[0],0,0,0.5,0,0,0,Integral_Limit|ChangingIntegralRate);
    PID_Init(&Move_Pid_Y,0.5f,0.1f,0.05f,move_pid_kp[1],move_pid_ki[1],move_pid_kd[1],move_pid_kf[1],0,0,0.5,0,0,0,Integral_Limit|ChangingIntegralRate);
}

void Move_Routinue_Pid_Cal()
{
    Move_Pid_Out[0] = PID_Calculate(&Move_Pid_X, measure_position[0], target_position[0],0.001f);
    Move_Pid_Out[1] = PID_Calculate(&Move_Pid_Y, measure_position[1], target_position[1],0.001f);
}

void Position_update()
{
    measure_position[0] = PC_frame.Position_MeasureX;
    measure_position[1] = PC_frame.Position_MeasureY;
    target_position[0] = PC_frame.Position_Target_X;
    target_position[1] = PC_frame.Position_Target_Y;
}
 /* Function prototypes -------------------------------------------------------*/
 
void Position_Control(void *argument)
{   
    Move_Routinue_Pid_Init();
    while(1)
    {   
        Position_update();
        Move_Routinue_Pid_Cal();
        osDelay(1);
    }
}
 