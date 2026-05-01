/**
 * @file Ctrl_motor_dm.c
 * @author Oyyp
 * @brief 驱动层写太多了会看得很乱，特意分出控制层调用驱动层API进行控制电机
 * @version 0.1
 * @date 2026-03-30 0.1 init
 *
 * @copyright Copyright
 *
 */

 /* Includes ------------------------------------------------------------------*/
 #include "ctrl_motor_dm.h"
 #include "drv_motor_dm.h"
 #include "drv_uart.h"
 #include "alg_pid.h"
 #include "cmsis_os2.h"
 #include <stdio.h>
 #include <string.h>
 
 /* Private macros ------------------------------------------------------------*/
 

 /* Private types -------------------------------------------------------------*/
 PID_TypeDef Motor_DM_1_To_4_PID[DM_Motor_1_To_4_Num];
 PID_TypeDef Motor_DM_SPEED_PID[DM_Motor_Normal_Num];
 PID_TypeDef Motor_DM_POSITION_PID[DM_Motor_Normal_Num];

 /* Private variables ---------------------------------------------------------*/
 float DM_PIDKP[DM_Motor_1_To_4_Num] = {10.68f, 13.35f, 13.35f, 10.68f, 10.68f, 10.68f};  //7.121
 float DM_PIDKI[DM_Motor_1_To_4_Num] = {0.01586f, 0.02000f, 0.02000f, 0.01586f, 0.01586f, 0.01586f}; //0.01186
 float DM_PIDKD[DM_Motor_1_To_4_Num] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  //0.0
 float DM_PIDKf[DM_Motor_1_To_4_Num] = {0.46875f, 0.46875f, 0.46875f, 0.46875f, 0.46875f, 0.46875f}; //0.3125
 float DM_SPEEDPIDKP[DM_Motor_Normal_Num] = {0.6f, 0.0f};
 float DM_SPEEDPIDKI[DM_Motor_Normal_Num] = {0.001f, 0.0f};
 float DM_SPEEDPIDKD[DM_Motor_Normal_Num] = {0.0f, 0.0f};
 float DM_SPEEDPIDKf[DM_Motor_Normal_Num] = {0.001f, 0.0f};
float DM_SPEEDPIDKffStaticPos[DM_Motor_Normal_Num] = {0.50f, 0.0f};
float DM_SPEEDPIDKffStaticNeg[DM_Motor_Normal_Num] = {0.42f, 0.0f};
 float DM_POSITIONPIDKP[DM_Motor_Normal_Num] = {5.625f, 0.0f};
 float DM_POSITIONPIDKI[DM_Motor_Normal_Num] = {0.0f, 0.0f};
 float DM_POSITIONPIDKD[DM_Motor_Normal_Num] = {1.0f, 0.0f};
 float DM_POSITIONPIDKf[DM_Motor_Normal_Num] = {0.0f, 0.0f};

 /* Private function declarations ---------------------------------------------*/
/** 
 * @brief 初始化PID控制器
 * @param void
 * @return void
 */
void Motor_DM_InitPID()
{
	PID_Init(&Motor_DM_1_To_4_PID[0],16384.0f,1000.0f,0.0f,DM_PIDKP[0],DM_PIDKI[0],DM_PIDKD[0],DM_PIDKf[0],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[1],16384.0f,1000.0f,0.0f,DM_PIDKP[1],DM_PIDKI[1],DM_PIDKD[1],DM_PIDKf[1],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[2],16384.0f,1000.0f,0.0f,DM_PIDKP[2],DM_PIDKI[2],DM_PIDKD[2],DM_PIDKf[2],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[3],16384.0f,1000.0f,0.0f,DM_PIDKP[3],DM_PIDKI[3],DM_PIDKD[3],DM_PIDKf[3],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[4],16384.0f,1000.0f,0.0f,DM_PIDKP[4],DM_PIDKI[4],DM_PIDKD[4],DM_PIDKf[4],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[5],16384.0f,1000.0f,0.0f,DM_PIDKP[5],DM_PIDKI[5],DM_PIDKD[5],DM_PIDKf[5],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_SPEED_PID[0],40.0f,2.0f,0.05f,DM_SPEEDPIDKP[0],DM_SPEEDPIDKI[0],DM_SPEEDPIDKD[0],DM_SPEEDPIDKf[0],DM_SPEEDPIDKffStaticPos[0],DM_SPEEDPIDKffStaticNeg[0],0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	// PID_Init(&Motor_DM_SPEED_PID[1],40.0f,1.0f,0.0f,DM_SPEEDPIDKP[1],DM_SPEEDPIDKI[1],DM_SPEEDPIDKD[1],DM_SPEEDPIDKf[1],DM_SPEEDPIDKffStaticPos[1],DM_SPEEDPIDKffStaticNeg[1],0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_POSITION_PID[0],10.0f,1.0f,0.01f,DM_POSITIONPIDKP[0],DM_POSITIONPIDKI[0],DM_POSITIONPIDKD[0],DM_POSITIONPIDKf[0],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
}
/** 
 * @brief 计算PID控制器
 * @param void
 * @return void
 */
void Motor_DM_CalPID()
{
	DM_Motor_1to4_Instances[0].Out=PID_Calculate(&Motor_DM_1_To_4_PID[0], DM_Motor_1to4_Instances[0].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[0].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[1].Out=PID_Calculate(&Motor_DM_1_To_4_PID[1], DM_Motor_1to4_Instances[1].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[1].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[2].Out=PID_Calculate(&Motor_DM_1_To_4_PID[2], DM_Motor_1to4_Instances[2].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[2].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[3].Out=PID_Calculate(&Motor_DM_1_To_4_PID[3], DM_Motor_1to4_Instances[3].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[3].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[4].Out=PID_Calculate(&Motor_DM_1_To_4_PID[4], DM_Motor_1to4_Instances[4].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[4].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[5].Out=PID_Calculate(&Motor_DM_1_To_4_PID[5], DM_Motor_1to4_Instances[5].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[5].Target_Omega,0.001f);
	DM_Motor_Instances[0].Control_Torque=PID_Calculate(&Motor_DM_SPEED_PID[0], DM_Motor_Instances[0].Rx_Data.Now_Omega, DM_Motor_Instances[0].Target_Omega,0.001f);
	DM_Motor_Instances[0].Target_Omega = PID_Calculate(&Motor_DM_POSITION_PID[0], DM_Motor_Instances[0].Rx_Data.Now_Angle, DM_Motor_Instances[0].Target_Angle,0.001f);
	// DM_Motor_Instances[1].Control_Torque=PID_Calculate(&Motor_DM_SPEED_PID[1], DM_Motor_Instances[1].Rx_Data.Now_Omega, DM_Motor_Instances[1].Target_Omega,0.001f);
}

/**
 * @brief 将输出值填入发送缓存区，并发送
 * 
 */
void Motor_DM_1_To_4_Output()
{
	*(int16_t *) DM_Motor_1to4_Instances[0].Tx_Data = (int16_t)DM_Motor_1to4_Instances[0].Out;
	*(int16_t *) DM_Motor_1to4_Instances[1].Tx_Data = (int16_t)DM_Motor_1to4_Instances[1].Out;
	*(int16_t *) DM_Motor_1to4_Instances[2].Tx_Data = (int16_t)DM_Motor_1to4_Instances[2].Out;
	*(int16_t *) DM_Motor_1to4_Instances[3].Tx_Data = (int16_t)DM_Motor_1to4_Instances[3].Out;
	*(int16_t *) DM_Motor_1to4_Instances[4].Tx_Data = (int16_t)DM_Motor_1to4_Instances[4].Out;
	*(int16_t *) DM_Motor_1to4_Instances[5].Tx_Data = (int16_t)DM_Motor_1to4_Instances[5].Out;
	CAN_Transmit_Data(&hfdcan1,0x200,CAN1_0x200_Tx_Data,8);
	CAN_Transmit_Data(&hfdcan2,0x1FF,CAN2_0x1ff_Tx_Data,8);
}

/**
 * @brief 将输出值填入发送缓存区，并发送
 * 
 */
void Motor_DM_Output()
{
	DM_Motor_Instances[0].Control_Torque=Motor_DM_SPEED_PID[0].Output;
	// Motor_DM_Normal_Output(&DM_Motor_Instances[0]);
	// Motor_DM_Normal_Output(&DM_Motor_Instances[1]);
}

 /* Function prototypes -------------------------------------------------------*/
void DMsetOutput(void *argument)
{
	osDelay(1500);
	// Motor_DM_Normal_Send_Enter(&DM_Motor_Instances[0]);
	for(;;)
	{  
		osDelay(1);
	}
}
