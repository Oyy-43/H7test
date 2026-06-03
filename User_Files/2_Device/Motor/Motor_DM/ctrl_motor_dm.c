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
 
 /* Private macros ------------------------------------------------------------*/


 /* Private types -------------------------------------------------------------*/
 PID_TypeDef Motor_DM_1_To_4_PID[DM_Motor_1_To_4_Num];
 PID_TypeDef Motor_DM3519_POS_PID[DM_Motor_1_To_4_Num];
 PID_TypeDef Motor_DM_SPEED_PID[DM_Motor_Normal_Num];
 PID_TypeDef Motor_DM_POSITION_PID[DM_Motor_Normal_Num];
 Struct_Filter_Frequency Motor_3519_Speed_Filter[DM_Motor_1_To_4_Num];
 Struct_Filter_Frequency Motor_4310_Speed_Filter[DM_Motor_Normal_Num];
 /* Private variables ---------------------------------------------------------*/
 float DM_PIDKP[DM_Motor_1_To_4_Num] = {2600.0f, 2600.0f};
 float DM_PIDKI[DM_Motor_1_To_4_Num] = {10.0f, 10.0f};
 float DM_PIDKD[DM_Motor_1_To_4_Num] = {0.0f, 0.0f};
 float DM_PIDKf[DM_Motor_1_To_4_Num] = {100.0f, 100.0f};
 float DM3519_POS_kp[DM_Motor_1_To_4_Num] = {25.0f, 25.0f};
 float DM3519_POS_ki[DM_Motor_1_To_4_Num] = {0.0f, 0.0f};
 float DM3519_POS_kd[DM_Motor_1_To_4_Num] = {0.0f, 0.0f};
 float DM3519_POS_kf[DM_Motor_1_To_4_Num] = {0.0f, 0.0f};
 float DM_SPEEDPIDKP[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_SPEEDPIDKI[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_SPEEDPIDKD[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_SPEEDPIDKf[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_SPEEDPIDKffStaticPos[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_SPEEDPIDKffStaticNeg[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_POSITIONPIDKP[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_POSITIONPIDKI[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_POSITIONPIDKD[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_POSITIONPIDKf[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};

 float test_out=0.0f;
 int32_t test_encoderMax=0,test_encoderMin=0;
 float test_angle = 0.95f;
 /* Private function declarations ---------------------------------------------*/
/** 
 * @brief 初始化PID控制器
 * @param void
 * @return void
 */
void Motor_DM_InitPID()
{
	uint8_t i;

	PID_Init(&Motor_DM_1_To_4_PID[0],16384.0f,2500.0f,0.0f,DM_PIDKP[0],DM_PIDKI[0],DM_PIDKD[0],DM_PIDKf[0],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	PID_Init(&Motor_DM_1_To_4_PID[1],16384.0f,2500.0f,0.0f,DM_PIDKP[1],DM_PIDKI[1],DM_PIDKD[1],DM_PIDKf[1],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	PID_Init(&Motor_DM3519_POS_PID[0],30.0f,0.5f,0.0f,DM3519_POS_kp[0],DM3519_POS_ki[0],DM3519_POS_kd[0],DM3519_POS_kf[0],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	PID_Init(&Motor_DM3519_POS_PID[1],30.0f,0.5f,0.0f,DM3519_POS_kp[1],DM3519_POS_ki[1],DM3519_POS_kd[1],DM3519_POS_kf[1],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	for (i = 0U; i < DM_Motor_Normal_Num; i++)
	{
		PID_Init(&Motor_DM_SPEED_PID[i],10.0f,1.5f,0.0f,DM_SPEEDPIDKP[i],DM_SPEEDPIDKI[i],DM_SPEEDPIDKD[i],DM_SPEEDPIDKf[i],DM_SPEEDPIDKffStaticPos[i],DM_SPEEDPIDKffStaticNeg[i],0.0f,0.0f,0.0f,0.0f,Integral_Limit);
		PID_Init(&Motor_DM_POSITION_PID[i],6.28f,0.0f,0.0f,DM_POSITIONPIDKP[i],DM_POSITIONPIDKI[i],DM_POSITIONPIDKD[i],DM_POSITIONPIDKf[i],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	}


    for (i = 0U; i < DM_Motor_1_To_4_Num; i++)
    {
        Filter_Frequency_Init(&Motor_3519_Speed_Filter[i],
                              0.0f,
                              0.0f,
                              Filter_Frequency_Type_LOWPASS,
                              70.0f,
                              FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f,
                              1000.0f,
                              8U);
    }
	
	for (i = 0U; i < DM_Motor_Normal_Num; i++)
	{
        Filter_Frequency_Init(&Motor_4310_Speed_Filter[i],
                              0.0f,
                              0.0f,
                              Filter_Frequency_Type_LOWPASS,
                              70.0f,
                              FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f,
                              1000.0f,
                              8U);
	}
}
/** 
 * @brief 计算PID控制器
 * @param void
 * @return void
 */
void Motor_DM_CalPID()
{
	if(Front_Calibrated)
	{
		DM_Motor_1to4_Instances[0].Target_Omega = PID_Calculate(&Motor_DM3519_POS_PID[0], DM_Motor_1to4_Instances[0].Encoder_Part, DM_Motor_1to4_Instances[0].Target_Total_Encoder, 0.001f);
	}
	if(Back_Calibrated)
	{
		DM_Motor_1to4_Instances[1].Target_Omega = PID_Calculate(&Motor_DM3519_POS_PID[1], DM_Motor_1to4_Instances[1].Encoder_Part, DM_Motor_1to4_Instances[1].Target_Total_Encoder, 0.001f);
	}

	Filter_Frequency_Set_Now(&Motor_3519_Speed_Filter[0], DM_Motor_1to4_Instances[0].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_3519_Speed_Filter[0]);
    DM_Motor_1to4_Instances[0].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_3519_Speed_Filter[0]);
    DM_Motor_1to4_Instances[0].Out = PID_Calculate(&Motor_DM_1_To_4_PID[0], DM_Motor_1to4_Instances[0].Filtered_Omega, DM_Motor_1to4_Instances[0].Target_Omega, 0.001f);

	Filter_Frequency_Set_Now(&Motor_3519_Speed_Filter[1], DM_Motor_1to4_Instances[1].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_3519_Speed_Filter[1]);
    DM_Motor_1to4_Instances[1].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_3519_Speed_Filter[1]);
    DM_Motor_1to4_Instances[1].Out = PID_Calculate(&Motor_DM_1_To_4_PID[1], DM_Motor_1to4_Instances[1].Filtered_Omega, DM_Motor_1to4_Instances[1].Target_Omega, 0.001f);
	
	Filter_Frequency_Set_Now(&Motor_4310_Speed_Filter[0], DM_Motor_Instances[0].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_4310_Speed_Filter[0]);
    DM_Motor_Instances[0].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_4310_Speed_Filter[0]);
    DM_Motor_Instances[0].Control_Torque = PID_Calculate(&Motor_DM_1_To_4_PID[0], DM_Motor_Instances[0].Filtered_Omega, DM_Motor_Instances[0].Target_Omega, 0.001f);

	Filter_Frequency_Set_Now(&Motor_4310_Speed_Filter[1], DM_Motor_Instances[1].Rx_Data.Now_Omega);
	Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_4310_Speed_Filter[1]);
	DM_Motor_Instances[1].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_4310_Speed_Filter[1]);
	DM_Motor_Instances[1].Control_Torque = PID_Calculate(&Motor_DM_1_To_4_PID[1], DM_Motor_Instances[1].Filtered_Omega, DM_Motor_Instances[1].Target_Omega, 0.001f);

	Filter_Frequency_Set_Now(&Motor_4310_Speed_Filter[2], DM_Motor_Instances[2].Rx_Data.Now_Omega);
	Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_4310_Speed_Filter[2]);
	DM_Motor_Instances[2].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_4310_Speed_Filter[2]);
	DM_Motor_Instances[2].Control_Torque = PID_Calculate(&Motor_DM_1_To_4_PID[2], DM_Motor_Instances[2].Filtered_Omega, DM_Motor_Instances[2].Target_Omega, 0.001f);

	Filter_Frequency_Set_Now(&Motor_4310_Speed_Filter[3], DM_Motor_Instances[3].Rx_Data.Now_Omega);
	Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_4310_Speed_Filter[3]);
	DM_Motor_Instances[3].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_4310_Speed_Filter[3]);
	DM_Motor_Instances[3].Control_Torque = PID_Calculate(&Motor_DM_1_To_4_PID[3], DM_Motor_Instances[3].Filtered_Omega, DM_Motor_Instances[3].Target_Omega, 0.001f);
	// DM_Motor_1to4_Instances[0].Out=PID_Calculate(&Motor_DM_1_To_4_PID[0], DM_Motor_1to4_Instances[0].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[0].Target_Omega,0.001f);
	// DM_Motor_1to4_Instances[1].Out=PID_Calculate(&Motor_DM_1_To_4_PID[1], DM_Motor_1to4_Instances[1].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[1].Target_Omega,0.001f);
	// DM_Motor_Instances[0].Control_Torque=PID_Calculate(&Motor_DM_SPEED_PID[0], DM_Motor_Instances[0].Rx_Data.Now_Omega, DM_Motor_Instances[0].Target_Omega,0.001f);
	// DM_Motor_Instances[0].Target_Omega = PID_Calculate(&Motor_DM_POSITION_PID[0], DM_Motor_Instances[0].Rx_Data.Now_Angle, DM_Motor_Instances[0].Target_Angle,0.001f);
	// DM_Motor_Instances[1].Control_Torque=PID_Calculate(&Motor_DM_SPEED_PID[1], DM_Motor_Instances[1].Rx_Data.Now_Omega, DM_Motor_Instances[1].Target_Omega,0.001f);
}

/**
 * @brief 将输出值填入发送缓存区，并发送
 * 
 */
void Motor_DM_1_To_4_SetOutput()
{
    int16_t out;

    out = (int16_t)(DM_Motor_1to4_Instances[0].Out);
    DM_Motor_1to4_Instances[0].Tx_Data[0] = (uint8_t)(((uint16_t)out) >> 8);    //0~3底盘电机
    DM_Motor_1to4_Instances[0].Tx_Data[1] = (uint8_t)((uint16_t)out);

    out = (int16_t)(DM_Motor_1to4_Instances[1].Out);
    DM_Motor_1to4_Instances[1].Tx_Data[0] = (uint8_t)(((uint16_t)out) >> 8);
    DM_Motor_1to4_Instances[1].Tx_Data[1] = (uint8_t)((uint16_t)out);
}

void DM_Motor_Output()
{
	Motor_DM_CalPID();
	Motor_DM_1_To_4_SetOutput();
}

/**
 * @brief 将输出值填入发送缓存区，并发送
 * 
 */
void Motor_DM_Output_Normal()
{
	DM_Motor_Instances[0].Control_Torque=Motor_DM_SPEED_PID[0].Output;
	// Motor_DM_Normal_Output(&DM_Motor_Instances[0]);
	// Motor_DM_Normal_Output(&DM_Motor_Instances[1]);
}

// void Recording_MaxMin()
// {
// 	DM_Motor_1to4_Instances[1].Target_Omega=-1.0f;
// 	while()
// 	    if (pid->Output < pid->MaxOut * 0.01f)
//         return;

//     if ((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
//     {
//         //电机堵转计数
//         pid->ERRORHandler.ERRORCount++;
//     }
//     else
//     {
//         pid->ERRORHandler.ERRORCount = 0;
//     }

//     if (pid->ERRORHandler.ERRORCount > 1000)
//     {
//         //电机堵转超过1000次
//         pid->ERRORHandler.ERRORType = Motor_Blocked;
//     }
// }

 /* Function prototypes -------------------------------------------------------*/
void DMsetOutput(void *argument)
{
	osDelay(1500);
	// Motor_DM_Normal_Send_Enter(&DM_Motor_Instances[0]);
	for(;;)
	{  
		// DM_Motor_1to4_Instances[1].Out=test_out;
		if(rc_channels.ch[4]<=0)
		{
			DM_Motor_1to4_Instances[0].Target_Omega=0.0f;
			DM_Motor_1to4_Instances[1].Target_Omega=0.0f;
		}
		else
		{
			// test_out = ALG_Sin_Generate(&test_out, 3.0f, 10.0f, 1000.0f);
			// DM_Motor_1to4_Instances[0].Target_Omega=test_out;
			// DM_Motor_1to4_Instances[1].Out = test_out;
			if(rc_channels.ch[6]<=0)
			{
				DM_Motor_1to4_Instances[0].Target_Total_Encoder = test_angle;
				DM_Motor_1to4_Instances[1].Target_Total_Encoder = test_angle;
			}
			else
			{
				DM_Motor_1to4_Instances[0].Target_Total_Encoder =test_remote_ch2;
				DM_Motor_1to4_Instances[1].Target_Total_Encoder =test_remote_ch2;
			}
			
		}
		// Motor_DM_CalPID();
		// Motor_DM_1_To_4_Output();
		// TransData_Send_Two_Float_Frame(&huart1, DM_Motor_1to4_Instances[1].Out, DM_Motor_1to4_Instances	[1].Filtered_Omega, 2);
		osDelay(1);
	}
}
