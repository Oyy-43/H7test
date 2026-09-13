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
#define LiftDownCheckFront_Torque -900.0f
#define LiftDownCheckBack_Torque  300.0f
 /* Private types -------------------------------------------------------------*/
 GravityFeedforward_Config gravity_feedforward_config = {1.4845f,0.0309f,0.0974f}; //重力前馈拟合曲线参数
 PID_TypeDef Motor_DM_1_To_4_PID[DM_Motor_1_To_4_Num];
 PID_TypeDef Motor_DM3519_POS_PID[DM_Motor_1_To_4_Num];
 PID_TypeDef Motor_DM_SPEED_PID[DM_Motor_Normal_Num];
 PID_TypeDef Motor_DM_POSITION_PID[DM_Motor_Normal_Num];
 PID_TypeDef Motor_LiftHeight_PID;
 Struct_Filter_Frequency Motor_3519_Speed_Filter[DM_Motor_1_To_4_Num];
 Struct_Filter_Frequency Motor_4310_Speed_Filter[DM_Motor_Normal_Num];
 /* Private variables ---------------------------------------------------------*/
 float DM_PIDKP[DM_Motor_1_To_4_Num] = {1560.0f, 1560.0f};  //2600.0f
 float DM_PIDKI[DM_Motor_1_To_4_Num] = {5.0f, 5.0f};		//10.0f
 float DM_PIDKD[DM_Motor_1_To_4_Num] = {0.0f, 0.0f};
 float DM_PIDKf[DM_Motor_1_To_4_Num] = {100.0f, 100.0f};	//100.0f

 float DM3519_POS_kp[DM_Motor_1_To_4_Num] = {2.0f, 2.0f};  	//4.0f (加大P，减小静差)2.0   原1.85
 float DM3519_POS_ki[DM_Motor_1_To_4_Num] = {0.0065f, 0.0075f}; //0.002f (大幅减小I，防积分过冲)0.01 0.0075
 float DM3519_POS_kd[DM_Motor_1_To_4_Num] = {0.0f, 0.0f};   //0.05f (加大D，增加阻尼)
 float DM3519_POS_kf[DM_Motor_1_To_4_Num] = {0.02f, 0.02f};

 float DM_SPEEDPIDKP[DM_Motor_Normal_Num] = {0.200f, 0.3f,10.25f,0.225f};
 float DM_SPEEDPIDKI[DM_Motor_Normal_Num] = {0.001f, 0.0005f,0.5f,0.001f};
 float DM_SPEEDPIDKD[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.04f,0.0f};
 float DM_SPEEDPIDKf[DM_Motor_Normal_Num] = {0.005065f, 0.005065f,0.00f,0.0f};

 float DM_SPEEDPIDKffStaticPos[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_SPEEDPIDKffStaticNeg[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 
 float DM_POSITIONPIDKP[DM_Motor_Normal_Num] = {7.5f,2.5f,4.5f,1.25f};
 float DM_POSITIONPIDKI[DM_Motor_Normal_Num] = {0.0f, 0.01f,0.5f,0.0015f};
 float DM_POSITIONPIDKD[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};
 float DM_POSITIONPIDKf[DM_Motor_Normal_Num] = {0.0f, 0.0f,0.0f,0.0f};

 float Motor_LiftHeight_kp = 2.5f;
 float Motor_LiftHeight_ki = 0.0f;
 float Motor_LiftHeight_kd = 0.0f;
 float Motor_LiftHeight_kf = 10.0f;
int16_t last_ch2;
 float test_out=0.0f;
 int32_t test_encoderMax=0,test_encoderMin=0;
 float Default_anglef = 0.0f;
 float Default_angleb = 0.0f;
 float Default_angleh = 0.0f;
 float g_forwardsee = 0.0f;
 
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
	PID_Init(&Motor_DM3519_POS_PID[0],15.0f,4.5f,0.0f,DM3519_POS_kp[0],DM3519_POS_ki[0],DM3519_POS_kd[0],DM3519_POS_kf[0],0.0f,0.0f,180.0f,10.0f,0.0f,0.0f,Integral_Limit|ChangingIntegralRate);
	PID_Init(&Motor_DM3519_POS_PID[1],15.0f,4.5f,0.0f,DM3519_POS_kp[1],DM3519_POS_ki[1],DM3519_POS_kd[1],DM3519_POS_kf[1],0.0f,0.0f,180.0f,10.0f,0.0f,0.0f,Integral_Limit|ChangingIntegralRate);
	/* 电机[0] */
	PID_Init(&Motor_DM_SPEED_PID[0],15.0f,1.5f,0.0f,DM_SPEEDPIDKP[0],DM_SPEEDPIDKI[0],DM_SPEEDPIDKD[0],DM_SPEEDPIDKf[0],DM_SPEEDPIDKffStaticPos[0],DM_SPEEDPIDKffStaticNeg[0],5.0f,1.0f,0.0f,0.0f,Integral_Limit|ChangingIntegralRate);
	PID_Init(&Motor_DM_POSITION_PID[0],30.0f,0.0f,0.0f,DM_POSITIONPIDKP[0],DM_POSITIONPIDKI[0],DM_POSITIONPIDKD[0],DM_POSITIONPIDKf[0],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	/* 电机[1] */
	PID_Init(&Motor_DM_SPEED_PID[1],10.0f,1.5f,0.0f,DM_SPEEDPIDKP[1],DM_SPEEDPIDKI[1],DM_SPEEDPIDKD[1],DM_SPEEDPIDKf[1],DM_SPEEDPIDKffStaticPos[1],DM_SPEEDPIDKffStaticNeg[1],3.0f,1.0f,0.0f,0.0f,Integral_Limit|ChangingIntegralRate);
	PID_Init(&Motor_DM_POSITION_PID[1],10.0f,0.0f,0.0f,DM_POSITIONPIDKP[1],DM_POSITIONPIDKI[1],DM_POSITIONPIDKD[1],DM_POSITIONPIDKf[1],0.0f,0.0f,3.0f,1.0f,0.0f,0.0f,Integral_Limit);
	/* 电机[2] */
	PID_Init(&Motor_DM_SPEED_PID[2],20.0f,3.0f,0.0f,DM_SPEEDPIDKP[2],DM_SPEEDPIDKI[2],DM_SPEEDPIDKD[2],DM_SPEEDPIDKf[2],DM_SPEEDPIDKffStaticPos[2],DM_SPEEDPIDKffStaticNeg[2],0.5f,0.0f,0.0f,0.0f,Integral_Limit|ChangingIntegralRate);
	PID_Init(&Motor_DM_POSITION_PID[2],6.0f,0.125f,0.01f,DM_POSITIONPIDKP[2],DM_POSITIONPIDKI[2],DM_POSITIONPIDKD[2],DM_POSITIONPIDKf[2],0.0f,0.0f,0.25f,0.0f,0.0f,0.0f,Integral_Limit|ChangingIntegralRate);
	/* 电机[3] */
	PID_Init(&Motor_DM_SPEED_PID[3],20.0f,1.5f,0.0f,DM_SPEEDPIDKP[3],DM_SPEEDPIDKI[3],DM_SPEEDPIDKD[3],DM_SPEEDPIDKf[3],DM_SPEEDPIDKffStaticPos[3],DM_SPEEDPIDKffStaticNeg[3],0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	PID_Init(&Motor_DM_POSITION_PID[3],15.0f,1.0f,0.01f,DM_POSITIONPIDKP[3],DM_POSITIONPIDKI[3],DM_POSITIONPIDKD[3],DM_POSITIONPIDKf[3],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);

	PID_Init(&Motor_LiftHeight_PID,6.0f, 0.0f, 0.0f, Motor_LiftHeight_kp, Motor_LiftHeight_ki, Motor_LiftHeight_kd, Motor_LiftHeight_kf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,Integral_Limit);


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
                              20.0f,
                              FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f,
                              1000.0f,
                              8U);
	}
}

/**
 * @brief 根据最大静态扭矩及其角度，计算任意位置的重力前馈
 * @param angle_rad   当前电机角度，单位：弧度
 * @param T_max       最大静态扭矩值（Nm），对应 angle_max 处的力矩
 * @param angle_max   出现最大静态扭矩时的电机角度，单位：弧度
 * @return            重力前馈力矩（Nm），正值表示与角度增加方向相同
 */
float  gravity_feedforward(float angle_rad, GravityFeedforward_Config *config) {
    // 重力矩模型：T_g = T_max * cos(angle - angle_max)
    // 前馈需抵消重力，所以取反
    return config->amplitude * cos(angle_rad + config->phase) + config->offset;
}

/** 
 * @brief 计算PID控制器
 * @param void
 * @return void
 */
void Motor_DM_CalPID()
{	
	float g_forward = 0.0f;
	if(Front_Calibrated && Back_Calibrated)
	{
		float follow_speed = 0.0f;

		/* 计算follow_speed（两电机共用），只要任一个电机需要闭环就计算 */
		if((LiftDown_CheckFlagFront == false) || (LiftDown_CheckFlagBack == false))
		{
			if(Robot_Mode == Robot_Mode_Manual)
			{
				follow_speed = PID_Calculate(&Motor_LiftHeight_PID, hipnuc_imu_data.eul[0],0.0f,0.001f);  //如果后面高，则out为负
			}
			if(Robot_Mode == Robot_Mode_Auto)
			{
				switch (LiftingState_t.state)
				{
				case LiftLevel200_Step2:
				if(Lift_HightFront == Down200_Front && Lift_HightBack == Down200_Back)
				{
					follow_speed = PID_Calculate(&Motor_LiftHeight_PID, hipnuc_imu_data.eul[0],0.0f,0.001f);
				}
					break;
				case  DownLevel200_Step7:
				if(Lift_HightFront == 0.0f && Lift_HightBack == 0.0f)
				{
					follow_speed = PID_Calculate(&Motor_LiftHeight_PID, hipnuc_imu_data.eul[0],0.0f,0.001f);
				}
				break;
				default:
				break;
				}
				/* 机械臂伸出/收回时，升降电机也要同时抬升/下降，需要跟随补偿 */
				if(GetKFS_State_t.state == GetKFS_Out_Process5
				|| GetKFS_State_t.state == GetKFS_Out_Process6
				|| GetKFS_State_t.state == GetKFS_Out_Process7)
				{
					follow_speed = PID_Calculate(&Motor_LiftHeight_PID, hipnuc_imu_data.eul[0],0.0f,0.001f);
				}
			}
		}

		/* 前电机[0]位置环 */
		if(LiftDown_CheckFlagFront == false)
		{
			if(Robot_Mode == Robot_Mode_Manual)
			{
				DM_Motor_1to4_Instances[0].Target_Omega = PID_Calculate(&Motor_DM3519_POS_PID[0], DM_Motor_1to4_Instances[0].Outch_Length, -DM_Motor_1to4_Instances[0].Target_Length, 0.001f)+follow_speed;
			}
			if(Robot_Mode == Robot_Mode_Auto)
			{
				DM_Motor_1to4_Instances[0].Target_Omega = PID_Calculate(&Motor_DM3519_POS_PID[0], DM_Motor_1to4_Instances[0].Outch_Length, -DM_Motor_1to4_Instances[0].Target_Length, 0.001f)+follow_speed;
			}
		}
		else
		{
			Motor_DM3519_POS_PID[0].ITerm=0.0f;
			Motor_DM3519_POS_PID[0].Iout=0.0f;
			Motor_DM3519_POS_PID[0].Output=0.0f;
		}

		/* 后电机[1]位置环 */
		if(LiftDown_CheckFlagBack == false)
		{
			if(Robot_Mode == Robot_Mode_Manual)
			{
				DM_Motor_1to4_Instances[1].Target_Omega = PID_Calculate(&Motor_DM3519_POS_PID[1], DM_Motor_1to4_Instances[1].Outch_Length, DM_Motor_1to4_Instances[1].Target_Length, 0.001f)+follow_speed;  //正向为输出，加负的out就是减速
			}
			if(Robot_Mode == Robot_Mode_Auto)
			{
				if(PC_frame.cmd_lift == 0x03)
				{
					DM_Motor_1to4_Instances[1].Target_Omega = PID_Calculate(&Motor_DM3519_POS_PID[1], DM_Motor_1to4_Instances[1].Outch_Length, DM_Motor_1to4_Instances[1].Target_Length+50.0f, 0.001f)+follow_speed;  //正向为输出，加负的out就是减速
				}
				else
				{
					DM_Motor_1to4_Instances[1].Target_Omega = PID_Calculate(&Motor_DM3519_POS_PID[1], DM_Motor_1to4_Instances[1].Outch_Length, DM_Motor_1to4_Instances[1].Target_Length, 0.001f)+follow_speed;  //正向为输出，加负的out就是减速
				}
			}
		}
		else
		{
			Motor_DM3519_POS_PID[1].ITerm=0.0f;
			Motor_DM3519_POS_PID[1].Iout=0.0f;
			Motor_DM3519_POS_PID[1].Output=0.0f;
		}


		float tmp_target2 = DM_Motor_Instances[3].Control_Angle;
		if(tmp_target2 > 8.00f)
		{
			DM_Motor_Instances[3].Control_Angle = 8.00f;
		}
		if(tmp_target2 < -7.00f)
		{
			DM_Motor_Instances[3].Control_Angle = -3.00f;
		}
		// float tmp_target2 = DM_Motor_Instances[3].Target_Angle;
		// if(tmp_target2 > 8.00f)
		// 	{
		// 		tmp_target2 = 8.00f;
		// 	}
		// if(tmp_target2 < -3.00f)
		// 	{
		// 		tmp_target2 = -3.00f;
		// 	}
		// DM_Motor_Instances[3].Target_Omega = PID_Calculate(&Motor_DM_POSITION_PID[3],
		// DM_Motor_Instances[3].Rx_Data.Now_Angle,
		// tmp_target2, 0.001f);
	}


	/* 前电机[0]速度环 */
	if(LiftDown_CheckFlagFront == false)
	{
		Filter_Frequency_Set_Now(&Motor_3519_Speed_Filter[0], DM_Motor_1to4_Instances[0].Rx_Data.Now_Omega);
    	Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_3519_Speed_Filter[0]);
    	DM_Motor_1to4_Instances[0].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_3519_Speed_Filter[0]);
    	DM_Motor_1to4_Instances[0].Out = PID_Calculate(&Motor_DM_1_To_4_PID[0], DM_Motor_1to4_Instances[0].Filtered_Omega, DM_Motor_1to4_Instances[0].Target_Omega, 0.001f);
	}
	else
	{
		/* 清空积分项，防止重新闭环时突跳 */
		Motor_DM_1_To_4_PID[0].ITerm = 0.0f;
		Motor_DM_1_To_4_PID[0].Iout = 0.0f;
		Motor_DM_1_To_4_PID[0].Output = 0.0f;
		DM_Motor_1to4_Instances[0].Out = LiftDownCheckFront_Torque;
	}

	/* 后电机[1]速度环 */
	if(LiftDown_CheckFlagBack == false)
	{
		Filter_Frequency_Set_Now(&Motor_3519_Speed_Filter[1], DM_Motor_1to4_Instances[1].Rx_Data.Now_Omega);
    	Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_3519_Speed_Filter[1]);
    	DM_Motor_1to4_Instances[1].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_3519_Speed_Filter[1]);
		DM_Motor_1to4_Instances[1].Out =PID_Calculate(&Motor_DM_1_To_4_PID[1], DM_Motor_1to4_Instances[1].Filtered_Omega, DM_Motor_1to4_Instances[1].Target_Omega, 0.001f);
	}
	else
	{
		/* 清空积分项，防止重新闭环时突跳 */
		Motor_DM_1_To_4_PID[1].ITerm = 0.0f;
		Motor_DM_1_To_4_PID[1].Iout = 0.0f;
		Motor_DM_1_To_4_PID[1].Output = 0.0f;
		DM_Motor_1to4_Instances[1].Out = LiftDownCheckBack_Torque;
	}
	
	
	float tmp_angle0 = DM_Motor_Instances[0].Target_Angle;
	{
		if(tmp_angle0 > 51.0f)
			{
				tmp_angle0 = 51.0f;
			}
		if(tmp_angle0 <= 0.0f)
			{
				tmp_angle0 = 0.0f;
			}
	}
	
	if(DM_Motor_Instances[0].Flag>0)
	{
		DM_Motor_Instances[0].Target_Omega = PID_Calculate(&Motor_DM_POSITION_PID[0],
			DM_Motor_Instances[0].Rx_Data.Now_Angle,
			-tmp_angle0, 0.001f);
		Filter_Frequency_Set_Now(&Motor_4310_Speed_Filter[0], DM_Motor_Instances[0].Rx_Data.Now_Omega);
    	Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_4310_Speed_Filter[0]);
    	DM_Motor_Instances[0].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_4310_Speed_Filter[0]);
    	DM_Motor_Instances[0].Control_Torque = PID_Calculate(&Motor_DM_SPEED_PID[0], DM_Motor_Instances[0].Filtered_Omega, DM_Motor_Instances[0].Target_Omega, 0.001f);
	}

	float tmp_angle1 = DM_Motor_Instances[1].Target_Angle;
	{
		if(tmp_angle1 > 20.0f)
			{
				tmp_angle1 = 20.0f;
			}
		if(tmp_angle1 <= 0.0f)
			{
				tmp_angle1 = 0.0f;
			}
	}

	if(DM_Motor_Instances[1].Flag>0)
	{
		DM_Motor_Instances[1].Target_Omega = PID_Calculate(&Motor_DM_POSITION_PID[1],
			DM_Motor_Instances[1].Rx_Data.Now_Angle,
			tmp_angle1, 0.001f);
		Filter_Frequency_Set_Now(&Motor_4310_Speed_Filter[1], DM_Motor_Instances[1].Rx_Data.Now_Omega);
		Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_4310_Speed_Filter[1]);
		DM_Motor_Instances[1].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_4310_Speed_Filter[1]);
		DM_Motor_Instances[1].Control_Torque = PID_Calculate(&Motor_DM_SPEED_PID[1], DM_Motor_Instances[1].Filtered_Omega, DM_Motor_Instances[1].Target_Omega, 0.001f);
	}

	float tmp_target = DM_Motor_Instances[2].Target_Angle;
	if(tmp_target > 3.40f)
	{
		tmp_target = 3.40f;
	}
	if(tmp_target < 0.0f)
	{
		tmp_target = 0.0f;
	}
	g_forward = gravity_feedforward(DM_Motor_Instances[2].Rx_Data.Now_Angle,&gravity_feedforward_config);
	DM_Motor_Instances[2].Target_Omega = PID_Calculate(&Motor_DM_POSITION_PID[2],
	DM_Motor_Instances[2].Rx_Data.Now_Angle,
	tmp_target, 0.001f);

	if(DM_Motor_Instances[2].Flag>0)
	{
		Filter_Frequency_Set_Now(&Motor_4310_Speed_Filter[2], DM_Motor_Instances[2].Rx_Data.Now_Omega);
		Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_4310_Speed_Filter[2]);
		DM_Motor_Instances[2].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_4310_Speed_Filter[2]);
		DM_Motor_Instances[2].Control_Torque = PID_Calculate(&Motor_DM_SPEED_PID[2], DM_Motor_Instances[2].Filtered_Omega, DM_Motor_Instances[2].Target_Omega, 0.001f)+(g_forward/0.315);
	}


	// if(DM_Motor_Instances[3].Flag>0)
	// {
	// 	Filter_Frequency_Set_Now(&Motor_4310_Speed_Filter[3], DM_Motor_Instances[3].Rx_Data.Now_Omega);
	// 	Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_4310_Speed_Filter[3]);
	// 	DM_Motor_Instances[3].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_4310_Speed_Filter[3]);
	// 	DM_Motor_Instances[3].Control_Torque = PID_Calculate(&Motor_DM_SPEED_PID[3], DM_Motor_Instances[3].Filtered_Omega, DM_Motor_Instances[3].Target_Omega, 0.001f);
	// }
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
	Motor_DM_Normal_Send_Enter(&DM_Motor_Instances[2]);
	osDelay(10);
	Motor_DM_Normal_Send_Enter(&DM_Motor_Instances[3]);
	osDelay(10);
	Motor_DM_Normal_Send_Enter(&DM_Motor_Instances[0]);
	osDelay(10);
	Motor_DM_Normal_Send_Enter(&DM_Motor_Instances[1]);
	osDelay(10);

	/* 首次上电保存电机零点到驱动器flash（仅执行一次，之后注释掉或删掉） */
	Motor_DM_Normal_Send_Save_Zero(&DM_Motor_Instances[2]);
	osDelay(10);
	Motor_DM_Normal_Send_Save_Zero(&DM_Motor_Instances[3]);
	osDelay(10);
	Motor_DM_Normal_Send_Save_Zero(&DM_Motor_Instances[0]);
	osDelay(10);
	Motor_DM_Normal_Send_Save_Zero(&DM_Motor_Instances[1]);
	for(;;)
	{  
		// DM_Motor_1to4_Instances[1].Out=test_out;
		switch(Robot_Mode)
		{
		case Robot_Mode_Stop:
			DM_Motor_1to4_Instances[0].Target_Omega=0.0f;
			DM_Motor_1to4_Instances[1].Target_Omega=0.0f;
			DM_Motor_Instances[3].Control_Omega =1.5f;
			DM_Motor_Instances[0].Target_Angle = 0.0f;
			// DM_Motor_Instances[1].Target_Omega = 0.0f;
			// DM_Motor_Instances[1].Target_Angle = Default_anglef;
			// DM_Motor_Instances[2].Target_Angle = Default_angleb;	   //2.0 0.45
			// DM_Motor_Instances[3].Control_Angle = Default_angleh;   //0.35 -0.6
			DM_Motor_Instances[1].Target_Angle = 0.0f;
			DM_Motor_Instances[2].Target_Angle = 2.0;	   //2.0 0.45  收:0.8
			DM_Motor_Instances[3].Control_Angle = -0.35;   //0.35 -0.6   -1.05
			break;
		case Robot_Mode_Manual:
			// DM_Motor_1to4_Instances[0].Target_Omega=10.0f;
			// DM_Motor_1to4_Instances[1].Out = test_out;f
			// TransData_Send_Two_Float_Frame(&huart1, DM_Motor_Instances[0].Control_Torque, DM_Motor_Instances[0].Filtered_Omega, 2);
			if(rc_channels.ch[6]<=0)
			{	
				if(rc_channels.ch[13]<=0)
				{
					DM_Motor_Instances[3].Control_Omega = 0.0f;
				}
				else
				{				
					DM_Motor_Instances[3].Control_Omega = 3.0f;
				}
				// DM_Motor_Instances[0].Target_Angle = 5.0f+(50.0f/820)*(rc_channels.ch[11]);
				DM_Motor_Instances[0].Target_Angle = 39.0f;
				DM_Motor_Instances[1].Target_Angle = (17.0f/800.0f)*(rc_channels.ch[2]);
				DM_Motor_Instances[2].Target_Angle = (3.4f/820) * rc_channels.ch[10];
				if(rc_channels.ch[14]<=0)
				{
					DM_Motor_Instances[3].Control_Angle = Default_anglef;  // default 3.65 -0.35
				}
				else
				{
					DM_Motor_Instances[3].Control_Angle = Default_angleb;  // default -1.75
				}
				DM_Motor_1to4_Instances[0].Target_Length = -10.0f;
				DM_Motor_1to4_Instances[1].Target_Length = 0.0f;

				// DM_Motor_Instances[1].Target_Angle = Default_angleb;
			}
			else
			{
				DM_Motor_Instances[0].Target_Angle = 5.0f;
				// DM_Motor_Instances[1].Target_Angle = Default_angleb;
				// test_out = ALG_Sin_Generate(&test_out, 8.0f, 3.0f, 1000.0f);
				// DM_Motor_Instances[0].Target_Omega = -test_out;
				// DM_Motor_Instances[2].Target_Omega = test_out;
				// DM_Motor_1to4_Instances[0].Target_Length = test_remote_ch2*70;
				DM_Motor_1to4_Instances[0].Target_Length =202;
				// DM_Motor_1to4_Instances[1].Target_Length = test_remote_ch2*70;
				// Default_angleb=Default_anglef-9.0f;
				// if(Default_angleb<0.0f)
				// {
				// 	Default_angleb=0.0f;
				// }
				DM_Motor_1to4_Instances[1].Target_Length =194;
			}
			break;
		case Robot_Mode_Auto:
			if(GetKFS_State_t.state == GetKFS_Out_Process5 ||GetKFS_State_t.state == GetKFS_Out_Process6)
			{
				DM_Motor_1to4_Instances[0].Target_Length = 208.0f;
				DM_Motor_1to4_Instances[1].Target_Length = 200.0f;
			}
			else
			{
				DM_Motor_1to4_Instances[0].Target_Length = Lift_HightFront;
				DM_Motor_1to4_Instances[1].Target_Length = Lift_HightBack;
			}
			DM_Motor_Instances[3].Control_Omega = 1.5f;
			if(GetWeapon_State_t.state==GetWeapon_Idle){
				switch(GetKFS_State_t.state)
				{
					case GetKFS_Idle:
					DM_Motor_Instances[0].Target_Angle = 39.0f;
					DM_Motor_Instances[2].Target_Angle = 2.0f;	   //2.0
					DM_Motor_Instances[3].Control_Angle = -0.35f;   //0.35
					break;
					case GetKFS_Process0:
					case GetKFS_Process1:
					case GetKFS_Process1_2:
					case GetKFS_Process2:
					case GetKFS_Process3:
					case GetKFS_Process4:
					case GetKFS_Down_Process0:
					case GetKFS_Down_Process1:
					case GetKFS_Down_Process2:
					case GetKFS_Down_Process3:
					case GetKFS_Out_Process0:
					case GetKFS_Out_Process1:
					case GetKFS_Out_Process2:
					case GetKFS_Out_Process3:
					case GetKFS_Out_Process4:
					case GetKFS_Out_Process5:
					case GetKFS_Out_Process6:
					case GetKFS_Out_Process7:
					case GetKFS_Save_Process0:
					case GetKFS_Save_Process1:
					case GetKFS_Save_Process1_2:
					case GetKFS_Save_Process2:
					case GetKFS_Save_Process3:
					case GetKFS_Save_Down_Process0:
					case GetKFS_Save_Down_Process1:
					case GetKFS_Save_Down_Process2:
					case GetKFS_Save_Down_Process3:
					case GetKFS_Save_Down_Process4:
					case GetKFS_High_Process0:
					case GetKFS_High_Process1:
					case GetKFS_High_Process1_2:
					case GetKFS_High_Process2:
					case GetKFS_High_Process3:
					case GetKFS_High_Process4:
					case GetKFS_Done_Save:
					case GetKFS_Done:
					DM_Motor_Instances[0].Target_Angle = Arm_Autocmd.Motor0_Height;
					DM_Motor_Instances[1].Target_Angle = Arm_Autocmd.Motor1_X;
					DM_Motor_Instances[2].Target_Angle = Arm_Autocmd.Motor2_Arm;
					DM_Motor_Instances[3].Control_Angle = Arm_Autocmd.Motor3_Suctiom;
					break;
				}}
		else{
			  switch(GetWeapon_State_t.state)
			  {
				case GetWeapon_Process2_5:
				case GetWeapon_Process3:
				case GetWeapon_Process4:
				DM_Motor_Instances[0].Target_Angle = 48.0f;
				DM_Motor_Instances[1].Target_Angle = 0.0f;
				DM_Motor_Instances[2].Target_Angle = 0.0f;
				DM_Motor_Instances[3].Control_Angle = 1.0f;
				break;
				default:
				DM_Motor_Instances[0].Target_Angle = 40.0f;
				DM_Motor_Instances[1].Target_Angle = 0.0f;
				DM_Motor_Instances[2].Target_Angle = 0.0f;
				DM_Motor_Instances[3].Control_Angle = 1.0f;
				break;
			  }
		}
		break;
		}
		// Motor_DM_CalPID();
		// Motor_DM_1_To_4_Output();
		osDelay(1);
	}
}
