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
 
 /* Private macros ------------------------------------------------------------*/
 

 /* Private types -------------------------------------------------------------*/
 PID_TypeDef Motor_DM_1_To_4_PID[DM_Motor_1_To_4_Num];

 /* Private variables ---------------------------------------------------------*/
 float DM_PIDKP[DM_Motor_1_To_4_Num] = {7.121f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
 float DM_PIDKI[DM_Motor_1_To_4_Num] = {0.01186f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
 float DM_PIDKD[DM_Motor_1_To_4_Num] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; float DM_PIDKf[DM_Motor_1_To_4_Num] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
 float Ts = 0.001f;
 float t = 0.0f;           // 时间累积变量
 float f = 1.0f;           // 当前频率（可随时间变化）
 float A = 4.136430f;      // 幅值，单位RPM或%额定速度
 float speed_ref = 0.0f;   // 目标速度
 float target_acc;
 float feedforward = 0.0f;
 float kf=0.05f;

/* DMA1不能访问DTCM，串口DMA发送缓冲放在RAM_D2并按Cache line对齐 */
static uint8_t uart1_tx_buf[96] RAM_D2_BUFFER;

 /* Private function declarations ---------------------------------------------*/
/** 
 * @brief 初始化PID控制器
 * @param void
 * @return void
 */
void Motor_DM_InitPID()
{
	PID_Init(&Motor_DM_1_To_4_PID[0],16384.0f,1000.0f,0.0f,DM_PIDKP[0],DM_PIDKI[0],DM_PIDKD[0],DM_PIDKf[0],0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[1],16384.0f,1000.0f,0.0f,DM_PIDKP[1],DM_PIDKI[1],DM_PIDKD[1],DM_PIDKf[1],0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[2],16384.0f,1000.0f,0.0f,DM_PIDKP[2],DM_PIDKI[2],DM_PIDKD[2],DM_PIDKf[2],0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[3],16384.0f,1000.0f,0.0f,DM_PIDKP[3],DM_PIDKI[3],DM_PIDKD[3],DM_PIDKf[3],0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[4],16384.0f,1000.0f,0.0f,DM_PIDKP[4],DM_PIDKI[4],DM_PIDKD[4],DM_PIDKf[4],0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
	PID_Init(&Motor_DM_1_To_4_PID[5],16384.0f,1000.0f,0.0f,DM_PIDKP[5],DM_PIDKI[5],DM_PIDKD[5],DM_PIDKf[5],0.0f,0.0f,0.0f,0.0f,Integral_Limit|ErrorHandle);
}
/** 
 * @brief 计算PID控制器
 * @param void
 * @return void
 */
void Motor_DM_CalPID()
{
   DM_Motor_1to4_Instances[0].Out=PID_Calculate(&Motor_DM_1_To_4_PID[0], DM_Motor_1to4_Instances[0].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[0].Target_Omega);
   DM_Motor_1to4_Instances[1].Out=PID_Calculate(&Motor_DM_1_To_4_PID[1], DM_Motor_1to4_Instances[1].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[1].Target_Omega);
   DM_Motor_1to4_Instances[2].Out=PID_Calculate(&Motor_DM_1_To_4_PID[2], DM_Motor_1to4_Instances[2].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[2].Target_Omega);
   DM_Motor_1to4_Instances[3].Out=PID_Calculate(&Motor_DM_1_To_4_PID[3], DM_Motor_1to4_Instances[3].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[3].Target_Omega);
   DM_Motor_1to4_Instances[4].Out=PID_Calculate(&Motor_DM_1_To_4_PID[4], DM_Motor_1to4_Instances[4].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[4].Target_Omega);
   DM_Motor_1to4_Instances[5].Out=PID_Calculate(&Motor_DM_1_To_4_PID[5], DM_Motor_1to4_Instances[5].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[5].Target_Omega);
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
	CAN_Transmit_Data(&hfdcan1,0x1FF,CAN1_0x1ff_Tx_Data,8);
}

/**
 * @brief 正弦信号填入
 * 
 */
void Motor_DM_1_To_4_SinOutput(DM_Motor_1to4_Instance *motor_instance)
{
	float w = 2.0f * 3.1415926f * f;
    speed_ref = A * sin(w * t);
	target_acc = A * w * cos(w * t);
    feedforward = kf * target_acc;
    // 3. 限幅保护（防止设定值超出安全范围）
    if (speed_ref > 4.136430f) speed_ref = 4.136430f;
    if (speed_ref < -4.136430f) speed_ref = -4.136430f;

    // 4. 送入速度环控制器
    motor_instance->Target_Omega = speed_ref+feedforward;

    // 5. 更新时间
    t += Ts;
    if (t > 30.0f) t = 0; // 循环或停止
}	

void Send_Data(DM_Motor_1to4_Instance *motor_instance)
{
	static uint8_t divider = 0;
	int len = 0;
	int32_t output_milli = 0;
	int32_t input_milli = 0;
	int32_t output_abs_milli = 0;
	int32_t input_abs_milli = 0;
	char output_sign = '+';
	char input_sign = '+';

	// 10ms发送一次，降低DMA忙冲突概率
	divider++;
	if (divider < 10)
	{
		return;
	}
	divider = 0;

	/* 避免DMA发送期间重写缓冲区 */
	if (huart1.gState != HAL_UART_STATE_READY)
	{
		return;
	}

	output_milli = (int32_t)(motor_instance->Rx_Data.Now_Omega * 1000.0f);
	input_milli = (int32_t)(motor_instance->Target_Omega * 1000.0f);

	if (output_milli < 0)
	{
		output_sign = '-';
		output_abs_milli = -output_milli;
	}
	else
	{
		output_abs_milli = output_milli;
	}

	if (input_milli < 0)
	{
		input_sign = '-';
		input_abs_milli = -input_milli;
	}
	else
	{
		input_abs_milli = input_milli;
	}

	len = snprintf((char *)uart1_tx_buf, sizeof(uart1_tx_buf),
		"output=%c%ld.%03ld,input=%c%ld.%03ld\r\n",
		output_sign,
		(long)(output_abs_milli / 1000),
		(long)(output_abs_milli % 1000),
		input_sign,
		(long)(input_abs_milli / 1000),
		(long)(input_abs_milli % 1000));

	if (len > 0)
	{
		if (len > (int)sizeof(uart1_tx_buf))
		{
			len = sizeof(uart1_tx_buf);
		}

		/* D-Cache开启时，发送前清Cache，保证DMA读到最新数据 */
		{
			uint32_t addr = (uint32_t)uart1_tx_buf;
			uint32_t size = (uint32_t)len;
			uint32_t aligned_addr = addr & ~((uint32_t)31U);
			uint32_t aligned_size = ((addr + size + 31U) & ~((uint32_t)31U)) - aligned_addr;
			SCB_CleanDCache_by_Addr((uint32_t *)aligned_addr, (int32_t)aligned_size);
		}

		(void)HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, (uint16_t)len);
	}
}
 /* Function prototypes -------------------------------------------------------*/
void DMsetOutput(void *argument)
{
	for(;;)
	{  
		// Send_Data(&DM_Motor_1to4_Instances[0]);
		Motor_DM_1_To_4_SinOutput(&DM_Motor_1to4_Instances[0]);
		Motor_DM_CalPID();
		Motor_DM_1_To_4_Output();
		osDelay(1);
	}
}
 