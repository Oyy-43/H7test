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

 /* Private variables ---------------------------------------------------------*/
 float DM_PIDKP[DM_Motor_1_To_4_Num] = {10.68f, 13.35f, 13.35f, 10.68f, 10.68f, 10.68f};  //7.121
 float DM_PIDKI[DM_Motor_1_To_4_Num] = {0.01586f, 0.02000f, 0.02000f, 0.01586f, 0.01586f, 0.01586f}; //0.01186
 float DM_PIDKD[DM_Motor_1_To_4_Num] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  //0.0
 float DM_PIDKf[DM_Motor_1_To_4_Num] = {0.46875f, 0.46875f, 0.46875f, 0.46875f, 0.46875f, 0.46875f}; //0.3125
 float Ts = 0.001f;
 float t = 0.0f;           // 时间累积变量
 float f = 1.0f;           // 当前频率（可随时间变化）
 float A = 6.204645f;      // 幅值，单位RPM或%额定速度  //2.068215f
 float speed_ref = 0.0f;   // 目标速度

/* DMA1不能访问DTCM，串口DMA发送缓冲放在RAM_D2并按Cache line对齐 */
#define DM_UART1_TX_FRAME_MAX_LEN    96U
#define DM_UART1_TX_QUEUE_DEPTH      16U

static uint8_t uart1_tx_queue[DM_UART1_TX_QUEUE_DEPTH][DM_UART1_TX_FRAME_MAX_LEN] RAM_D2_BUFFER;
static uint16_t uart1_tx_len_queue[DM_UART1_TX_QUEUE_DEPTH] = {0};
static volatile uint8_t uart1_tx_head = 0;
static volatile uint8_t uart1_tx_tail = 0;
static volatile uint8_t uart1_tx_count = 0;
static volatile uint8_t uart1_tx_busy = 0;

static uint8_t uart1_tx_pending_buf[DM_UART1_TX_FRAME_MAX_LEN] RAM_D2_BUFFER;
static uint16_t uart1_tx_pending_len = 0;
static uint8_t uart1_tx_pending_valid = 0;

static uint8_t DM_UART1_QueuePush(const uint8_t *data, uint16_t len);
static void DM_UART1_TryStartTx(void);

static uint8_t DM_UART1_QueuePush(const uint8_t *data, uint16_t len)
{
	uint8_t ok = 0;
	uint32_t primask = __get_PRIMASK();

	if (len == 0U || len > DM_UART1_TX_FRAME_MAX_LEN)
	{
		return 0;
	}

	__disable_irq();
	if (uart1_tx_count < DM_UART1_TX_QUEUE_DEPTH)
	{
		uint8_t idx = uart1_tx_head;
		memcpy(uart1_tx_queue[idx], data, len);
		uart1_tx_len_queue[idx] = len;
		uart1_tx_head = (uint8_t)((uart1_tx_head + 1U) % DM_UART1_TX_QUEUE_DEPTH);
		uart1_tx_count++;
		ok = 1;
	}
	if (primask == 0U)
	{
		__enable_irq();
	}

	return ok;
}

static void DM_UART1_TryStartTx(void)
{
	uint8_t idx = 0;
	uint16_t len = 0;
	HAL_StatusTypeDef ret;
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	if ((uart1_tx_busy != 0U) || (uart1_tx_count == 0U))
	{
		if (primask == 0U)
		{
			__enable_irq();
		}
		return;
	}

	idx = uart1_tx_tail;
	len = uart1_tx_len_queue[idx];
	uart1_tx_busy = 1U;
	if (primask == 0U)
	{
		__enable_irq();
	}

	/* D-Cache开启时，发送前清Cache，保证DMA读到最新数据 */
	{
		if (((SCB->CCR & SCB_CCR_DC_Msk) != 0U) && (len > 0U))
		{
			uint32_t addr = (uint32_t)&uart1_tx_queue[idx][0];
			uint32_t size = (uint32_t)len;
			uint32_t aligned_addr = addr & ~((uint32_t)31U);
			uint32_t aligned_size = ((addr + size + 31U) & ~((uint32_t)31U)) - aligned_addr;
			SCB_CleanDCache_by_Addr((uint32_t *)aligned_addr, (int32_t)aligned_size);
		}
	}

	ret = HAL_UART_Transmit_DMA(&huart1, uart1_tx_queue[idx], len);
	if (ret != HAL_OK)
	{
		primask = __get_PRIMASK();
		__disable_irq();
		uart1_tx_busy = 0U;
		if (primask == 0U)
		{
			__enable_irq();
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t has_pending = 0;
	uint32_t primask = __get_PRIMASK();

	if (huart->Instance != USART1)
	{
		return;
	}

	__disable_irq();
	if (uart1_tx_count > 0U)
	{
		uart1_tx_tail = (uint8_t)((uart1_tx_tail + 1U) % DM_UART1_TX_QUEUE_DEPTH);
		uart1_tx_count--;
	}
	uart1_tx_busy = 0U;
	has_pending = (uart1_tx_count > 0U) ? 1U : 0U;
	if (primask == 0U)
	{
		__enable_irq();
	}

	if (has_pending != 0U)
	{
		DM_UART1_TryStartTx();
	}
}

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
	DM_Motor_1to4_Instances[0].Out=PID_Calculate(&Motor_DM_1_To_4_PID[0], DM_Motor_1to4_Instances[0].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[0].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[1].Out=PID_Calculate(&Motor_DM_1_To_4_PID[1], DM_Motor_1to4_Instances[1].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[1].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[2].Out=PID_Calculate(&Motor_DM_1_To_4_PID[2], DM_Motor_1to4_Instances[2].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[2].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[3].Out=PID_Calculate(&Motor_DM_1_To_4_PID[3], DM_Motor_1to4_Instances[3].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[3].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[4].Out=PID_Calculate(&Motor_DM_1_To_4_PID[4], DM_Motor_1to4_Instances[4].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[4].Target_Omega,0.001f);
	DM_Motor_1to4_Instances[5].Out=PID_Calculate(&Motor_DM_1_To_4_PID[5], DM_Motor_1to4_Instances[5].Rx_Data.Now_Omega, DM_Motor_1to4_Instances[5].Target_Omega,0.001f);
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
 * @brief 正弦信号填入
 * 
 */
void Motor_DM_1_To_4_SinOutput(DM_Motor_1to4_Instance *motor_instance)
{
	float w = 2.0f * 3.1415926f * f;
    speed_ref = A * sin(w * t);
    // 3. 限幅保护（防止设定值超出安全范围）
    if (speed_ref > 6.204645f) speed_ref = 6.204645f;
    if (speed_ref < -6.204645f) speed_ref = -6.204645f;

    // 4. 送入速度环控制器
	motor_instance->Target_Omega = speed_ref;

    // 5. 更新时间
    t += Ts;
    if (t > 30.0f) t = 0; // 循环或停止++
}	

void Send_Data(DM_Motor_1to4_Instance *motor_instance)
{
	int len = 0;
	uint8_t frame_buf[DM_UART1_TX_FRAME_MAX_LEN];
	int32_t output_milli = 0;
	int32_t input_milli = 0;
	int32_t output_abs_milli = 0;
	int32_t input_abs_milli = 0;
	char output_sign = '+';
	char input_sign = '+';
	uint16_t frame_len = 0;

	/* 先尝试把上次未入队的数据补入队列，避免丢帧 */
	if (uart1_tx_pending_valid != 0U)
	{
		if (DM_UART1_QueuePush(uart1_tx_pending_buf, uart1_tx_pending_len) != 0U)
		{
			uart1_tx_pending_valid = 0U;
			uart1_tx_pending_len = 0U;
		}
		else
		{
			DM_UART1_TryStartTx();
			return;
		}
	}

	// 每次调用都尝试发送（任务周期约1ms）

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

	len = snprintf((char *)frame_buf, sizeof(frame_buf),
		"output=%c%ld.%03ld,input=%c%ld.%03ld\r\n",
		output_sign,
		(long)(output_abs_milli / 1000),
		(long)(output_abs_milli % 1000),
		input_sign,
		(long)(input_abs_milli / 1000),
		(long)(input_abs_milli % 1000));

	if (len > 0)
	{
		if (len > (int)sizeof(frame_buf))
		{
			len = sizeof(frame_buf);
		}
		frame_len = (uint16_t)len;

		if (DM_UART1_QueuePush(frame_buf, frame_len) == 0U)
		{
			memcpy(uart1_tx_pending_buf, frame_buf, frame_len);
			uart1_tx_pending_len = frame_len;
			uart1_tx_pending_valid = 1U;
		}
		DM_UART1_TryStartTx();
	}
}
 /* Function prototypes -------------------------------------------------------*/
void DMsetOutput(void *argument)
{
	for(;;)
	{  
		// Send_Data(&DM_Motor_1to4_Instances[0]);
		// if(rc_channels.ch[4]<=0)
		// {
		// 	DM_Motor_1to4_Instances[0].Target_Omega = 0;
		// 	DM_Motor_1to4_Instances[1].Target_Omega = 0;
		// 	DM_Motor_1to4_Instances[2].Target_Omega = 0;
		// 	DM_Motor_1to4_Instances[3].Target_Omega = 0;
		// 	DM_Motor_1to4_Instances[4].Target_Omega = 0;
		// 	DM_Motor_1to4_Instances[5].Target_Omega = 0;
		// }
		// else
		// {
		//  Motor_DM_1_To_4_SinOutput(&DM_Motor_1to4_Instances[0]);
		// 	// DM_Motor_1to4_Instances[0].Target_Omega = 5.0f;
		// 	// DM_Motor_1to4_Instances[1].Target_Omega = 5.0f;
		// }
		Motor_DM_CalPID();
		Motor_DM_1_To_4_Output();
		osDelay(1);
	}
}
 