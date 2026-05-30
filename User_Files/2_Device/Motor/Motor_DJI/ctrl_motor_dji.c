/**
 * @file ctrl_motor_dji.c
 * @author Oyyp
 * @brief 对大疆电机的控制
 * @version 0.1
 * @date 2026-04-18 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "ctrl_motor_dji.h"
#include "FreeRTOS.h"
#include "task.h"
#include "1_Middleware/Algorithm/Filter/Frequency/alg_filter_frequency.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
PID_TypeDef Motor_2006_SPEED_PID[DJI_Motor_C610_Num];
PID_TypeDef Motor_3508_SPEED_PID[DJI_Motor_C620_Num];
/* Private variables ---------------------------------------------------------*/
float PID_3508_SPEEDKP[DJI_Motor_C620_Num] = {812.3f, 812.3f, 812.3f, 812.3f};  //0.2732 0.6664
float PID_3508_SPEEDKI[DJI_Motor_C620_Num] = {0.9f, 0.9f, 0.9f, 0.9f};
float PID_3508_SPEEDKD[DJI_Motor_C620_Num] = {0.0f, 0.0f, 0.0f, 0.0f};
float PID_3508_SPEEDKf[DJI_Motor_C620_Num] = {25.0f, 25.0f, 25.0f, 25.0f};
float PID_2006_SPEEDKP[DJI_Motor_C610_Num] = {565.5f, 565.5f};  //0.2732 0.6664
float PID_2006_SPEEDKI[DJI_Motor_C610_Num] = {0.9f, 0.9f};
float PID_2006_SPEEDKD[DJI_Motor_C610_Num] = {0.0f, 0.0f};
float PID_2006_SPEEDKf[DJI_Motor_C610_Num] = {22.5f, 22.5f};
float Sin_Target;
float test_Target;
Struct_Filter_Frequency Motor_2006_Speed_Filter[DJI_Motor_C610_Num];
Struct_Filter_Frequency Motor_3508_Speed_Filter[DJI_Motor_C620_Num];
/* Private function declarations ---------------------------------------------*/
void Motor_DJI_InitPID()
{
    uint8_t i;

    PID_Init(&Motor_3508_SPEED_PID[0],16384.0f,2000.0f,0.0f,PID_3508_SPEEDKP[0],PID_3508_SPEEDKI[0],PID_3508_SPEEDKD[0],PID_3508_SPEEDKf[0],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&Motor_3508_SPEED_PID[1],16384.0f,2000.0f,0.0f,PID_3508_SPEEDKP[1],PID_3508_SPEEDKI[1],PID_3508_SPEEDKD[1],PID_3508_SPEEDKf[1],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit); 
    PID_Init(&Motor_3508_SPEED_PID[2],16384.0f,2000.0f,0.0f,PID_3508_SPEEDKP[2],PID_3508_SPEEDKI[2],PID_3508_SPEEDKD[2],PID_3508_SPEEDKf[2],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&Motor_3508_SPEED_PID[3],16384.0f,2000.0f,0.0f,PID_3508_SPEEDKP[3],PID_3508_SPEEDKI[3],PID_3508_SPEEDKD[3],PID_3508_SPEEDKf[3],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
	PID_Init(&Motor_2006_SPEED_PID[0],10000.0f,1000.0f,0.0f,PID_2006_SPEEDKP[0],PID_2006_SPEEDKI[0],PID_2006_SPEEDKD[0],PID_2006_SPEEDKf[0],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&Motor_2006_SPEED_PID[1],10000.0f,1000.0f,0.0f,PID_2006_SPEEDKP[1],PID_2006_SPEEDKI[1],PID_2006_SPEEDKD[1],PID_2006_SPEEDKf[1],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);

    
    for (i = 0U; i < DJI_Motor_C610_Num; i++)
    {
        Filter_Frequency_Init(&Motor_2006_Speed_Filter[i],
                              0.0f,
                              0.0f,
                              Filter_Frequency_Type_LOWPASS,
                              70.0f,
                              FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f,
                              1000.0f,
                              8U);
    }

    for (i = 0U; i < DJI_Motor_C620_Num; i++)
    {
        Filter_Frequency_Init(&Motor_3508_Speed_Filter[i],
                              0.0f,
                              0.0f,
                              Filter_Frequency_Type_LOWPASS,
                              70.0f,
                              FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f,
                              1000.0f,
                              8U);
    }
}

void Motor_DJI_CalPID()
{
    Filter_Frequency_Set_Now(&Motor_2006_Speed_Filter[0], DJI_Motor_Instances[4].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_2006_Speed_Filter[0]);
    DJI_Motor_Instances[4].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_2006_Speed_Filter[0]);
    DJI_Motor_Instances[4].Out = PID_Calculate(&Motor_2006_SPEED_PID[0], DJI_Motor_Instances[4].Filtered_Omega, DJI_Motor_Instances[4].Target_Omega, 0.001f);

    Filter_Frequency_Set_Now(&Motor_2006_Speed_Filter[1], DJI_Motor_Instances[5].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_2006_Speed_Filter[1]);
    DJI_Motor_Instances[5].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_2006_Speed_Filter[1]);
    DJI_Motor_Instances[5].Out = PID_Calculate(&Motor_2006_SPEED_PID[1], DJI_Motor_Instances[5].Filtered_Omega, DJI_Motor_Instances[5].Target_Omega, 0.001f);

    //3508底盘速度低通滤波器
    Filter_Frequency_Set_Now(&Motor_3508_Speed_Filter[0], DJI_Motor_Instances[0].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_3508_Speed_Filter[0]);
    DJI_Motor_Instances[0].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_3508_Speed_Filter[0]);
    DJI_Motor_Instances[0].Out = PID_Calculate(&Motor_3508_SPEED_PID[0], DJI_Motor_Instances[0].Filtered_Omega, DJI_Motor_Instances[0].Target_Omega, 0.001f);

    Filter_Frequency_Set_Now(&Motor_3508_Speed_Filter[1], DJI_Motor_Instances[1].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_3508_Speed_Filter[1]);
    DJI_Motor_Instances[1].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_3508_Speed_Filter[1]);
    DJI_Motor_Instances[1].Out = PID_Calculate(&Motor_3508_SPEED_PID[1], DJI_Motor_Instances[1].Filtered_Omega, DJI_Motor_Instances[1].Target_Omega, 0.001f);

    Filter_Frequency_Set_Now(&Motor_3508_Speed_Filter[2], DJI_Motor_Instances[2].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_3508_Speed_Filter[2]);
    DJI_Motor_Instances[2].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_3508_Speed_Filter[2]);
    DJI_Motor_Instances[2].Out = PID_Calculate(&Motor_3508_SPEED_PID[2], DJI_Motor_Instances[2].Filtered_Omega, DJI_Motor_Instances[2].Target_Omega, 0.001f);

    Filter_Frequency_Set_Now(&Motor_3508_Speed_Filter[3], DJI_Motor_Instances[3].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_3508_Speed_Filter[3]);
    DJI_Motor_Instances[3].Filtered_Omega = Filter_Frequency_Get_Out(&Motor_3508_Speed_Filter[3]);
    DJI_Motor_Instances[3].Out = PID_Calculate(&Motor_3508_SPEED_PID[3], DJI_Motor_Instances[3].Filtered_Omega, DJI_Motor_Instances[3].Target_Omega, 0.001f);
}

void DJI_Motor_Output()
{
    Motor_DJI_CalPID();
    Motor_DJI_SetOutput();
}
/* Function prototypes -------------------------------------------------------*/
void DJISetOut(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t period_ticks = pdMS_TO_TICKS(1U);

    if (period_ticks == 0U)
    {
        period_ticks = 1U;
    }

    for(;;)
    {
        Sin_Target = ALG_Sin_Generate(&Sin_Target, 3.0f, 10.0f, 1000.0f);
        // ALG_Value_Toggle_Periodic(&Sin_Target, 10.0f, -10.0f, 2.0f, 1000.0f);
        // DJI_Motor_Instances[0].Target_Omega = 10.0f;
        // DJI_Motor_Instances[1].Target_Omega = 10.0f;
        // DJI_Motor_Instances[2].Target_Omega = 10.0f;
        // DJI_Motor_Instances[3].Target_Omega = 12.0f;
        // DJI_Motor_Instances[4].Target_Omega = Sin_Target;
        // DJI_Motor_Instances[5].Target_Omega = -10.0f;
        // Motor_DJI_CalPID();
        // Motor_DJI_Output();
        // TransData_Send_Two_Float_Frame(&huart1, DJI_Motor_Instances[3].Out, DJI_Motor_Instances[3].Filtered_Omega, 2);
        xTaskDelayUntil(&last_wake_time, period_ticks);
    }
}
