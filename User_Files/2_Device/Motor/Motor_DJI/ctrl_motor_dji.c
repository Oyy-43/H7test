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
/* Private variables ---------------------------------------------------------*/
float PID_SPEEDKP[DJI_Motor_C610_Num] = {3.732f, 3.732f, 3.732f, 3.732f};  //0.2732 0.6664
float PID_SPEEDKI[DJI_Motor_C610_Num] = {0.01f, 0.01f, 0.01f, 0.01f};
float PID_SPEEDKD[DJI_Motor_C610_Num] = {0.0f, 0.0f, 0.0f, 0.0f};
float PID_SPEEDKf[DJI_Motor_C610_Num] = {0.03f, 0.03f, 0.03f, 0.03f};
float Sin_Target;
float test_Target;
Struct_Filter_Frequency Motor_2006_Speed_Filter[DJI_Motor_C610_Num];
/* Private function declarations ---------------------------------------------*/
void Motor_DJI_InitPID()
{
    uint8_t i;

	PID_Init(&Motor_2006_SPEED_PID[0],10000.0f,1000.0f,0.0f,PID_SPEEDKP[0],PID_SPEEDKI[0],PID_SPEEDKD[0],PID_SPEEDKf[0],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&Motor_2006_SPEED_PID[1],10000.0f,1000.0f,0.0f,PID_SPEEDKP[1],PID_SPEEDKI[1],PID_SPEEDKD[1],PID_SPEEDKf[1],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&Motor_2006_SPEED_PID[2],10000.0f,1000.0f,0.0f,PID_SPEEDKP[2],PID_SPEEDKI[2],PID_SPEEDKD[2],PID_SPEEDKf[2],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&Motor_2006_SPEED_PID[3],10000.0f,1000.0f,0.0f,PID_SPEEDKP[3],PID_SPEEDKI[3],PID_SPEEDKD[3],PID_SPEEDKf[3],0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);

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
}

void Motor_DJI_CalPID()
{
    float filtered_omega;

    Filter_Frequency_Set_Now(&Motor_2006_Speed_Filter[0], DM_Motor_C610_Instances[0].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_2006_Speed_Filter[0]);
    filtered_omega = Filter_Frequency_Get_Out(&Motor_2006_Speed_Filter[0]);
    DM_Motor_C610_Instances[0].Out = PID_Calculate(&Motor_2006_SPEED_PID[0], filtered_omega, DM_Motor_C610_Instances[0].Target_Omega, 0.001f);

    Filter_Frequency_Set_Now(&Motor_2006_Speed_Filter[1], DM_Motor_C610_Instances[1].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_2006_Speed_Filter[1]);
    filtered_omega = Filter_Frequency_Get_Out(&Motor_2006_Speed_Filter[1]);
    DM_Motor_C610_Instances[1].Out = PID_Calculate(&Motor_2006_SPEED_PID[1], filtered_omega, DM_Motor_C610_Instances[1].Target_Omega, 0.001f);

    Filter_Frequency_Set_Now(&Motor_2006_Speed_Filter[2], DM_Motor_C610_Instances[2].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_2006_Speed_Filter[2]);
    filtered_omega = Filter_Frequency_Get_Out(&Motor_2006_Speed_Filter[2]);
    DM_Motor_C610_Instances[2].Out = PID_Calculate(&Motor_2006_SPEED_PID[2], filtered_omega, DM_Motor_C610_Instances[2].Target_Omega, 0.001f);

    Filter_Frequency_Set_Now(&Motor_2006_Speed_Filter[3], DM_Motor_C610_Instances[3].Rx_Data.Now_Omega);
    Filter_Frequency_TIM_Calculate_PeriodElapsedCallback(&Motor_2006_Speed_Filter[3]);
    filtered_omega = Filter_Frequency_Get_Out(&Motor_2006_Speed_Filter[3]);
    DM_Motor_C610_Instances[3].Out = PID_Calculate(&Motor_2006_SPEED_PID[3], filtered_omega, DM_Motor_C610_Instances[3].Target_Omega, 0.001f);
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
            // Sin_Target = ALG_Sin_Generate(&Sin_Target, 2.0f, 25.0f, 1000.0f);
            // ALG_Value_Toggle_Periodic(&Sin_Target, 1.0f, -1.0f, 2.0f, 1000.0f);
            // DM_Motor_C610_Instances[2].Target_Omega = Sin_Target;
            Motor_DJI_CalPID();
            Motor_DJI_Output();
            // TransData_Send_Two_Float_Frame(&huart1, DM_Motor_C610_Instances[2].Out, DM_Motor_C610_Instances[2].Rx_Data.Now_Omega, 2);
        xTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

