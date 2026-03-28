#include "task_config_and_callback.h"

#include "sys_timestamp.h"
#include "drv_uart.h"
#include "cmsis_os2.h"
#include "bsp_buzzer.h"

uint64_t us_time=0;
uint32_t ms_time=0;
uint16_t s_time=0;

/**
 * @brief 3600秒任务回调函数
 * @retval None
 */
void Task3600s_Callback()
{
    Timestamp_TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 任务初始化函数
 * @retval None
 */
void Task_Init()
{
   //定时器时间戳初始化
    Timestamp_Init(&htim5);

   //蜂鸣器初始化
    Buzzer_Init(4000,0.0f);

   //定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim5);

   //标记初始化完成 
    init_finished = true;  
}

void Timestamp_fuc(void *argument)
{
    for(;;)
    {
       us_time = Timestamp_Get_Now_Microsecond();
       ms_time = Timestamp_Get_Now_Millisecond();
       s_time = Timestamp_Get_Now_Second();
       osDelay(1);
    }
}
