#include "task_config_and_callback.h"

#include "sys_timestamp.h"
#include "drv_uart.h"
#include "cmsis_os2.h"
#include "bsp_buzzer.h"
#include "bsp_ws2812.h"
#include "drv_spi.h"
#include "bsp_key.h"

/* Private variables ---------------------------------------------------------*/
uint64_t us_time=0;
uint32_t ms_time=0;
uint16_t s_time=0;

// 全局初始化完成标志位
bool init_finished = false;

// LED灯
int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

/**
 * @brief 3600秒任务回调函数
 * @retval None
 */
void Task3600s_Callback()
{
    Timestamp_TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 1ms任务回调函数
 * @retval 后续可添加更多每1MS需要执行的任务
 */
void Task1ms_Callback()
{
    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red >= 18)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }
        if (green >= 18)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }
        if (blue >= 18)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        if (red_minus_flag)
        {
            red--;
        }
        else
        {
            red++;
        }
        if (green_minus_flag)
        {
            green--;
        }
        else
        {
            green++;
        }
        if (blue_minus_flag)
        {
            blue--;
        }
        else
        {
            blue++;
        }

        WS2812_Set_RGB(255, 0, 0);

        // 发送实例
        TIM_10ms_Write_PeriodElapsedCallback();
    }

    BSP_Key_TIM_1ms_Process_PeriodElapsedCallback();
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        // 处理按键状态
        BSP_Key_TIM_50ms_Read_PeriodElapsedCallback();
    }

}

/**
 * @brief 任务初始化函数
 * @retval None
 */
void Task_Init()
{
   //定时器时间戳初始化
    Timestamp_Init(&htim5);

   //WS2812 spi初始化
    SPI_Init(&hspi6, NULL);

   //定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim7);

   //蜂鸣器初始化
    Buzzer_Init(4000,0.0f);

   //初始化WS2812灯珠
    WS2812_Init(0, 0, 0);

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
