/**
 * @file tele_task.c
 * @author ZomFly
 * @brief edgeTX遥控器数据处理任务
 * @version 0.1
 * @date 2026-04-05 0.1 init
 *
 * @copyright Copyright
 *
 */

 /* Includes ------------------------------------------------------------------*/
#include "tele_task.h"
#include "crsf_utils.h"
/* global channels defined in crsf.c */
extern crsf_channels_t rc_channels;
 
 
 /* Private macros ------------------------------------------------------------*/
 /* Private types -------------------------------------------------------------*/
 CH_tatus_S ch9_status;
 /* Private variables ---------------------------------------------------------*/
 float test;
 /* Private function declarations ---------------------------------------------*/
float get_vbat_voltage(void)
{
    uint16_t adcx = 0;
    // 乘数算子
    // 3.3V / 65535
    //(10K Ω + 100K Ω)  / 10K Ω = 11
    const float multiplier = ((3.3f/65535) * 11.0f);
    float voltage;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0.0f;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0.0f;
    }

    adcx = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    voltage =  (float)adcx * multiplier;

    return voltage;
}

void Remote_Status_Update(CH_tatus_S *ch_status, uint8_t ch_index)
{
    int16_t val = rc_channels.ch[ch_index];

    if (val < 0)
    {
        ch_status->Now_GPIO_State = BSP_Channel_Minus;
    }
    else /* val > 0 */
    {
        ch_status->Now_GPIO_State = BSP_Channel_Plus;
    }
    if (ch_status->Pre_GPIO_State == BSP_Channel_Minus)
    {
        if (ch_status->Now_GPIO_State == BSP_Channel_Minus)
        {
            ch_status->Key_Status = CH_Status_FREE;
        }
        else
        {
            ch_status->Key_Status = CH_Status_TRIG_FREE_PRESSED;
        }
    }
    else
    {
        if (ch_status->Now_GPIO_State == BSP_Channel_Minus)
        {
            ch_status->Key_Status = CH_Status_TRIG_PRESSED_FREE;
        }
        else
        {
            ch_status->Key_Status = CH_Status_PRESSED;
        }
    }

    ch_status->Pre_GPIO_State = ch_status->Now_GPIO_State;
}


void tele_task(void *argument)
{
    float voltage = 0.0f;
    UART_Init(&CRSF_UART, crsf_rx_idle_callback);
    while (1)
    {
        voltage = get_vbat_voltage();
        // crsf_send_GPS(28.0805804f, -80.1234567f, 0, 88.8f, 90.9f, 8);
        crsf_send_BatterySensor(voltage, 1145, ((voltage + .00005f) * 100000.f), 99);
        osDelay(500);
    }
}
 
 /* Function prototypes -------------------------------------------------------*/
 
 