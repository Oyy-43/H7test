#ifndef __DRV_USB_H
#define __DRV_USB_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sys_timestamp.h"
#include "usbd_cdc_if.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/
// 缓冲区字节长度
#define USB_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/
/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*USB_Callback)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief USB通信处理结构体
 */
typedef struct Struct_USB_Manage_Object
{
    UART_HandleTypeDef *UART_Handler;
    USB_Callback Callback_Function;

    // 双缓冲适配的缓冲区以及当前激活的缓冲区
    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[USB_BUFFER_SIZE];
    uint8_t *Rx_Buffer_Active;
    uint8_t *Rx_Buffer_Ready;

    // 接收时间戳
    uint64_t Rx_Time_Stamp;
}Struct_USB_Manage_Object;


/* Exported constants --------------------------------------------------------*/
extern bool init_finished;

extern Struct_USB_Manage_Object USB0_Manage_Object;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void USB_Init(USB_Callback Callback_Function);

uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length);

void USB_ReceiveCallback(uint16_t Size);

#endif /* __DRV_USB_H */