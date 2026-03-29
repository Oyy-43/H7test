#ifndef __DRV_CAN_H
#define __DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);

/**
 * @brief CAN通信处理结构体
 *
 */
typedef struct Struct_CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;

    // 与接收相关的数据
    FDCAN_RxHeaderTypeDef Rx_Header;
    uint8_t Rx_Buffer[64];

    // 接收时间戳
    uint64_t Rx_Timestamp;
} Struct_CAN_Manage_Object;

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern bool init_finished;

extern struct Struct_CAN_Manage_Object CAN1_Manage_Object;
extern struct Struct_CAN_Manage_Object CAN2_Manage_Object;
extern struct Struct_CAN_Manage_Object CAN3_Manage_Object;

/* Exported function declarations --------------------------------------------*/
void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function);

uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length);

void TIM_100us_CAN_PeriodElapsedCallback();

void TIM_1ms_CAN_PeriodElapsedCallback();

#endif // __DRV_CAN_H