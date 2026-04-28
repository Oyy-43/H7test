/**
 * @file withPC.c
 * @author Oyyp
 * @brief 与上位机的通讯
 * @version 0.1
 * @date 2026-04-17 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "withPC.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Computer_Frame_S PC_frame;
static uint16_t pc_rx_timeout_ms = 0;

/* Private function declarations ---------------------------------------------*/
uint8_t PC_rx_msg_check(uint8_t *Buffer, uint16_t Length)
{
    if (Length != PC_FRAME_LEN)
    {
        return 0;
    }
    if (Buffer[0] != FRAME_HEADER || Buffer[Length - 1] != FRAME_TAIL)
    {
        return 0;
    }
    return 1;
}


void PC_rx_idle_callback(uint8_t *Buffer, uint16_t Length)
{
    if(!PC_rx_msg_check(Buffer, Length))
    {
        return;
    }
    memcpy(&PC_frame, Buffer, sizeof(Computer_Frame_S));
    pc_rx_timeout_ms = 0;
}

void PC_rx_timeout_1ms_process(void)
{
    if (pc_rx_timeout_ms < PC_RX_TIMEOUT_MS)
    {
        pc_rx_timeout_ms++;
    }
    else
    {
        memset(&PC_frame, 0, sizeof(Computer_Frame_S));
    }
}

/* Function prototypes -------------------------------------------------------*/

