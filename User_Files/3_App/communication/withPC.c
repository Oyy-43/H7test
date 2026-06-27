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
#include <math.h>


/* Private macros ------------------------------------------------------------*/
#define Auto_VxMAX 2.5f
#define Auto_VyMAX 2.5f


/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Computer_Frame_S PC_frame;
ComputerTransmit_Frame_S PC_Transmit_Frame;
static uint16_t pc_rx_timeout_ms = 0;

static uint8_t PC_frame_value_check(const Computer_Frame_S *frame)    //有效值检查与限幅
{
    if (frame == NULL)
    {
        return 0;
    }

    if (!isfinite(frame->cmd_vx) || !isfinite(frame->cmd_vy) || !isfinite(frame->cmd_yaw))
    {
        return 0;
    }

    if ((fabsf(frame->cmd_vx) > Auto_VxMAX) || (fabsf(frame->cmd_vy) > Auto_VyMAX))
    {
        return 0;
    }

    if ((frame->cmd_yaw < 0.0f) || (frame->cmd_yaw >= 360.0f))
    {
        return 0;
    }

    return 1;
}

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
    Computer_Frame_S tmp_frame;

    if(!PC_rx_msg_check(Buffer, Length))
    {
        return;
    }

    memcpy(&tmp_frame, Buffer, sizeof(Computer_Frame_S));
    if (!PC_frame_value_check(&tmp_frame))
    {
        return;
    }

    PC_frame = tmp_frame;
    pc_rx_timeout_ms = 0;
}

void PC_transmitData(void)
{
    PC_Transmit_Frame.header = FRAME_HEADER;
    PC_Transmit_Frame.Calibration_flag = (Front_Calibrated && Back_Calibrated) ? 0x01 : 0x00;
    switch(LiftingState_t.state)
    {
        case No_Lifting:
            PC_Transmit_Frame.Lift_flag = 0x00;
            break;
        case LiftLevel200_Step1:
        case LiftLevel200_Step2:
        case LiftLevel200_Step3:
        case LiftLevel200_Step4:
        case LiftLevel200_Step5:
        case LiftLevel200_Step6:
        case LiftLevel200_Step7:
            PC_Transmit_Frame.Lift_flag = 0x01;
            break;
        case DownLevel200_Step1:
        case DownLevel200_Step2:
        case DownLevel200_Step3:
        case DownLevel200_Step4:
        case DownLevel200_Step5:
        case DownLevel200_Step6:
        case DownLevel200_Step7:
            PC_Transmit_Frame.Lift_flag = 0x02;
            break;
        default:
            PC_Transmit_Frame.Lift_flag = 0x00;
            break;
    }
    PC_Transmit_Frame.Eul_YAW = hipnuc_imu_data.eul[2];
    PC_Transmit_Frame.tail = FRAME_TAIL;
    USB_Transmit_Data((uint8_t *)&PC_Transmit_Frame, sizeof(ComputerTransmit_Frame_S));
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
        PC_frame.cmd_yaw = hipnuc_imu_data.eul[2];
    }
}

uint8_t PC_Is_Online(void)
{
    return (pc_rx_timeout_ms < PC_RX_TIMEOUT_MS);
}

/* Function prototypes -------------------------------------------------------*/

