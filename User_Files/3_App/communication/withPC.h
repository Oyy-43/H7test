#ifndef _WITH_PC_H_
#define _WITH_PC_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/
#define FRAME_HEADER 0xAA
#define FRAME_TAIL 0x0D
#define PC_FRAME_LEN 14U
#define PC_RX_TIMEOUT_MS 500U

/* Exported types ------------------------------------------------------------*/
typedef struct __attribute__((packed))
{
    uint8_t header; // 帧头
    float cmd_vx; // 4字节，X向速度指令
    float cmd_vy; // 4字节，Y向速度指令
    float cmd_vz; // 4字节，绕Z轴旋转速度指
    uint8_t tail; // 帧尾
}Computer_Frame_S;
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern Computer_Frame_S PC_frame;

/* Exported function declarations --------------------------------------------*/
void PC_rx_idle_callback(uint8_t *Buffer, uint16_t Length);
void PC_rx_timeout_1ms_process(void);




#endif // _WITH_PC_H_