#ifndef _WITH_PC_H_
#define _WITH_PC_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Lift.h"
#include "drv_usb.h"

/* Exported macros -----------------------------------------------------------*/
#define FRAME_HEADER 0xAA
#define FRAME_TAIL 0x0D
#define PC_FRAME_LEN 34U
#define PC_RX_TIMEOUT_MS 500U

/* Exported types ------------------------------------------------------------*/
typedef struct __attribute__((packed))
{
    uint8_t header;                  // 帧头
    uint8_t Calibration_Flag;        // 1字节，校准标志位，0表示未请求校准，1表示请求校准
    float cmd_vx;                    // 4字节，X向速度指令
    float cmd_vy;                    // 4字节，Y向速度指令
    float cmd_yaw;                   // 4字节，航向角指令，单位为度，范围 [0, 360)
    uint8_t cmd_lift;                // 1字节，升降指令
    float motor0_height;             // 4字节，上层机构电机0的高度
    float motor1_x_length;           // 4字节，上层机构电机1的伸出长度
    float motor2_target_angle;       // 4字节，上层机构电机2的目标角度
    float motor3_target_angle;       // 4字节，上层机构电机3的目标角度
    uint8_t cmd_servocontrol;        // 舵机指令
    uint8_t suction_cup_control;     // 吸盘控制指令
    uint8_t tail; // 帧尾
}Computer_Frame_S;

typedef struct __attribute__((packed))
{
    uint8_t header;                       //帧头
    uint8_t Calibration_flag;             // 1字节，校准标志位，发0x00则为未校准，发0x01则为已校准
    uint8_t Lift_flag;                    // 1字节，升降状态标志位，0表示未升降，1表示正在上台阶，2表示正在下台阶
    float Eul_YAW;                        // 4字节，当前的yaw角
    uint8_t tail;                         // 帧尾
}ComputerTransmit_Frame_S;
/* Exported constants --------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/
extern Computer_Frame_S PC_frame;

/* Exported function declarations --------------------------------------------*/
void PC_rx_idle_callback(uint8_t *Buffer, uint16_t Length);
void PC_rx_timeout_1ms_process(void);
void PC_transmitData(void);
uint8_t PC_Is_Online(void);




#endif // _WITH_PC_H_
