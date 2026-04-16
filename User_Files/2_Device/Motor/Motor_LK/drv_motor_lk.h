#ifndef __DRV_MOTOR_LK_H
#define __DRV_MOTOR_LK_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "robot_def.h"
#include "drv_can.h"
#include "string.h"
#include "cmsis_os2.h"
#include "crsf.h"

/* Exported macros -----------------------------------------------------------*/
#define MF_Current_Mapping 0.008056640625   //  33/4096
#define MG_Current_Mapping 0.01611328125    //  66/4096
#define LK_Motor_Num 2
#define DEVICE_STD_ID 0x140

/* Exported types ------------------------------------------------------------*/
/**
 * @brief lk电机的类型
 * 
 */
typedef enum LKMotor_type
{
    MS = 0,
    MF = 1,
    MG = 2
} LKMotor_type;

// 接收的数据
typedef struct LKRxDataRow
{
    int8_t Motor_Temperature;           //单位1°C/LSB
    float voltage;                    //单位0.01V/LSB
    float current;                    //单位0.01A/LSB
    uint8_t Motor_State;                //电机状态，各个位代表不同的电机状态
    uint8_t Error_State;                //错误状态，各个位代表不同的电机错误状态
    float Torque_Current;             //电机iq分辨率：MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    int16_t Speed;                      //单位： 1dps/LSB
    uint16_t Encoder;                   //编码器计数 范围：0~65535
    uint16_t EncoderRaw;                //原始编码器计数 范围：0~65535
    uint16_t EncoderOffset;             //编码器0偏移 范围：0~65535  该点作为电机角度的零点
    float  iA;                        //电机相电流A分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    float  iB;                        //电机相电流B分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    float  iC;                        //电机相电流C分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint8_t  brake_state;               //电机刹车状态,如果为0x00则刹车启动，为0x01则刹车释放
    float  Motor_multiAngle;          //电机多圈角度值，正值表示顺时针累计角度，负值表示逆时针累计角度
    float  CircleAngle;               //电机单圈角度值，以编码器零点为起始点，顺时针增加，再次到达零点时数值回 0，单位 0.01°/LSB，数值范围 0~36000*减速比-1。
}LKRxDataRow;

typedef struct LKMotor_Instance
{
    //翎控电机的类型
    LKMotor_type LK_Type;                      //电机类型
    // 电机ID
    uint8_t ID;
    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t CAN_Rx_ID;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t CAN_Tx_ID;
    // 电机状态
    uint8_t Motor_State;
    // 错误状态
    uint8_t Error_State;
    // 电机对外接口信息
    LKRxDataRow Rx_Data;
    // 减速比
    float Reduction_Ratio;
    // 目标角度
    float Target_Angle;
    // 目标速度
    float Target_Omega;
    // 目标电流
    float Target_Current;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float Nearest_Angle;

}LKMotor_Instance;

typedef struct __attribute__((packed)) LKRXData_Frame1
{
    uint8_t cmd_id;
    uint8_t Motor_Temperature;           //单位1°C/LSB
    uint16_t Voltage;                    //单位0.01V/LSB
    uint16_t Current;                    //单位MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint8_t Motor_State;                //电机状态，各个位代表不同的电机状态
    uint8_t Error_State;                //错误状态，各个位代表不同的电机错误状态
}LKRXData_Frame1;

typedef struct __attribute__((packed)) LKRXData_Frame2
{
    uint8_t cmd_id;
    uint8_t Motor_Temperature;           //单位1°C/LSB
    uint16_t Current;                    //单位MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint16_t Speed;                      //单位： 1dps/LSB
    uint16_t Encoder;                   //编码器计数 范围：0~655
}LKRXData_Frame2;

typedef struct __attribute__((packed)) LKRXData_Frame3
{
    uint8_t cmd_id;
    uint8_t Motor_Temperature;           //单位1°C/LSB
    uint16_t  iA;                        //电机相电流A分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint16_t  iB;                        //电机相电流B分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
    uint16_t  iC;                        //电机相电流C分量，分辨率 MF:(33/4096A)/LSB MG:(66/4096A)/LSB
}LKRXData_Frame3;

typedef struct __attribute__((packed)) LKRXData_Breakstate
{
    uint8_t cmd_id;
    uint8_t brake_state;               //电机刹车状态,如果为0x00则刹车启动，为0x01则刹车释放
    uint8_t reserved[6];
}LKRXData_Breakstate;

typedef struct __attribute__((packed)) LKRXData_Encoder
{
    uint8_t cmd_id;
    uint8_t reserved;
    uint16_t Encoder;                   //编码器计数 范围：0~65535
    uint16_t EncoderRaw;                //原始编码器计数 范围：0~65535
    uint16_t EncoderOffset;             //编码器0偏移 范围：0~65535  该点作为电机角度的零点
}LKRXData_Encoder;

typedef struct __attribute__((packed)) LKRXData_MultiAngle
{
    uint8_t cmd_id;      // 0x92
    uint8_t angle_b[7];  // DATA[1]..DATA[7], little-endian
} LKRXData_MultiAngle;

typedef struct __attribute__((packed)) LKRXData_CircleAngle
{
    uint8_t cmd_id;      // 0x93
    uint8_t reserved[3];  // DATA[1]..DATA[7], little-endian
    uint32_t CircleAngle;  // DATA[4]..DATA[7], little-endian, 单圈角度值，以编码器零点为起始点，顺时针增加，再次到达零点时数值回0，单位0.01°/LSB，数值范围0~36000*减速比-1。
} LKRXData_CircleAngle;





/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern LKMotor_Instance LK_Motor_Instances[LK_Motor_Num];


/* Exported function declarations --------------------------------------------*/
void Motor_LK_Init_All(void);
void Motor_LK_LoopAngleControl1_AutoDirection(LKMotor_Instance *motor_instance,float angle,uint16_t maxSpeed);
void Motor_LK_Data_Process(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);
void Motor_LK_CAN3_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);
void Motor_LK_GetFeedback(LKMotor_Instance *motor_instance);
void Motor_LK_Stop(LKMotor_Instance *motor_instance);
void Motor_LK_TorqueControl(LKMotor_Instance *motor_instance,int16_t iqControl);

#endif /* __DRV_MOTOR_LK_H */