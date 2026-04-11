#ifndef __DRV_MOTOR_DM_H
#define __DRV_MOTOR_DM_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "drv_can.h"

/* Exported macros -----------------------------------------------------------*/
    // 电机实例表长度
    #define DM_Motor_Normal_Num 2
    #define DM_Motor_1_To_4_Num 6

    //每个电机的控制范围, 传统模式有效
    #define DM_3519_PMAX 0.0f
    #define DM_3519_VMAX 0.0f
    #define DM_3519_TMAX 0.0f
    #define DM_3519_Current_MAX 0.0f
    #define DM_8009P_PMAX 6.28f
    #define DM_8009P_VMAX 10.0f
    #define DM_8009P_TMAX 40.0f
    #define DM_8009P_Current_MAX 0.0f

    //一些一拖四模式的常数定义
    // 一圈编码器刻度
    #define ENCODER_NUM_PER_ROUND 8192

    // 扭矩电流常数, 由于电机手册没给, 则以额定扭矩除以额定电流计算
    #define CURRENT_TO_TORQUE (1.2f / 10.0f)
    // 扭矩电流到输出刻度的转化系数
    #define CURRENT_TO_OUT (16384.0f / 10.261194f)
    // 最大输出刻度
    #define OUT_MAX (16384.0f)

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 达妙电机状态
 *
 */
typedef enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
}Enum_Motor_DM_Status;

/**
 * @brief 达妙电机的ID枚举类型, 一拖四模式用
 *
 */
typedef enum Enum_Motor_DM_Motor_ID_1_To_4
{
    Motor_DM_ID_0x201 = 1,
    Motor_DM_ID_0x202,
    Motor_DM_ID_0x203,
    Motor_DM_ID_0x204,
    Motor_DM_ID_0x205,
    Motor_DM_ID_0x206,
    Motor_DM_ID_0x207,
    Motor_DM_ID_0x208,
}Enum_Motor_DM_Motor_ID_1_To_4;

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
typedef enum Enum_Motor_DM_Control_Status_Normal
{
    Motor_DM_Control_Status_DISABLE = 0x0,
    Motor_DM_Control_Status_ENABLE,
    Motor_DM_Control_Status_OVERVOLTAGE = 0x8,
    Motor_DM_Control_Status_UNDERVOLTAGE,
    Motor_DM_Control_Status_OVERCURRENT,
    Motor_DM_Control_Status_MOS_OVERTEMPERATURE,
    Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,
    Motor_DM_Control_Status_LOSE_CONNECTION,
    Motor_DM_Control_Status_MOS_OVERLOAD,
}Enum_Motor_DM_Control_Status_Normal;

/**
 * @brief 达妙电机控制方式
 *
 */
typedef enum Enum_Motor_DM_Control_Method
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_Control_Method_NORMAL_OMEGA,
    Motor_DM_Control_Method_NORMAL_EMIT,
    Motor_DM_Control_Method_1_TO_4_CURRENT,
    Motor_DM_Control_Method_1_TO_4_OMEGA,
    Motor_DM_Control_Method_1_TO_4_ANGLE,
}Enum_Motor_DM_Control_Method;

/**
 * @brief 达妙电机传统模式源数据
 *
 */
typedef struct __attribute__((packed)) Struct_Motor_DM_CAN_Rx_Data_Normal
{
    uint8_t CAN_ID : 4;
    uint8_t Control_Status_Enum : 4;
    uint16_t Angle_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} Struct_Motor_DM_CAN_Rx_Data_Normal;


/**
 * @brief 达妙电机经过处理的数据, 传统模式有效
 *
 */
typedef struct Struct_Motor_DM_Rx_Data_Normal
{
    Enum_Motor_DM_Control_Status_Normal Control_Status;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
} Struct_Motor_DM_Rx_Data_Normal;


/**
 * @brief 达妙电机一拖四模式源数据
 *
 */
typedef struct __attribute__((packed)) Struct_Motor_DM_CAN_Rx_Data_1_To_4
{
    uint16_t Encoder_Reverse;
    // 角速度100倍
    int16_t Omega_Reverse;
    // 电流值, mA
    int16_t Current_Reverse;
    uint8_t Rotor_Temperature;
    uint8_t Error_code;
} Struct_Motor_DM_CAN_Rx_Data_1_To_4;


/**
 * @brief 达妙电机经过处理的数据, 一拖四模式有效
 *
 */
typedef struct Struct_Motor_DM_Rx_Data_1_To_4
{
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    uint8_t Error_code;
    float Now_Rotor_Temperature;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
} Struct_Motor_DM_Rx_Data_1_To_4;


/**
 * @brief 达妙电机常规源数据, MIT控制报文
 *
 */
typedef struct __attribute__((packed))Struct_Motor_DM_CAN_Tx_Data_Normal_MIT
{
    uint16_t Control_Angle_Reverse;
    uint8_t Control_Omega_11_4;
    uint8_t Control_Omega_3_0_K_P_11_8;
    uint8_t K_P_7_0;
    uint8_t K_D_11_4;
    uint8_t K_D_3_0_Control_Torque_11_8;
    uint8_t Control_Torque_7_0;
}Struct_Motor_DM_CAN_Tx_Data_Normal_MIT;

/**
 * @brief 达妙电机常规源数据, 位置速度控制报文
 *
 */
typedef struct __attribute__((packed)) Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega
{
    float Control_Angle;
    float Control_Omega;
} Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega;

/**
 * @brief 达妙电机常规源数据, 速度控制报文
 *
 */
typedef struct __attribute__((packed)) Struct_Motor_DM_CAN_Tx_Data_Normal_Omega
{
    float Control_Omega;
} Struct_Motor_DM_CAN_Tx_Data_Normal_Omega;

/**
 * @brief 达妙电机常规源数据, EMIT控制报文
 *
 */
typedef struct __attribute__((packed)) Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT
{
    float Control_Angle;
    // 限定速度用, rad/s的100倍
    uint16_t Control_Omega;
    // 限定电流用, 电流最大值的10000倍
    uint16_t Control_Current;
} Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT;

/**
 * @brief 达妙电机传统模式下的电机实例结构体
 * 
 */
typedef struct DM_Motor_Instance
{
    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t CAN_Rx_ID;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t CAN_Tx_ID;
    // 电机状态
    Enum_Motor_DM_Status Motor_DM_Status;
    // 电机对外接口信息
    Struct_Motor_DM_Rx_Data_Normal Rx_Data;
    // 电机控制方式
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method;
    // 当前时刻的电机接收flag
    uint32_t Flag;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag;
    // 发送缓冲区
    uint8_t Tx_Data[8];
    // 角度, rad, 目标角度
    float Control_Angle;
    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
    float Control_Omega;
    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
    float Control_Torque;
    // 电流, A, EMIT模式目标电流
    float Control_Current;
    //PMAX,MIT模式有效，位置控制最大值，单位为rad
    float PMAX;
    //VMAX,MIT模式有效，速度控制最大值，单位为rad/s
    float VMAX;
    //TMAX,MIT模式有效，转矩控制最大值，单位为N.m
    float TMAX;
    //Current_MAX, EMIT模式有效, 电流控制最大值, 单位为A
    float Current_MAX;
    // K_P, 0~500, MIT模式有效
    float K_P;
    // K_D, 0~5, MIT模式有效
    float K_D;
}DM_Motor_Instance;

/**
 * @brief 达妙电机一拖四模式下的电机实例结构体
 * 
 */
typedef struct DM_Motor_1to4_Instance
{
    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    Enum_Motor_DM_Motor_ID_1_To_4 CAN_Rx_ID;
    // 电机状态
    Enum_Motor_DM_Status Motor_DM_Status;
    // 电机对外接口信息
    Struct_Motor_DM_Rx_Data_1_To_4 Rx_Data;
    // 电机控制方式
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method;
    // 当前时刻的电机接收flag
    uint32_t Flag;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 编码器偏移
    int32_t Encoder_Offset;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float Nearest_Angle;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;
    // 输出量
    float Out;
    // 目标的角度, 单位度
    float Target_Angle;
    // 目标的速度, rad/s
    float Target_Omega;
    // 目标的扭矩, Nm
    float Target_Torque;
    // 前馈的速度, rad/s
    float Feedforward_Omega;
    // 前馈的扭矩, Nm
    float Feedforward_Torque;
}DM_Motor_1to4_Instance;


/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern DM_Motor_Instance DM_Motor_Instances[DM_Motor_Normal_Num];
extern DM_Motor_1to4_Instance DM_Motor_1to4_Instances[DM_Motor_1_To_4_Num];

/* Exported function declarations --------------------------------------------*/
uint8_t *allocate_tx_data(const FDCAN_HandleTypeDef *hcan, Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID_1_To_4);

void Motor_DM_Init(DM_Motor_Instance *motor_instance, const FDCAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
    Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __PMax, float __VMax, float __TMax, float __Current_Max);

void Motor_DM_Init_All(void);

void Motor_DM_1_To_4_Init(DM_Motor_1to4_Instance *motor_instance, const FDCAN_HandleTypeDef *hcan, int32_t __Encoder_Offset,
    Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID, Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __Nearest_Angle, float __Gearbox_Rate);


void Motor_DM_Normal_Data_Process(DM_Motor_Instance *motor_instance);

void Motor_DM_1_To_4_Data_Process(DM_Motor_1to4_Instance *motor_instance);

void Motor_DM_Normal_Output(DM_Motor_Instance *motor_instance);

void Motor_DM_CAN1_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);
void Motor_DM_CAN2_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);

#endif /* __DRV_MOTOR_DM_H */
