#ifndef __DRV_MOTOR_DJI_H
#define __DRV_MOTOR_DJI_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "alg_basic.h"
#include "drv_can.h"
#include "robot_def.h"

/* Exported macros -----------------------------------------------------------*/
// 电机实例表长度
#define DJI_Motor_C610_Num 2

#define C610_ENCODER_NUM_PER_ROUND 8192
#define C610_CURRENT_TO_TORQUE     0.005      //0.18f / 36.0f
#define C610_CURRENT_TO_OUT        1000.0f    //10000.0f / 10.0f
#define C610_OUT_MAX               10000.0f
#define C610_Gearbox_Rate          36.0f

#define DJI_Motor_C620_Num 4

#define C620_ENCODER_NUM_PER_ROUND 8192
#define C620_CURRENT_TO_TORQUE     0.5208      //0.3f / 19.2f
#define C620_CURRENT_TO_OUT        819.2f    //16384.0f / 20.0f
#define C620_OUT_MAX               16384.0f
#define C620_Gearbox_Rate          19.2f



/* Exported types ------------------------------------------------------------*/
/**
 * @brief 大疆电机类型枚举
 * 
 */
typedef enum Enum_Motor_DJI_Type
{
    Motor_DJI_Type_C610 = 0,
    Motor_DJI_Type_C620,
}Enum_Motor_DJI_Type;



/**
 * @brief 大疆电机状态
 *
 */
typedef enum Enum_Motor_DJI_Status
{
    Motor_DJI_Status_DISABLE = 0,
    Motor_DJI_Status_ENABLE,
}Enum_Motor_DJI_Status;

/**
 * @brief 大疆电机驱动版本, 分配CAN_ID时使用, 一般都是default, 6020电机使用
 *
 */
typedef enum Enum_Motor_DJI_GM6020_Driver_Version
{
    Motor_DJI_GM6020_Driver_Version_DEFAULT = 0,
    Motor_DJI_GM6020_Driver_Version_6020,
}Enum_Motor_DJI_GM6020_Driver_Version;


/**
 * @brief 大疆电机的ID枚举类型, 一拖四模式用
 *
 */
typedef enum Enum_Motor_DJI_ID
{
    Motor_DJI_ID_0x201 = 1,
    Motor_DJI_ID_0x202,
    Motor_DJI_ID_0x203,
    Motor_DJI_ID_0x204,
    Motor_DJI_ID_0x205,
    Motor_DJI_ID_0x206,
    Motor_DJI_ID_0x207,
    Motor_DJI_ID_0x208,
    Motor_DJI_ID_0x209,
    Motor_DJI_ID_0x20A,
    Motor_DJI_ID_0x20B,
}Enum_Motor_DJI_ID;

/**
 * @brief 大疆电机源数据
 *
 */
typedef struct __attribute__((packed)) Struct_Motor_DJI_CAN_Rx_Data_Row
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
} Struct_Motor_DJI_CAN_Rx_Data_Row;

/**
 * @brief 大疆电机经过处理的数据
 *
 */
typedef struct Struct_Motor_DJI_Rx_Data
{
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Temperature;
    float Now_Power;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
} Struct_Motor_DJI_Rx_Data;

typedef struct DJI_Motor_Instance
{
    // 电机类型
    Enum_Motor_DJI_Type Motor_Type;
    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 电机状态
    Enum_Motor_DJI_Status Motor_DJI_Status;
    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;
    // 滤波后速度
    float Filtered_Omega;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 编码器偏移
    int32_t Encoder_Offset;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float Nearest_Angle;
    // 当前时刻的电机接收flag
    uint32_t Flag;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag;
    // 输出量
    float Out;
    // 目标的角度, rad
    float Target_Angle;
    // 目标的速度, rad/s
    float Target_Omega;
    // 目标的扭矩, Nm
    float Target_Torque;
    // 前馈的速度, rad/s
    float Feedforward_Omega;
    // 前馈的扭矩, Nm
    float Feedforward_Torque;
} DJI_Motor_Instance;

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern DJI_Motor_Instance DJI_Motor_Instances[DJI_Motor_C610_Num+DJI_Motor_C620_Num];
/* Exported function declarations --------------------------------------------*/
void Motor_DJI_TIM_100ms_Alive_PeriodElapsedCallback();
void Motor_DJI_Init_All();
void Motor_DJI_SetOutput();
void Motor_DJI_CAN1_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);
void Motor_DJI_CAN2_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer);

#endif /* __DRV_MOTOR_DJI_H */