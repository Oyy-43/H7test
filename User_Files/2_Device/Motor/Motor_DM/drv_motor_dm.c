/**
 * @file drv_motor_dm.c
 * @author Oyyp
 * @brief 借鉴麻神的，麻神写的有点太复杂了qwq
 * @version 0.1
 * @date 2026-03-30 0.1 初版
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_basic.h"
#include "drv_motor_dm.h"
#include "drv_can.h"

/* Private macros ------------------------------------------------------------*/


/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// 清除电机错误信息, 传统模式有效
uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
// 使能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
// 失能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
// 保存当前电机位置为零点, 传统模式有效
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
// 一拖四清除当前电机错误信息, 1拖4模式有效,data[0]和data[1]分别为CANID_L和CANID_H
uint8_t DM_Motor_1_To_4_CAN_Message_Clear_Error[8] = {0xff, 0xff, 0x55, 0x3c, 0xff, 0xff, 0xff, 0xff}; 
// 一拖四设置零点, 1拖4模式有效,data[0]和data[1]分别为CANID_L和CANID_H
uint8_t DM_Motor_1_To_4_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0x55, 0x50, 0xff, 0xff, 0xff, 0xff};


// 传统模式电机实例表
DM_Motor_Instance DM_Motor_Instances[DM_Motor_Normal_Num] = {0};
DM_Motor_1to4_Instance DM_Motor_1to4_Instances[DM_Motor_1_To_4_Num] = {0};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 分配CAN发送缓冲区, 一拖四模式有效
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __DJI_Motor_Driver_Version 达妙驱动版本, 电机ID 1~4 的标识符为0x200，电机ID 5~8 的标识符为0x1ff
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(const FDCAN_HandleTypeDef *hcan, Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID_1_To_4)
{
    uint8_t *tmp_tx_data_ptr = NULL;
    if (hcan == &hfdcan1)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case (Motor_DM_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DM_ID_0x205):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x206):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x207):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x208):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);

            break;
        }
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case (Motor_DM_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DM_ID_0x205):
        {
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x206):
        {
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x207):
        {
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x208):
        {
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[6]);

            break;
        }
        }
    }
    else if (hcan == &hfdcan3)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case (Motor_DM_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DM_ID_0x205):
        {
            tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x206):
        {
            tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x207):
        {
            tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x208):
        {
            tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[6]);

            break;
        }
        }
    }
    else
    {
        tmp_tx_data_ptr = NULL;
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __Motor_DM_Control_Method 电机控制方式
 * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 * @param __Current_Max 最大电流, 与上位机串口中上电打印电流保持一致, EMIT模式需要
 */
void Motor_DM_Init(DM_Motor_Instance *motor_instance, const FDCAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
    Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __PMax, float __VMax, float __TMax, float __Current_Max)
{

    if (hcan->Instance == FDCAN1)
    {
        motor_instance->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        motor_instance->CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        motor_instance->CAN_Manage_Object = &CAN3_Manage_Object;
    }
    motor_instance->CAN_Rx_ID = __CAN_Rx_ID;
    switch (__Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_NORMAL_MIT):
    {
        motor_instance->CAN_Tx_ID = __CAN_Tx_ID;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
    {
        motor_instance->CAN_Tx_ID = __CAN_Tx_ID + 0x100;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_OMEGA):
    {
        motor_instance->CAN_Tx_ID = __CAN_Tx_ID + 0x200;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_EMIT):
    {
        motor_instance->CAN_Tx_ID = __CAN_Tx_ID + 0x300;
        break;
    }
    case (Motor_DM_Control_Method_1_TO_4_CURRENT):
    case (Motor_DM_Control_Method_1_TO_4_OMEGA):
    case (Motor_DM_Control_Method_1_TO_4_ANGLE):
    default:
    {
        motor_instance->CAN_Tx_ID = __CAN_Tx_ID;
        break;
    }
    }
    motor_instance->Motor_DM_Control_Method = __Motor_DM_Control_Method;
    motor_instance->PMAX = __PMax;
    motor_instance->VMAX = __VMax;
    motor_instance->TMAX = __TMax;
    motor_instance->Current_MAX = __Current_Max;

}

/**
 * @brief 初始化所有达妙电机实例，并进行使能
 * 
 */
void Motor_DM_Init_All(void)
{
    Motor_DM_Init(&DM_Motor_Instances[0], &hfdcan2, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, DM_8009P_PMAX, DM_8009P_VMAX, DM_8009P_TMAX, DM_8009P_Current_MAX);
    Motor_DM_Init(&DM_Motor_Instances[1], &hfdcan2, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT, DM_8009P_PMAX, DM_8009P_VMAX, DM_8009P_TMAX, DM_8009P_Current_MAX);
    Motor_DM_1_To_4_Init(&DM_Motor_1to4_Instances[0], &hfdcan1, 0, Motor_DM_ID_0x201, Motor_DM_Control_Method_1_TO_4_OMEGA, 0.0f, 19.0f);
    Motor_DM_1_To_4_Init(&DM_Motor_1to4_Instances[1], &hfdcan1, 0, Motor_DM_ID_0x202, Motor_DM_Control_Method_1_TO_4_OMEGA, 0.0f, 19.0f);
    Motor_DM_1_To_4_Init(&DM_Motor_1to4_Instances[2], &hfdcan1, 0, Motor_DM_ID_0x203, Motor_DM_Control_Method_1_TO_4_OMEGA, 0.0f, 19.0f);
    Motor_DM_1_To_4_Init(&DM_Motor_1to4_Instances[3], &hfdcan1, 0, Motor_DM_ID_0x204, Motor_DM_Control_Method_1_TO_4_OMEGA, 0.0f, 19.0f);
    Motor_DM_1_To_4_Init(&DM_Motor_1to4_Instances[4], &hfdcan2, 0, Motor_DM_ID_0x205, Motor_DM_Control_Method_1_TO_4_OMEGA, 0.0f, 19.0f);
    Motor_DM_1_To_4_Init(&DM_Motor_1to4_Instances[5], &hfdcan2, 0, Motor_DM_ID_0x206, Motor_DM_Control_Method_1_TO_4_OMEGA, 0.0f, 19.0f);
}

/**
 * @brief 电机初始化（1拖4模式）
 * 
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DM_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Nearest_Angle 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
 * @param __Gearbox_Rate 减速比, 默认19.0f
 */
void Motor_DM_1_To_4_Init(DM_Motor_1to4_Instance *motor_instance, const FDCAN_HandleTypeDef *hcan, int32_t __Encoder_Offset,
    Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID, Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __Nearest_Angle, float __Gearbox_Rate)
{
    if (hcan->Instance == FDCAN1)
    {
        motor_instance->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        motor_instance->CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        motor_instance->CAN_Manage_Object = &CAN3_Manage_Object;
    }
    motor_instance->CAN_Rx_ID = __CAN_Rx_ID;
    motor_instance->Motor_DM_Control_Method = __Motor_DM_Control_Method;
    motor_instance->Encoder_Offset = __Encoder_Offset;
    motor_instance->Nearest_Angle = __Nearest_Angle;
    motor_instance->Gearbox_Rate = __Gearbox_Rate;
    motor_instance->Tx_Data = allocate_tx_data(hcan, __CAN_Rx_ID);
}

/**
 * @brief 传统数据处理过程
 *
 */
void Motor_DM_Normal_Data_Process(DM_Motor_Instance *motor_instance)
{
    // 数据处理过程
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    Struct_Motor_DM_CAN_Rx_Data_Normal *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_Normal *)motor_instance->CAN_Manage_Object->Rx_Buffer;

    // 电机ID不匹配, 则不进行处理
    if (tmp_buffer->CAN_ID != (motor_instance->CAN_Tx_ID & 0x0f))
    {
        return;
    }

    // 处理大小端
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Angle_Reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);

    motor_instance->Rx_Data.Control_Status = (Enum_Motor_DM_Control_Status_Normal)(tmp_buffer->Control_Status_Enum);

    // 计算电机本身信息
    motor_instance->Rx_Data.Now_Angle = Basic_Math_Int_To_Float(tmp_encoder, 0x7fff, (1 << 16) - 1, 0,motor_instance->PMAX);
    motor_instance->Rx_Data.Now_Omega = Basic_Math_Int_To_Float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, motor_instance->VMAX);
    motor_instance->Rx_Data.Now_Torque = Basic_Math_Int_To_Float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, motor_instance->TMAX);
    motor_instance->Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + BASIC_MATH_CELSIUS_TO_KELVIN;
    motor_instance->Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + BASIC_MATH_CELSIUS_TO_KELVIN;
}


/**
 * @brief 1拖4模式下反馈报文数据处理过程
 *
 */
void Motor_DM_1_To_4_Data_Process(DM_Motor_1to4_Instance *motor_instance)
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DM_CAN_Rx_Data_1_To_4 *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_1_To_4 *)motor_instance->CAN_Manage_Object->Rx_Buffer;

    // 处理大小端
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - motor_instance->Rx_Data.Pre_Encoder;
    if (delta_encoder < -ENCODER_NUM_PER_ROUND / 2)
    {
        // 正方向转过了一圈
        motor_instance->Rx_Data.Total_Round++;
    }
    else if (delta_encoder > ENCODER_NUM_PER_ROUND / 2)
    {
        // 反方向转过了一圈
        motor_instance->Rx_Data.Total_Round--;
    }
    motor_instance->Rx_Data.Total_Encoder = motor_instance->Rx_Data.Total_Round * ENCODER_NUM_PER_ROUND + tmp_encoder + motor_instance->Encoder_Offset;

    // 计算电机本身信息
    motor_instance->Rx_Data.Now_Angle = (float) motor_instance->Rx_Data.Total_Encoder / (float) ENCODER_NUM_PER_ROUND * 2.0f * PI;
    motor_instance->Rx_Data.Now_Omega = tmp_omega / 100.0f * BASIC_MATH_RPM_TO_RADPS;
    motor_instance->Rx_Data.Now_Torque = tmp_current / 1000.0f * CURRENT_TO_TORQUE * motor_instance->Gearbox_Rate;
    motor_instance->Rx_Data.Error_code = tmp_buffer->Error_code;
    motor_instance->Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature ;

    // 存储预备信息0
    motor_instance->Rx_Data.Pre_Encoder = tmp_encoder;
}


/**
 * @brief 电机数据输出到CAN总线
 *
 */
void Motor_DM_Normal_Output(DM_Motor_Instance *motor_instance)
{
    // 电机控制
    switch (motor_instance->Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_NORMAL_MIT):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *)motor_instance->Tx_Data;

        uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

        tmp_angle = Basic_Math_Float_To_Int(motor_instance->Control_Angle, 0, motor_instance->PMAX, 0x7fff, (1 << 16) - 1);
        tmp_omega = Basic_Math_Float_To_Int(motor_instance->Control_Omega, 0, motor_instance->VMAX, 0x7ff, (1 << 12) - 1);
        tmp_torque = Basic_Math_Float_To_Int(motor_instance->Control_Torque, 0, motor_instance->TMAX, 0x7ff, (1 << 12) - 1);
        tmp_k_p = Basic_Math_Float_To_Int(motor_instance->K_P, 0, 500.0f, 0, (1 << 12) - 1);
        tmp_k_d = Basic_Math_Float_To_Int(motor_instance->K_D, 0, 5.0f, 0, (1 << 12) - 1);

        tmp_buffer->Control_Angle_Reverse = Basic_Math_Endian_Reverse_16(&tmp_angle, NULL);
        tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
        tmp_buffer->Control_Omega_3_0_K_P_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
        tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
        tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
        tmp_buffer->K_D_3_0_Control_Torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

        CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, motor_instance->Tx_Data, 8);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *) motor_instance->Tx_Data;

        tmp_buffer->Control_Angle = motor_instance->Control_Angle;
        tmp_buffer->Control_Omega = motor_instance->Control_Omega;

        CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, motor_instance->Tx_Data, 8);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_OMEGA):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *) motor_instance->Tx_Data;

        tmp_buffer->Control_Omega = motor_instance->Control_Omega;

        CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, motor_instance->Tx_Data, 4);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_EMIT):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *) motor_instance->Tx_Data;

        tmp_buffer->Control_Angle = motor_instance->Control_Angle;
        tmp_buffer->Control_Omega = (uint16_t)(motor_instance->Control_Omega * 100.0f);
        tmp_buffer->Control_Current = (uint16_t)(motor_instance->Control_Current / motor_instance->Current_MAX * 10000.0f);

        CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, motor_instance->Tx_Data, 8);

        break;
    }
    case (Motor_DM_Control_Method_1_TO_4_CURRENT):
    case (Motor_DM_Control_Method_1_TO_4_OMEGA):
    case (Motor_DM_Control_Method_1_TO_4_ANGLE):
    default:
    {
        break;
    }
    }
}


/**
 * @brief CAN通信接收回调函数
 *
 */
void Motor_DM_CAN1_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{

    if ((Header == NULL) || (Buffer == NULL))
    {
        return;
    }
    switch (Header->Identifier)
    {
        case (0x11):
        if(DM_Motor_Instances[0].CAN_Manage_Object != NULL)
        {
            DM_Motor_Instances[0].Flag +=1;
            Motor_DM_Normal_Data_Process(&DM_Motor_Instances[0]);
        }
        break;
        case (0x12):
        if(DM_Motor_Instances[1].CAN_Manage_Object != NULL)
        {
            DM_Motor_Instances[1].Flag +=1;
            Motor_DM_Normal_Data_Process(&DM_Motor_Instances[1]);
        }
        break;
        case (0x201):
        if(DM_Motor_1to4_Instances[0].CAN_Manage_Object != NULL)
        {
            DM_Motor_1to4_Instances[0].Flag +=1;
            Motor_DM_1_To_4_Data_Process(&DM_Motor_1to4_Instances[0]);
        }
        break;
        case (0x202):
        if(DM_Motor_1to4_Instances[1].CAN_Manage_Object != NULL)
        {
            DM_Motor_1to4_Instances[1].Flag +=1;
            Motor_DM_1_To_4_Data_Process(&DM_Motor_1to4_Instances[1]);
        }
        break;
        case (0x203):
        if(DM_Motor_1to4_Instances[2].CAN_Manage_Object != NULL)
        {
            DM_Motor_1to4_Instances[2].Flag +=1;
            Motor_DM_1_To_4_Data_Process(&DM_Motor_1to4_Instances[2]);
        }
        break;
        case (0x204):
        if(DM_Motor_1to4_Instances[3].CAN_Manage_Object != NULL)
        {
            DM_Motor_1to4_Instances[3].Flag +=1;
            Motor_DM_1_To_4_Data_Process(&DM_Motor_1to4_Instances[3]);
        }
        break;       
        default:
        break;
    }
}

/**
 * @brief DMcan2回调函数编写
 * 
 */
void Motor_DM_CAN2_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    if ((Header == NULL) || (Buffer == NULL))
    {
        return;
    }
    switch (Header->Identifier)
    {
        case (0x205):
        if(DM_Motor_1to4_Instances[4].CAN_Manage_Object != NULL)
        {
            DM_Motor_1to4_Instances[4].Flag +=1;
            Motor_DM_1_To_4_Data_Process(&DM_Motor_1to4_Instances[4]);
        }
        break;
        case (0x206):
        if(DM_Motor_1to4_Instances[5].CAN_Manage_Object != NULL)
        {
            DM_Motor_1to4_Instances[5].Flag +=1;
            Motor_DM_1_To_4_Data_Process(&DM_Motor_1to4_Instances[5]);
        }
        break;        
        default:
        break;
    }
}

/**
 * @brief 发送清除错误信息
 *
 */
void Motor_DM_Normal_Send_Clear_Error(DM_Motor_Instance *motor_instance)
{
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, DM_Motor_CAN_Message_Clear_Error, 8);
}

/**
 * @brief 发送使能电机
 *
 */
void Motor_DM_Normal_Send_Enter(DM_Motor_Instance *motor_instance)
{
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, DM_Motor_CAN_Message_Enter, 8);
}

/**
 * @brief 发送失能电机
 *
 */
void Motor_DM_Normal_Send_Exit(DM_Motor_Instance *motor_instance)
{
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, DM_Motor_CAN_Message_Exit, 8);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void Motor_DM_Normal_Send_Save_Zero(DM_Motor_Instance *motor_instance)
{
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, DM_Motor_CAN_Message_Save_Zero, 8);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void Motor_DM_Normal_TIM_100ms_Alive_PeriodElapsedCallback(DM_Motor_Instance *motor_instance)
{
    // 判断该时间段内是否接收过电机数据
    if (motor_instance->Flag == motor_instance->Pre_Flag)
    {
        // 电机断开连接
        motor_instance->Motor_DM_Status = Motor_DM_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        motor_instance->Motor_DM_Status = Motor_DM_Status_ENABLE;
    }

    motor_instance->Pre_Flag = motor_instance->Flag;

    if (motor_instance->Motor_DM_Status == Motor_DM_Status_DISABLE)
    {
        Motor_DM_Normal_Send_Enter(motor_instance);
    }
}