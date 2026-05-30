/**
 * @file drv_motor_dji.c
 * @author Oyyp
 * @brief 移植于麻神的大疆电机驱动代码
 * @version 0.1
 * @date 2026-04-17 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_motor_dji.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
DJI_Motor_Instance DJI_Motor_Instances[DJI_Motor_C610_Num+DJI_Motor_C620_Num] = {0};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 分配CAN发送缓冲区, 大疆有效
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __DJI_Motor_Driver_Version 大疆驱动版本, 电机ID 1~4 的标识符为0x200，电机ID 5~8 的标识符为0x1ff
 * @return uint8_t* 缓冲区指针
 */
/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __DJI_Motor_Driver_Version 大疆驱动版本, 当且仅当当前被分配电机为6020, 且是电流驱动新版本时选2023, 否则都是default
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data_DJI(const FDCAN_HandleTypeDef *hcan,Enum_Motor_DJI_ID __CAN_ID,Enum_Motor_DJI_GM6020_Driver_Version __DJI_Motor_Driver_Version)
{
    uint8_t *tmp_tx_data_ptr = NULL;
    if (hcan == &hfdcan1)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DJI_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DJI_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DJI_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DJI_ID_0x205):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x206):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x207):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[4]);
            }

            break;
        }
        case (Motor_DJI_ID_0x208):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[6]);
            }

            break;
        }
        case (Motor_DJI_ID_0x209):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x20A):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x20B):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[4]);
            }

            break;
        }
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DJI_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DJI_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DJI_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DJI_ID_0x205):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x206):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x207):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[4]);
            }

            break;
        }
        case (Motor_DJI_ID_0x208):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[6]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[6]);
            }

            break;
        }
        case (Motor_DJI_ID_0x209):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x20A):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x20B):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[4]);
            }

            break;
        }
        }
    }
    else if (hcan == &hfdcan3)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DJI_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DJI_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DJI_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DJI_ID_0x205):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x206):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x207):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[4]);
            }
            break;
        }
        case (Motor_DJI_ID_0x208):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[6]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[6]);
            }
            break;
        }
        case (Motor_DJI_ID_0x209):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[0]);
            }
            break;
        }
        case (Motor_DJI_ID_0x20A):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[2]);
            }
            break;
        }
        case (Motor_DJI_ID_0x20B):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[4]);
            }
            break;
        }
        }
    }
    return (tmp_tx_data_ptr);
}

void Motor_DJI_Init(DJI_Motor_Instance *motor_instance, const FDCAN_HandleTypeDef *hcan, Enum_Motor_DJI_ID __CAN_Rx_ID,
    int32_t __Encoder_Offset,float __Nearest_Angle, float __Gearbox_Rate,Enum_Motor_DJI_Type __Motor_Type)
{
    if (hcan == &hfdcan1)
    {
        motor_instance->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan == &hfdcan2) 
    {
        motor_instance->CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan == &hfdcan3)
    {
        motor_instance->CAN_Manage_Object = &CAN3_Manage_Object;
    }
    motor_instance->CAN_Rx_ID = __CAN_Rx_ID;
    motor_instance->Encoder_Offset = __Encoder_Offset;
    motor_instance->Nearest_Angle = __Nearest_Angle;
    motor_instance->Gearbox_Rate = __Gearbox_Rate;
    motor_instance->Tx_Data = allocate_tx_data_DJI(hcan, __CAN_Rx_ID, Motor_DJI_GM6020_Driver_Version_DEFAULT);
    motor_instance->Flag = 0;
    motor_instance->Pre_Flag = 0;
    motor_instance->Target_Angle = 0.0f;
    motor_instance->Target_Omega = 0.0f;
    motor_instance->Target_Torque = 0.0f;
    motor_instance->Motor_Type = __Motor_Type;
}

void Motor_DJI_C610_Data_Process(DJI_Motor_Instance *motor_instance)
{
   // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data_Row *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data_Row *)motor_instance->CAN_Manage_Object->Rx_Buffer;

    // 处理大小端
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - motor_instance->Rx_Data.Pre_Encoder;
    if (delta_encoder < -C610_ENCODER_NUM_PER_ROUND / 2)
    {
        // 正方向转过了一圈
        motor_instance->Rx_Data.Total_Round++;
    }
    else if (delta_encoder > C610_ENCODER_NUM_PER_ROUND / 2)
    {
        // 反方向转过了一圈
        motor_instance->Rx_Data.Total_Round--;
    }
    motor_instance->Rx_Data.Total_Encoder = motor_instance->Rx_Data.Total_Round * C610_ENCODER_NUM_PER_ROUND + tmp_encoder;

    // 计算电机本身信息
    motor_instance->Rx_Data.Now_Angle = (float) motor_instance->Rx_Data.Total_Encoder / (float) C610_ENCODER_NUM_PER_ROUND * 2.0f * PI / motor_instance->Gearbox_Rate;
    motor_instance->Rx_Data.Now_Omega = (float) tmp_omega * BASIC_MATH_RPM_TO_RADPS / motor_instance->Gearbox_Rate;
    motor_instance->Rx_Data.Now_Torque = tmp_current / C610_CURRENT_TO_OUT * C610_CURRENT_TO_TORQUE * motor_instance->Gearbox_Rate;
    motor_instance->Rx_Data.Now_Temperature = tmp_buffer->Temperature + BASIC_MATH_CELSIUS_TO_KELVIN;

    // 存储预备信息
    motor_instance->Rx_Data.Pre_Encoder = tmp_encoder;
}

void Motor_DJI_C620_Data_Process(DJI_Motor_Instance *motor_instance)
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
   Struct_Motor_DJI_CAN_Rx_Data_Row *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data_Row *)motor_instance->CAN_Manage_Object->Rx_Buffer;

    // 处理大小端
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - motor_instance->Rx_Data.Pre_Encoder;
    if (delta_encoder < -C620_ENCODER_NUM_PER_ROUND / 2)
    {
        // 正方向转过了一圈
        motor_instance->Rx_Data.Total_Round++;
    }
    else if (delta_encoder > C620_ENCODER_NUM_PER_ROUND / 2)
    {
        // 反方向转过了一圈
        motor_instance->Rx_Data.Total_Round--; 
    }
    motor_instance->Rx_Data.Total_Encoder = motor_instance->Rx_Data.Total_Round * C620_ENCODER_NUM_PER_ROUND + tmp_encoder;

    // 计算电机本身信息
    motor_instance->Rx_Data.Now_Angle = (float) motor_instance->Rx_Data.Total_Encoder / (float) C620_ENCODER_NUM_PER_ROUND * 2.0f * PI / C620_Gearbox_Rate;
    motor_instance->Rx_Data.Now_Omega = (float) tmp_omega * BASIC_MATH_RPM_TO_RADPS / C620_Gearbox_Rate;
    motor_instance->Rx_Data.Now_Torque = tmp_current / C620_CURRENT_TO_OUT * C620_CURRENT_TO_TORQUE * C620_Gearbox_Rate;
    motor_instance->Rx_Data.Now_Temperature = tmp_buffer->Temperature + BASIC_MATH_CELSIUS_TO_KELVIN;
//  motor_instance->Rx_Data.Now_Power = power_calculate(POWER_K_0, POWER_K_1, POWER_K_2, POWER_A, Rx_Data.Now_Torque / C620_Gearbox_Rate, Rx_Data.Now_Omega * C620_Gearbox_Rate);

    // 存储预备信息
    motor_instance->Rx_Data.Pre_Encoder = tmp_encoder;
}



void Motor_DJI_Init_All()
{
    Motor_DJI_Init(&DJI_Motor_Instances[0], &hfdcan1, Motor_DJI_ID_0x201, 0, 0.0f, C620_Gearbox_Rate,Motor_DJI_Type_C620);
    Motor_DJI_Init(&DJI_Motor_Instances[1], &hfdcan1, Motor_DJI_ID_0x202, 0, 0.0f, C620_Gearbox_Rate,Motor_DJI_Type_C620);
    Motor_DJI_Init(&DJI_Motor_Instances[2], &hfdcan1, Motor_DJI_ID_0x203, 0, 0.0f, C620_Gearbox_Rate,Motor_DJI_Type_C620);
    Motor_DJI_Init(&DJI_Motor_Instances[3], &hfdcan1, Motor_DJI_ID_0x204, 0, 0.0f, C620_Gearbox_Rate,Motor_DJI_Type_C620);
    Motor_DJI_Init(&DJI_Motor_Instances[4], &hfdcan2, Motor_DJI_ID_0x201, 0, 0.0f, C610_Gearbox_Rate,Motor_DJI_Type_C610);
    Motor_DJI_Init(&DJI_Motor_Instances[5], &hfdcan2, Motor_DJI_ID_0x202, 0, 0.0f, C610_Gearbox_Rate,Motor_DJI_Type_C610);
}


/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Motor_DJI_TIM_100ms_Alive_PeriodElapsedCallback()
{
    for(uint8_t i = 0; i < DJI_Motor_C610_Num+DJI_Motor_C620_Num; i++)
    {
            // 判断该时间段内是否接收过电机数据
        if (DJI_Motor_Instances[i].Flag == DJI_Motor_Instances[i].Pre_Flag)
        {
            // 电机断开连接
            DJI_Motor_Instances[i].Motor_DJI_Status = Motor_DJI_Status_DISABLE;
            DJI_Motor_Instances[i].Target_Angle = 0.0f;
            DJI_Motor_Instances[i].Target_Omega = 0.0f;
            DJI_Motor_Instances[i].Target_Torque = 0.0f;
        }
        else
        {
            // 电机保持连接
            DJI_Motor_Instances[i].Motor_DJI_Status = Motor_DJI_Status_ENABLE;
        }
        DJI_Motor_Instances[i].Pre_Flag = DJI_Motor_Instances[i].Flag;
    }
}

void Motor_DJI_CAN1_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    if ((Header == NULL) || (Buffer == NULL))
    {
        return;
    }
    switch (Header->Identifier)
    {
        case (0x201):
        if(DJI_Motor_Instances[0].CAN_Manage_Object != NULL)
        {
            DJI_Motor_Instances[0].Flag +=1;
            switch (DJI_Motor_Instances[0].Motor_Type)
            {
            case Motor_DJI_Type_C610:
            Motor_DJI_C610_Data_Process(&DJI_Motor_Instances[0]);
                break;
            case Motor_DJI_Type_C620:
            Motor_DJI_C620_Data_Process(&DJI_Motor_Instances[0]);
                break;
            }

        }
        break;
        case (0x202):
        if(DJI_Motor_Instances[1].CAN_Manage_Object != NULL)
        {
            DJI_Motor_Instances[1].Flag +=1;
            switch (DJI_Motor_Instances[1].Motor_Type)
            {
            case Motor_DJI_Type_C610:
            Motor_DJI_C610_Data_Process(&DJI_Motor_Instances[1]);
                break;
            case Motor_DJI_Type_C620:
            Motor_DJI_C620_Data_Process(&DJI_Motor_Instances[1]);
                break;
            }
        }
        break;
        case (0x203):
        if(DJI_Motor_Instances[2].CAN_Manage_Object != NULL)
        {
            DJI_Motor_Instances[2].Flag +=1;
            switch (DJI_Motor_Instances[2].Motor_Type)
            {
            case Motor_DJI_Type_C610:
            Motor_DJI_C610_Data_Process(&DJI_Motor_Instances[2]);
                break;
            case Motor_DJI_Type_C620:
            Motor_DJI_C620_Data_Process(&DJI_Motor_Instances[2]);
                break;
            }
        }
        break;
        case (0x204):
        if(DJI_Motor_Instances[3].CAN_Manage_Object != NULL)
        {
            DJI_Motor_Instances[3].Flag +=1;
            switch (DJI_Motor_Instances[3].Motor_Type)
            {
            case Motor_DJI_Type_C610:
            Motor_DJI_C610_Data_Process(&DJI_Motor_Instances[3]);
                break;
            case Motor_DJI_Type_C620:
            Motor_DJI_C620_Data_Process(&DJI_Motor_Instances[3]);
                break;
            }
        }
        break;
    }
}

void Motor_DJI_CAN2_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    if ((Header == NULL) || (Buffer == NULL))
    {
        return;
    }
    switch (Header->Identifier)
    {
        case (0x201):
        if(DJI_Motor_Instances[4].CAN_Manage_Object != NULL)
        {
            DJI_Motor_Instances[4].Flag +=1;
            switch (DJI_Motor_Instances[4].Motor_Type)
            {
            case Motor_DJI_Type_C610:
            Motor_DJI_C610_Data_Process(&DJI_Motor_Instances[4]);
                break;
            case Motor_DJI_Type_C620:
            Motor_DJI_C620_Data_Process(&DJI_Motor_Instances[4]);
                break;
            }
        }
        break;
        case (0x202):
        if(DJI_Motor_Instances[5].CAN_Manage_Object != NULL)
        {
            DJI_Motor_Instances[5].Flag +=1;
            switch (DJI_Motor_Instances[5].Motor_Type)
            {
            case Motor_DJI_Type_C610:
            Motor_DJI_C610_Data_Process(&DJI_Motor_Instances[5]);
                break;
            case Motor_DJI_Type_C620:
            Motor_DJI_C620_Data_Process(&DJI_Motor_Instances[5]);
                break;
            }
        }
        break;
    }
}

void Motor_DJI_SetOutput()
{
    int16_t out;

    out = (int16_t)(DJI_Motor_Instances[0].Out);
    DJI_Motor_Instances[0].Tx_Data[0] = (uint8_t)(((uint16_t)out) >> 8);    //0~3底盘电机
    DJI_Motor_Instances[0].Tx_Data[1] = (uint8_t)((uint16_t)out);

    out = (int16_t)(DJI_Motor_Instances[1].Out);
    DJI_Motor_Instances[1].Tx_Data[0] = (uint8_t)(((uint16_t)out) >> 8);
    DJI_Motor_Instances[1].Tx_Data[1] = (uint8_t)((uint16_t)out);

    out = (int16_t)(DJI_Motor_Instances[2].Out);
    DJI_Motor_Instances[2].Tx_Data[0] = (uint8_t)(((uint16_t)out) >> 8);
    DJI_Motor_Instances[2].Tx_Data[1] = (uint8_t)((uint16_t)out);

    out = (int16_t)(DJI_Motor_Instances[3].Out);
    DJI_Motor_Instances[3].Tx_Data[0] = (uint8_t)(((uint16_t)out) >> 8);
    DJI_Motor_Instances[3].Tx_Data[1] = (uint8_t)((uint16_t)out);

    out = (int16_t)(DJI_Motor_Instances[4].Out);
    DJI_Motor_Instances[4].Tx_Data[0] = (uint8_t)(((uint16_t)out) >> 8);    //4~5抬升运动主动轮电机
    DJI_Motor_Instances[4].Tx_Data[1] = (uint8_t)((uint16_t)out);

    out = (int16_t)(DJI_Motor_Instances[5].Out);
    DJI_Motor_Instances[5].Tx_Data[0] = (uint8_t)(((uint16_t)out) >> 8);
    DJI_Motor_Instances[5].Tx_Data[1] = (uint8_t)((uint16_t)out);

}