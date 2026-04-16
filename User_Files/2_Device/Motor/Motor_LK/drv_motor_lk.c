/**
 * @file drv_motor_lk.c
 * @author Oyyp
 * @brief LK电机驱动库
 * @version 0.1
 * @date 2026-04-13 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_motor_lk.h"


/* Private macros ------------------------------------------------------------*/


/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
float target;
LKMotor_Instance LK_Motor_Instances[LK_Motor_Num] = {0};
static uint8_t lk_circle_angle_valid[LK_Motor_Num] = {0};
static float lk_circle_angle_prev[LK_Motor_Num] = {0.0f};


//请求电机数据报文1，返回电机温度、电压和错误状态标志位
uint8_t LK_Motor_CAN_GETDATA1[8] = {0x9A,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//清楚电机错误标志位报文
uint8_t LK_Motor_CAN_CLEARError[8] = {0x9B,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  //电机状态没有恢复正常时，错误标志无法清除
//请求电机数据报文2，返回电机温度、编码器计数和电流
uint8_t LK_Motor_CAN_GETDATA2[8] = {0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//请求电机数据报文3，返回电机温度、和三相电流的数据
uint8_t LK_Motor_CAN_GETDATA3[8] = {0x9D,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//电机关闭指令，清除电机转动圈数以及之前接收的控制指令，仍可回复控制命令，但不执行动作
uint8_t LK_Motor_CAN_DISABLE[8] = {0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//电机开启指令,电机恢复正常工作
uint8_t LK_Motor_CAN_ENABLE[8] = {0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//停止电机，但不清除电机运行状态。再次发送控制指令即可控制电机动作
uint8_t LK_Motor_CAN_STOP[8] = {0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//读取电机抱闸器状态报文，返回电机抱闸器状态
uint8_t LK_Motor_CAN_GET_BreakState[8] = {0x8C,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//读取电机编码器数据报文，返回电机编码器计数、原始编码器计数和编码器零点偏移数据
uint8_t LK_Motor_CAN_GET_Encoder[8] = {0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//读取电机多圈角度数据报文，返回电机多圈角度数据
uint8_t LK_Motor_CAN_GET_MultiAngle[8] = {0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//读取电机单圈角度数据报文，返回电机单圈角度数据
uint8_t LK_Motor_CAN_GET_CircleAngle[8] = {0x94,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/* Private function declarations ---------------------------------------------*/
/**
 * @brief CAN3的LK电机报文接收回报函数
 * 
 * @param Header 
 * @param Buffer 
 */
void Motor_LK_CAN3_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    Motor_LK_Data_Process(Header, Buffer);
}


void Motor_LK_Frame1_Process(LKMotor_Instance *motor_instance)
{
    int16_t temp_voltage,temp_current;
    LKRXData_Frame1 *tmp_buffer = (LKRXData_Frame1 *)motor_instance->CAN_Manage_Object->Rx_Buffer;
    motor_instance->Rx_Data.Motor_Temperature =tmp_buffer->Motor_Temperature;
    temp_voltage = GET16(&tmp_buffer->Voltage);
    temp_current = GET16(&tmp_buffer->Current);
    motor_instance->Rx_Data.voltage = (float)temp_voltage * 0.01;
    motor_instance->Rx_Data.current = (float)temp_current * 0.01;
    motor_instance->Rx_Data.Motor_State = tmp_buffer->Motor_State;
    motor_instance->Rx_Data.Error_State = tmp_buffer->Error_State;

}

void Motor_LK_Frame2_Process(LKMotor_Instance *motor_instance)
{
    int16_t temp_torque;
    LKRXData_Frame2 *tmp_buffer = (LKRXData_Frame2 *)motor_instance->CAN_Manage_Object->Rx_Buffer;
    motor_instance->Rx_Data.Motor_Temperature =tmp_buffer->Motor_Temperature;
    temp_torque =(int16_t)GET16(&tmp_buffer->Current);
    switch (motor_instance->LK_Type)
    {
    case MS:
        motor_instance->Rx_Data.Torque_Current = (float)temp_torque;
        break;
    case MF:
        motor_instance->Rx_Data.Torque_Current = (float)temp_torque * MF_Current_Mapping;
        break;
    case MG:
        motor_instance->Rx_Data.Torque_Current = (float)temp_torque * MG_Current_Mapping;
        break;
    default:
        break;
    }
    motor_instance->Rx_Data.Speed = ((int16_t)GET16(&tmp_buffer->Speed))/motor_instance->Reduction_Ratio;
    motor_instance->Rx_Data.Encoder =(uint16_t)GET16(&tmp_buffer->Encoder);
}

void Motor_LK_Frame3_Process(LKMotor_Instance *motor_instance)
{
    int16_t temp_iA,temp_iB,temp_iC;
    LKRXData_Frame3 *tmp_buffer = (LKRXData_Frame3 *)motor_instance->CAN_Manage_Object->Rx_Buffer;
    motor_instance->Rx_Data.Motor_Temperature =tmp_buffer->Motor_Temperature;
    temp_iA =(int16_t)GET16(&tmp_buffer->iA);
    temp_iB =(int16_t)GET16(&tmp_buffer->iB);
    temp_iC =(int16_t)GET16(&tmp_buffer->iC);
    switch (motor_instance->LK_Type)
    {
        case MS:
        break;
        case MF:
        motor_instance->Rx_Data.iA = (float)temp_iA * MF_Current_Mapping;
        motor_instance->Rx_Data.iB = (float)temp_iB * MF_Current_Mapping;
        motor_instance->Rx_Data.iC = (float)temp_iC * MF_Current_Mapping;
        break;
        case MG:
        motor_instance->Rx_Data.iA = (float)temp_iA * MG_Current_Mapping;
        motor_instance->Rx_Data.iB = (float)temp_iB * MG_Current_Mapping;
        motor_instance->Rx_Data.iC = (float)temp_iC * MG_Current_Mapping;
    }
}

void Motor_LK_BreakState_Process(LKMotor_Instance *motor_instance)
{
    LKRXData_Breakstate *tmp_buffer = (LKRXData_Breakstate *)motor_instance->CAN_Manage_Object->Rx_Buffer;
    motor_instance->Rx_Data.brake_state = tmp_buffer->brake_state;
}

void Motor_LK_Encoder_Process(LKMotor_Instance *motor_instance)
{
    LKRXData_Encoder *tmp_buffer = (LKRXData_Encoder *)motor_instance->CAN_Manage_Object->Rx_Buffer;
    motor_instance->Rx_Data.Encoder = (uint16_t)GET16(&tmp_buffer->Encoder);
    motor_instance->Rx_Data.EncoderRaw = (uint16_t)GET16(&tmp_buffer->Encoder);
    motor_instance->Rx_Data.EncoderOffset = (uint16_t)GET16(&tmp_buffer->EncoderOffset);
}

void Motor_LK_MultiAngle_Process(LKMotor_Instance *motor_instance)
{
    int64_t temp_multiAngle;
    LKRXData_MultiAngle *tmp_buffer = (LKRXData_MultiAngle *)motor_instance->CAN_Manage_Object->Rx_Buffer;
    uint64_t raw_angle;
    raw_angle = ((uint64_t)tmp_buffer->angle_b[0]) |
                ((uint64_t)tmp_buffer->angle_b[1] << 8) |
                ((uint64_t)tmp_buffer->angle_b[2] << 16) |
                ((uint64_t)tmp_buffer->angle_b[3] << 24) |
                ((uint64_t)tmp_buffer->angle_b[4] << 32) |
                ((uint64_t)tmp_buffer->angle_b[5] << 40) |
                ((uint64_t)tmp_buffer->angle_b[6] << 48);
    if (raw_angle & (1ULL << 55))
    {
        raw_angle |= 0xFF00000000000000ULL;
    }

    temp_multiAngle = (int64_t)raw_angle;
    motor_instance->Rx_Data.Motor_multiAngle = (float)(temp_multiAngle+1) /100.0f/motor_instance->Reduction_Ratio;
}

void Motor_LK_CircleAngle_Process(LKMotor_Instance *motor_instance)
{
    int32_t idx;
    uint32_t temp_circleAngle;
    float temp_nowCircleAngle, err_CircleAngle;
    LKRXData_CircleAngle *tmp_buffer = (LKRXData_CircleAngle *)motor_instance->CAN_Manage_Object->Rx_Buffer;

    idx = (int32_t)(motor_instance - &LK_Motor_Instances[0]);
    if ((idx < 0) || (idx >= LK_Motor_Num))
    {
        return;
    }

    temp_circleAngle = (uint32_t)(GET32(&tmp_buffer->CircleAngle)+1)/(motor_instance->Reduction_Ratio);
    temp_nowCircleAngle = (float)temp_circleAngle / 100.0f;

    if (!lk_circle_angle_valid[idx])
    {
        lk_circle_angle_valid[idx] = 1;
        lk_circle_angle_prev[idx] = temp_nowCircleAngle;
        motor_instance->Rx_Data.CircleAngle = temp_nowCircleAngle;
        motor_instance->Rx_Data.Motor_multiAngle = temp_nowCircleAngle;
        return;
    }

    err_CircleAngle = temp_nowCircleAngle - lk_circle_angle_prev[idx];
    if (err_CircleAngle > 180.0f)
    {
        err_CircleAngle -= 360.0f;
    }
    else if (err_CircleAngle < -180.0f)
    {
        err_CircleAngle += 360.0f;
    }

    motor_instance->Rx_Data.CircleAngle = temp_nowCircleAngle;
    motor_instance->Rx_Data.Motor_multiAngle += err_CircleAngle;
    lk_circle_angle_prev[idx] = temp_nowCircleAngle;
}
/**
 * @brief 正常模式下反馈报文数据处理过程
 *
 */
void Motor_LK_Data_Process(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    if ((Header == NULL) || (Buffer == NULL))
    {
        return;
    }
    uint8_t cmd_id = Buffer[0];
    switch (Header->Identifier)
    {
        case (0x141):
        switch (cmd_id)
        {
            case 0x9A:
            case 0x9B:
            Motor_LK_Frame1_Process(&LK_Motor_Instances[0]);
            break;
            case 0x9C:
            case 0xA0:
            case 0xA1:
            case 0xA2:
            case 0xA3:
            case 0xA4:
            case 0xA5:
            case 0xA6:
            case 0xA7:
            case 0xA8:
            Motor_LK_Frame2_Process(&LK_Motor_Instances[0]);
            break;
            case 0x9D:
            Motor_LK_Frame3_Process(&LK_Motor_Instances[0]);
            break;
            case 0x8C:
            Motor_LK_BreakState_Process(&LK_Motor_Instances[0]);
            break;
            case 0x90:
            Motor_LK_Encoder_Process(&LK_Motor_Instances[0]);
            break;
            case 0x92:
            Motor_LK_MultiAngle_Process(&LK_Motor_Instances[0]);
            break;
            case 0x94:
            Motor_LK_CircleAngle_Process(&LK_Motor_Instances[0]);
            break;
        }
        break;
        case (0x142):
        switch (cmd_id)
        {
            case 0x9A:
            case 0x9B:
            Motor_LK_Frame1_Process(&LK_Motor_Instances[1]);
            break;
            case 0x9C:
            case 0xA0:
            case 0xA1:
            case 0xA2:
            case 0xA3:
            case 0xA4:
            case 0xA5:
            case 0xA6:
            case 0xA7:
            case 0xA8:
            Motor_LK_Frame2_Process(&LK_Motor_Instances[1]);
            break;
            case 0x9D:
            Motor_LK_Frame3_Process(&LK_Motor_Instances[1]);
            break;
            case 0x8C:
            Motor_LK_BreakState_Process(&LK_Motor_Instances[1]);
            break;
            case 0x90:
            Motor_LK_Encoder_Process(&LK_Motor_Instances[1]);
            break;
            case 0x92:
            Motor_LK_MultiAngle_Process(&LK_Motor_Instances[1]);
            break;
            case 0x94:
            Motor_LK_CircleAngle_Process(&LK_Motor_Instances[1]);
            break;
        }
        break;
        default:
        break;
    }
}


/**
 * @brief LK电机实例初始化
 * 
 * @param motor_instance 
 * @param hcan 
 * @param __ID 
 */
void Motor_LK_Init(LKMotor_Instance *motor_instance,LKMotor_type motor_type, const FDCAN_HandleTypeDef *hcan,uint16_t __ID,float reduction_ratio)
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
    motor_instance->CAN_Rx_ID = DEVICE_STD_ID+__ID;
    motor_instance->CAN_Tx_ID = DEVICE_STD_ID+__ID;
    motor_instance->Reduction_Ratio = reduction_ratio;
    motor_instance->LK_Type = motor_type;
}

/**
 * @brief 初始化所有LK电机实例
 */
void Motor_LK_Init_All(void)
{
    Motor_LK_Init(&LK_Motor_Instances[0],MG,&hfdcan3,1,6.0f);
    Motor_LK_Init(&LK_Motor_Instances[1],MG,&hfdcan3,2,6.0f);
}

/**
 * @brief 停止电机
 * 
 * @param motor_instance 
 */
void Motor_LK_Stop(LKMotor_Instance *motor_instance)
{
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, LK_Motor_CAN_STOP, 8);
}

/**
 * @brief 获取电机的温度，电流，电压，错误码，状态码
 * 
 * @param motor_instance 
 */
void Motor_LK_GetFrame1(LKMotor_Instance *motor_instance)
{
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, LK_Motor_CAN_GETDATA1, 8);
}

/**
 * @brief 获取电机的温度，电流，速度，编码器
 * 
 * @param motor_instance 
 */
void Motor_LK_GetFrame2(LKMotor_Instance *motor_instance)
{
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, LK_Motor_CAN_GETDATA2, 8);
}

/**
 * @brief 获取电机的温度，三相电,数据
 * 
 * @param motor_instance 
 */
void Motor_LK_GetFrame3(LKMotor_Instance *motor_instance)
{
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, LK_Motor_CAN_GETDATA3, 8);
}

/**
 * @brief 读取电机的单圈角度
 * 
 * @param motor_ptr 
 */
void Motor_LK_GetCircleAngle(LKMotor_Instance *motor_ptr)
{
    CAN_Transmit_Data(motor_ptr->CAN_Manage_Object->CAN_Handler, motor_ptr->CAN_Tx_ID, LK_Motor_CAN_GET_CircleAngle, 8);
}

/**
 * @brief 读取电机的多圈角度
 * 
 * @param motor_ptr 
 */
void Motor_LK_GetMultiAngle(LKMotor_Instance *motor_ptr)
{
    CAN_Transmit_Data(motor_ptr->CAN_Manage_Object->CAN_Handler, motor_ptr->CAN_Tx_ID, LK_Motor_CAN_GET_MultiAngle, 8);
}

/**
 * @brief 读取电机的编码器数据命令 
 * 
 */
void Motor_LK_GetEncoder(LKMotor_Instance *motor_ptr)
{
    CAN_Transmit_Data(motor_ptr->CAN_Manage_Object->CAN_Handler, motor_ptr->CAN_Tx_ID, LK_Motor_CAN_GET_Encoder, 8);
}

/**
 * @brief 读取电机的抱闸器状态
 * 
 */
void Motor_LK_GetBreakState(LKMotor_Instance *motor_ptr)
{
    CAN_Transmit_Data(motor_ptr->CAN_Manage_Object->CAN_Handler, motor_ptr->CAN_Tx_ID, LK_Motor_CAN_GET_BreakState, 8);
}

/**
 * @brief 增量位置闭环控制命令 2
 * @details 主机发送该命令以控制电机的位置增量
 *          1.该命令下电机的最大速度由入参中的 maxSpeed 值限制。
 *          2.该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 *          3.该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制；
 * @param id 
 * @param angleIncrement 对应实际位置为 0.01degree/LSB
 */
void Motor_LK_AddAngle2(LKMotor_Instance *motor_instance,int32_t angleIncrement,uint32_t maxSpeed)
{
    int32_t anglecontrol = angleIncrement*100;
    uint32_t u = (uint32_t)anglecontrol;
    uint8_t Data[8]={0};
    Data[0]=0xA8;
    Data[1]=0x00;   
    Data[2]=(uint8_t)(maxSpeed & 0xFF);
    Data[3]=(uint8_t)((maxSpeed>>8) & 0xFF);
    Data[4]=(uint8_t)(u & 0xFF);
    Data[5]=(uint8_t)((u>>8) & 0xFF);
    Data[6]=(uint8_t)((u>>16) & 0xFF);
    Data[7]=(uint8_t)((u>>24) & 0xFF);
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, Data, 8);
}

/**
 * @brief 增量位置闭环控制命令 1
 * @details 主机发送该命令以控制电机的位置增量
 *          1.该命令下电机的最大速度由上位机中的 Max Speed 值限制。
 *          2.该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 *          3.该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制；
 * @param id 
 * @param angleIncrement 
 */
void Motor_LK_AddAngle1(LKMotor_Instance *motor_instance,int32_t angleIncrement)
{
    int32_t anglecontrol = angleIncrement*100;
    uint32_t u = (uint32_t)anglecontrol;
    uint8_t Data[8]={0};
    Data[0]=0xA7;
    Data[1]=0x00;   
    Data[2]=0x00;
    Data[3]=0x00;
    Data[4]=(uint8_t)(u & 0xFF);
    Data[5]=(uint8_t)((u>>8) & 0xFF);
    Data[6]=(uint8_t)((u>>16) & 0xFF);
    Data[7]=(uint8_t)((u>>24) & 0xFF);
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, Data, 8);
}

/**
 * @brief 单圈位置闭环控制命令 2
 * @details 主机发送该命令以控制电机的位置（单圈角度）。
 *          1.该命令下电机的最大速度由入参中的 maxSpeed 值限制。
 *          2.该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 *          3.该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制；
 * @param id 
 * @param Direction 方向 0x00：顺时针 0x01：逆时针
 * @param maxSpeed 最大速度 限制了转动的最大速度，单位，dps，360 代表 360dps（degree per second，度每秒）
 * @param circleAngle 单圈角度值，以编码器零点为起始点，顺时针增加，再次到达零点时数值回0，单位0.01°/LSB，数值范围0~36000*减速比-1(35999)
 */
void Motor_LK_LoopAngleControl2(LKMotor_Instance *motor_instance,uint8_t Direction,uint16_t maxSpeed,uint32_t circleAngle)
{
    uint16_t temp_maxSpeed;
    uint32_t temp_circleAngle;
    temp_maxSpeed = maxSpeed * motor_instance->Reduction_Ratio;
    temp_circleAngle = circleAngle*motor_instance->Reduction_Ratio;
    uint8_t Data[8]={0};
    Data[0]=0xA6;
    Data[1]=Direction;   
    Data[2]=(uint8_t)(temp_maxSpeed & 0xFF);
    Data[3]=(uint8_t)((temp_maxSpeed>>8) & 0xFF);
    Data[4]=(uint8_t)(temp_circleAngle & 0xFF);
    Data[5]=(uint8_t)((temp_circleAngle>>8) & 0xFF);
    Data[6]=(uint8_t)((temp_circleAngle>>16) & 0xFF);
    Data[7]=(uint8_t)((temp_circleAngle>>24) & 0xFF);
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, Data, 8);
}

/**
 * @brief 单圈位置闭环控制命令 1
 * @details 主机发送该命令以控制电机的位置（单圈角度）。
 *          1.该命令下电机的最大速度由上位机中的 Max Speed 值限制。
 *          2.该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 *          3.该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制；
 * @param id 
 * @param Direction 方向 0x00：顺时针 0x01：逆时针
 * @param circleAngle 单圈角度值，以编码器零点为起始点，顺时针增加，再次到达零点时数值回0，单位0.01°/LSB，数值范围0~36000*减速比-1(35999)
 */
void Motor_LK_LoopAngleControl1(LKMotor_Instance *motor_instance,uint8_t Direction,uint32_t circleAngle)
{
    uint32_t temp_circleAngle = circleAngle * motor_instance->Reduction_Ratio;
    uint8_t Data[8]={0};
    Data[0]=0xA5;
    Data[1]=Direction;
    Data[2]=0x00;
    Data[3]=0x00;
    Data[4]=(uint8_t)(temp_circleAngle & 0xFF);
    Data[5]=(uint8_t)((temp_circleAngle>>8) & 0xFF);
    Data[6]=(uint8_t)((temp_circleAngle>>16) & 0xFF);
    Data[7]=(uint8_t)((temp_circleAngle>>24) & 0xFF);
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, Data, 8);
}

/**
 * @brief 单圈位置闭环控制命令 1（自动判定最短方向）
 * @param motor_instance 电机实例
 * @param angle 目标单圈角度，单位°，支持小数（如325.55）
 * @param maxSpeed 最大速度，单位dps
 */
void Motor_LK_LoopAngleControl1_AutoDirection(LKMotor_Instance *motor_instance,float angle,uint16_t maxSpeed)
{
    const int32_t circle_modulus = 36000;
    const uint32_t hold_deadband = 450;    // 3.00 deg
    int32_t target = (int32_t)(angle * 100.0f + ((angle >= 0.0f) ? 0.5f : -0.5f));
    int32_t current = (int32_t)(motor_instance->Rx_Data.CircleAngle * 100.0f + 0.5f);
    uint32_t cw;
    uint32_t ccw;
    uint32_t min_err;
    uint8_t direction;

    target %= circle_modulus;
    if (target < 0)
    {
        target += circle_modulus;
    }

    current %= circle_modulus;
    if (current < 0)
    {
        current += circle_modulus;
    }

    cw = (uint32_t)((target + circle_modulus - current) % circle_modulus);
    ccw = (uint32_t)((current + circle_modulus - target) % circle_modulus);
    min_err = (cw < ccw) ? cw : ccw;
    if (min_err <= hold_deadband)
    {
        Motor_LK_Stop(motor_instance);
        return;
    }
    direction = (cw <= ccw) ? 0x00 : 0x01;

    Motor_LK_LoopAngleControl2(motor_instance, direction, maxSpeed, (uint32_t)target);
}

/**
 * @brief 多圈位置闭环控制命令 2
 * @details 主机发送该命令以控制电机的位置。
 *          1.该命令下电机的最大速度由入参中的 maxSpeed 值限制。0~360dps
 *          2.该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 *          3.该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque Current 值限制；
 *          4.电机转动方向由目标位置和当前位置的差值决定。
 * @param id 
 * @param angle 对应实际位置为 0.01degree/LSB
 */
void Motor_LK_LoopsAngleControl2(LKMotor_Instance *motor_instance,float angle,uint16_t maxSpeed)
{
    int32_t anglecontrol = angle*100;
    uint32_t u = (uint32_t)anglecontrol;
    uint8_t Data[8]={0};
    Data[0]=0xA4;
    Data[1]=0x00;   
    Data[2]=(uint8_t)(maxSpeed & 0xFF);
    Data[3]=(uint8_t)((maxSpeed>>8) & 0xFF);
    Data[4]=(uint8_t)(u & 0xFF);
    Data[5]=(uint8_t)((u>>8) & 0xFF);
    Data[6]=(uint8_t)((u>>16) & 0xFF);
    Data[7]=(uint8_t)((u>>24) & 0xFF);
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, Data, 8);
}

void Motor_LK_TorqueControl(LKMotor_Instance *motor_instance,int16_t iqControl)
{
    uint16_t u1 = (uint16_t)iqControl;
    uint8_t Data[8]={0};
    Data[0]=0xA1;
    Data[1]=0x00;   
    Data[2]=0x00;
    Data[3]=0x00;
    Data[4]=(uint8_t)(u1 & 0xFF);
    Data[5]=(uint8_t)((u1>>8) & 0xFF);
    Data[6]=0x00;
    Data[7]=0x00;
    CAN_Transmit_Data(motor_instance->CAN_Manage_Object->CAN_Handler, motor_instance->CAN_Tx_ID, Data, 8);
}

void Motor_LK_GetFeedback(LKMotor_Instance *motor_instance)
{
    Motor_LK_GetFrame1(motor_instance);
    osDelay(1);
    Motor_LK_GetFrame2(motor_instance);
    osDelay(1);
    Motor_LK_GetCircleAngle(motor_instance);
    osDelay(1);
}
/* Function prototypes -------------------------------------------------------*/
// void LKsetOutput(void *argument)
// {
//     static float target_filtered = 0.0f;
//     float target_raw;

//     for(;;)
//     {
//         target_raw = (rc_channels.ch[11]+820)*0.2f;
//         target_filtered += 0.2f * (target_raw - target_filtered);
//         target = target_filtered;
//         Motor_LK_GetFrame1(&LK_Motor_Instances[0]);
//         osDelay(1);
//         Motor_LK_GetCircleAngle(&LK_Motor_Instances[0]);
//         osDelay(1);
//         if(rc_channels.ch[6]<=0)
//         {
//             Motor_LK_Stop(&LK_Motor_Instances[0]);
//             osDelay(1);
//         }
//         else
//         {
//             Motor_LK_LoopAngleControl1_AutoDirection(&LK_Motor_Instances[0], target, 15);
//         }
//         osDelay(1); 
//     }
// }

