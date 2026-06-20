#include "task_config_and_callback.h"

/* Private variables ---------------------------------------------------------*/
uint64_t us_time=0;
uint32_t ms_time=0;
uint16_t s_time=0;
uint16_t Servo_Angle1 = 0;
uint16_t Servo_Angle2 = 0;

// 全局初始化完成标志位
bool init_finished = false;
bool Calibration_finished = false; //底盘电机校准完成标志位

// 机器人模式全局状态
Enum_Robot_Mode Robot_Mode = Robot_Mode_Stop;

// LED灯
int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

void Motor_CanMessage_Transmit()
{

        CAN_Transmit_Data(&hfdcan1,0x200,CAN1_0x200_Tx_Data,8);
       
	    // CAN_Transmit_Data(&hfdcan2,0x200,CAN2_0x1ff_Tx_Data,8);
        CAN_Transmit_Data(&hfdcan3,0x1FF,CAN3_0x1ff_Tx_Data,8);

        Motor_DM_Normal_Output(&DM_Motor_Instances[0]);
        Motor_DM_Normal_Output(&DM_Motor_Instances[1]);
        Motor_DM_Normal_Output(&DM_Motor_Instances[2]);
        Motor_DM_Normal_Output(&DM_Motor_Instances[3]);  
}

void CAN1_Callback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    switch(Header->Identifier)
    {
        case (0x201):
        case (0x202):
        case (0x203):
        case (0x204):
        Motor_DJI_CAN1_RxCpltCallback(Header, Buffer);
        break;
    }
}

void CAN2_Callback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    Motor_DM_CAN2_RxCpltCallback(Header, Buffer);

}

void CAN3_Callback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
    switch(Header->Identifier)
    {
        case (0x205):
        case (0x206):
        Motor_DM_CAN3_RxCpltCallback(Header, Buffer);
        break;
    }
}

void serial_Callback(uint8_t *Buffer, uint16_t Length)
{
   PC_rx_idle_callback(Buffer, Length);
}


void Robot_Mode_Change_Check()
{
    if(rc_channels.ch[4]==0)
    {
        Robot_Mode = Robot_Mode_Stop;
    }
    else if(rc_channels.ch[4]<0)
    {
        Robot_Mode = Robot_Mode_Manual;
    }
    else if(rc_channels.ch[4]>0)
    {
        Robot_Mode = Robot_Mode_Auto;
    }
}

void Servo_Motor_Control()
{
    switch(Robot_Mode)
    {
        case Robot_Mode_Stop:
        Servo_Angle1 = 0.0f;
        Servo_Angle2 = 0.0f;
        break;
        case Robot_Mode_Manual:
        if(rc_channels.ch[7] < 0 )
        {
            Servo_Angle1 = 55.0f;
            Servo_Angle2 = 0.0f;
        }
        else if(rc_channels.ch[7] == 0 )
        {
            Servo_Angle1 = 55.0f;
            Servo_Angle2 = 90.0f;
        }
        else if(rc_channels.ch[7] > 0 )
        {
            Servo_Angle1 = 145.0f;
            Servo_Angle2 = 90.0f;
        }
        break;
        case Robot_Mode_Auto:
            switch (PC_frame.cmd_servocontrol)
            {
                case 0x00:
                Servo_Angle1 = 55.0f;
                Servo_Angle2 = 0.0f;
                break;
                case 0x01:
                Servo_Angle1 = 55.0f;
                Servo_Angle2 = 90.0f;
                break;
                case 0x02:
                Servo_Angle1 = 145.0f;
                Servo_Angle2 = 90.0f;
                break;
            }
        break;
    }
}

void Robot_Calibration_Check()
{
    switch (Robot_Mode)
    {
        case Robot_Mode_Stop:
        break;
        case Robot_Mode_Manual:
        if(ch9_status.Key_Status==CH_Status_TRIG_FREE_PRESSED)
        {
            Calibration_finished = true;
        }
        else
        {
            Calibration_finished = false;
        }
        break;
        case Robot_Mode_Auto:
        if(PC_frame.Calibration_Flag == 1)
        {
            Calibration_finished = true;
        }
        else
        {
            Calibration_finished = false;
        }
        break;
    }
}

/**
 * @brief 3600秒任务回调函数
 * @retval None
 */
void Task3600s_Callback()
{
    Timestamp_TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 1ms任务回调函数
 * @retval 后续可添加更多每1MS需要执行的任务
 */
void Task1ms_Callback()
{
    // PC_rx_timeout_1ms_process();
    Remote_Status_Update(&ch9_status, 9);
    MeasureFSM_Run();
    LiftFSM_Run();
    Robot_Calibration_Check();
    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red >= 18)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }
        if (green >= 18)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }
        if (blue >= 18)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        if (red_minus_flag)
        {
            red--;
        }
        else
        {
            red++;
        }
        if (green_minus_flag)
        {
            green--;
        }
        else
        {
            green++;
        }
        if (blue_minus_flag)
        {
            blue--;
        }
        else
        {
            blue++;
        }

        WS2812_Set_RGB(255, 0, 0);

        // 发送实例
        TIM_10ms_Write_PeriodElapsedCallback();
        Servo_Motor_Control();
    }

    BSP_Key_TIM_1ms_Process_PeriodElapsedCallback();
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        // 处理按键状态
        BSP_Key_TIM_50ms_Read_PeriodElapsedCallback();
    }
    static int mod100 = 0;
    mod100++;
    if (mod100 == 100)
    {
        mod100 = 0;

        // 处理电机状态
        Motor_DJI_TIM_100ms_Alive_PeriodElapsedCallback();
    }

    static int mod128 = 0;
    mod128++;
    if (mod128 == 128)
    {
        mod128 = 0;

        // 处理陀螺仪相关
        // BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
    }
    
    // filter_kalman.Vector_Z[0][0] = motor.Get_Now_Angle();
    // filter_kalman.Vector_Z[1][0] = motor.Get_Now_Omega();
    // filter_kalman.TIM_Predict_PeriodElapsedCallback();
    // filter_kalman.TIM_Update_PeriodElapsedCallback();

    // float yaw = BSP_BMI088.Get_Euler_Angle()[0][0] / BASIC_MATH_DEG_TO_RAD;
    // float pitch = BSP_BMI088.Get_Euler_Angle()[1][0] / BASIC_MATH_DEG_TO_RAD;
    // float roll = BSP_BMI088.Get_Euler_Angle()[2][0] / BASIC_MATH_DEG_TO_RAD;
    // float q0 = BSP_BMI088.Get_Quaternion()[0];
    // float q1 = BSP_BMI088.Get_Quaternion()[1];
    // float q2 = BSP_BMI088.Get_Quaternion()[2];
    // float q3 = BSP_BMI088.Get_Quaternion()[3];
    // float temperature = BSP_BMI088.BMI088_Accel.Get_Now_Temperature();
    // float calculating_time = BSP_BMI088.Get_Calculating_Time();
    // float loss = BSP_BMI088.Get_Accel_Chi_Square_Loss();
    // float origin_accel_x = BSP_BMI088.Get_Original_Accel()[0][0];
    // float origin_accel_y = BSP_BMI088.Get_Original_Accel()[1][0];
    // float origin_accel_z = BSP_BMI088.Get_Original_Accel()[2][0];
    // float origin_gyro_x = BSP_BMI088.Get_Original_Gyro()[0][0];
    // float origin_gyro_y = BSP_BMI088.Get_Original_Gyro()[1][0];
    // float origin_gyro_z = BSP_BMI088.Get_Original_Gyro()[2][0];
    // float now_time = SYS_Timestamp.Get_Now_Microsecond() / 1000000.0f;
    // float accel_x = BSP_BMI088.Get_Accel()[0][0];
    // float accel_y = BSP_BMI088.Get_Accel()[1][0];
    // float accel_z = BSP_BMI088.Get_Accel()[2][0];
    // float gyro_x = BSP_BMI088.Get_Gyro()[0][0];
    // float gyro_y = BSP_BMI088.Get_Gyro()[1][0];
    // float gyro_z = BSP_BMI088.Get_Gyro()[2][0];
    // float rotation_matrix_r00 = BSP_BMI088.Get_Rotation_Matrix()[0][0];
    // float rotation_matrix_r01 = BSP_BMI088.Get_Rotation_Matrix()[0][1];
    // float rotation_matrix_r02 = BSP_BMI088.Get_Rotation_Matrix()[0][2];
    // float rotation_matrix_r10 = BSP_BMI088.Get_Rotation_Matrix()[1][0];
    // float rotation_matrix_r11 = BSP_BMI088.Get_Rotation_Matrix()[1][1];
    // float rotation_matrix_r12 = BSP_BMI088.Get_Rotation_Matrix()[1][2];
    // float rotation_matrix_r20 = BSP_BMI088.Get_Rotation_Matrix()[2][0];
    // float rotation_matrix_r21 = BSP_BMI088.Get_Rotation_Matrix()[2][1];
    // float rotation_matrix_r22 = BSP_BMI088.Get_Rotation_Matrix()[2][2];
    // float motor_target_angle = motor.Get_Target_Angle();
    // float motor_now_angle = motor.Get_Now_Angle();
    // float motor_target_omega = motor.Get_Target_Omega();
    // float motor_now_omega = motor.Get_Now_Omega();
    // float motor_target_torque = motor.Get_Target_Torque();
    // float motor_now_torque = motor.Get_Now_Torque();
    // float filter_omega = filter_kalman.Vector_X[1][0];
    // float float_red = static_cast<float>(red);
    // float float_green = static_cast<float>(green);
    // float float_blue = static_cast<float>(blue);

}

/**
 * @brief 任务初始化函数
 * @retval None
 */
void Task_Init()
{
   //定时器时间戳初始化
    Timestamp_Init(&htim5);

   //USB通讯初始化
    USB_Init(serial_Callback);
    UART_Init(&huart10,hipnuc_data_unpacked); 
    UART_Init(&huart1,TFmini_GetDistanceFront);

    // 陀螺仪的SPI
    // SPI_Init(&hspi2, SPI2_Callback);

   //WS2812 spi初始化
    SPI_Init(&hspi6, NULL);

   //CAN初始化
    CAN_Init(&hfdcan1, CAN1_Callback);
    CAN_Init(&hfdcan2, CAN2_Callback);
    CAN_Init(&hfdcan3, CAN3_Callback);

   //电源ADC的初始化
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

   //定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim7);

   //蜂鸣器初始化
    Buzzer_Init(4000,0.0f);

   //舵机PWM初始化
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

   //5V,24V电源输出初始化  第一位参数控制CAN1的2+2，第二位控制CAN2的2+2,第三位控制5v开关
    BSP_Power_Init(false,true,true);


   //初始化WS2812灯珠
    WS2812_Init(0, 0, 0);

   //DM电机PID初始化
    Motor_DJI_InitPID();
    Motor_DM_InitPID();
    Motor_LK_InitPID();

   //初始化全部电机
    Motor_DJI_Init_All();
    Motor_DM_Init_All();
    Motor_LK_Init_All();
   //

   //标记初始化完成 
    init_finished = true;   
}

void Timestamp_fuc(void *argument)
{
    for(;;)
    {
    //    us_time = Timestamp_Get_Now_Microsecond();
    //    ms_time = Timestamp_Get_Now_Millisecond();
    //    s_time = Timestamp_Get_Now_Second();
       Robot_Mode_Change_Check(); 
       DJI_Motor_Output();
       DM_Motor_Output();
       Motor_CanMessage_Transmit();
       Servo_SetAngle(&htim1,TIM_CHANNEL_1,Servo_Angle1);
       Servo_SetAngle(&htim2,TIM_CHANNEL_3,Servo_Angle2);
       PC_transmitData();
       osDelay(1);
    }
}
