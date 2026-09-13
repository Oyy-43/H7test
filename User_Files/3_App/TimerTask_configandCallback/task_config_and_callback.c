#include "task_config_and_callback.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
uint16_t Banding_Flag = 0;
uint64_t us_time=0;
uint32_t ms_time=0;
uint16_t s_time=0;
uint16_t Servo_Angle1 = 0;
uint16_t Servo_Angle2 = 0;
bool imu_yaw_init_flag = false;

bool IO_Status[4]= {false, false, false, false};  //0是夹爪上面的光电，1是对接完成信号，2是夹爪下面的光电
// 全局初始化完成标志位
bool init_finished = false;
bool Calibration_finished = false; //底盘电机校准完成标志位

BSP_KEY_S KeyBoard ={
    .Pre_GPIO_State = GPIO_PIN_SET,
	.Now_GPIO_State = GPIO_PIN_SET,
	.Key_Status = BSP_Key_Status_FREE,
};

const GPIO_PinState KEYBOARD_FREE_STATE = GPIO_PIN_SET;
// 机器人模式全局状态
Enum_Robot_Mode Robot_Mode = Robot_Mode_Stop;

// 比赛项目全局状态
Enum_Competition_Mode Competition_Mode = Competition_Mode_None;

// 按键模式选择状态机相关变量
static Enum_Key_Select_State Key_Select_State = Key_Select_State_Idle;
static uint16_t Key_LongPress_Cnt = 0;       // 长按计时计数器（ms）
static uint8_t  Key_Select_Press_Cnt = 0;    // 选择期间的短按次数
#define KEY_LONG_PRESS_THRESHOLD_MS  500     // 长按判定阈值 500ms

// LED灯
int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

void Check_IO_INPUT()
{
    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)
    {
        IO_Status[0] = true;
    }
    else
    {
        IO_Status[0] = false;
    }
    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_SET)
    {
        IO_Status[1] = true;
    }
    else
    {
        IO_Status[1] = false;
    }
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    {
        IO_Status[2] = true;
        Banding_Flag++;
        Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_D6, 1.0f, 80); // 发出提示音
        if(Banding_Flag>65534)
        {
            Banding_Flag=65534;
        }
    }
    else
    {
        Banding_Flag= 0;
        IO_Status[2] = false;
    }
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
    {
        IO_Status[3] = true;

    }
    else
    {
        IO_Status[3] = false;
    }
}

void KeyBoard_Init()
{
    KeyBoard.Pre_GPIO_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
}

void KeyBoard_TIM_1ms_Process_PeriodElapsedCallback()
{
if (KeyBoard.Pre_GPIO_State == KEYBOARD_FREE_STATE)
    {
        if (KeyBoard.Now_GPIO_State == KEYBOARD_FREE_STATE)
        {
            KeyBoard.Key_Status = BSP_Key_Status_FREE;
        }
        else
        {
            KeyBoard.Key_Status = BSP_Key_Status_TRIG_FREE_PRESSED;
        }
    }
    else
    {
        if (KeyBoard.Now_GPIO_State == KEYBOARD_FREE_STATE)
        {
            KeyBoard.Key_Status = BSP_Key_Status_TRIG_PRESSED_FREE;
        }
        else
        {
            KeyBoard.Key_Status = BSP_Key_Status_PRESSED;
        }
    }
    KeyBoard.Pre_GPIO_State = KeyBoard.Now_GPIO_State;
}

void KeyBoard_TIM_50ms_Read_PeriodElapsedCallback()
{
    KeyBoard.Now_GPIO_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
}

/**
 * @brief 按键模式选择处理函数（每1ms调用一次）
 *        长按500ms进入选择状态 -> 短按计数 -> 长按500ms退出并确定比赛项目
 * @retval None
 */
void KeyBoard_Mode_Select_Process()
{
    switch (Key_Select_State)
    {
        case Key_Select_State_Idle:
            // 空闲状态：检测长按
            if (KeyBoard.Key_Status == BSP_Key_Status_PRESSED)
            {
                Key_LongPress_Cnt++;
                if (Key_LongPress_Cnt >= KEY_LONG_PRESS_THRESHOLD_MS)
                {
                    // 长按达到阈值，进入选择模式
                    Key_Select_State = Key_Select_State_Entering;
                    Key_LongPress_Cnt = 0;
                    Key_Select_Press_Cnt = 0;
                    // 蜂鸣器提示进入选择模式（一声长响）
                    Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_A6, 1.0f, 100);
                }
            }
            else
            {
                Key_LongPress_Cnt = 0;
            }
            break;

        case Key_Select_State_Entering:
            // 等待松手确认进入
            if (KeyBoard.Key_Status == BSP_Key_Status_TRIG_PRESSED_FREE)
            {
                Key_Select_State = Key_Select_State_Counting;
                // 短提示音确认已进入
                Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_A6, 0.5f, 50);
            }
            break;

        case Key_Select_State_Counting:
            // 长按退出检测
            if (KeyBoard.Key_Status == BSP_Key_Status_PRESSED)
            {
                Key_LongPress_Cnt++;
                if (Key_LongPress_Cnt >= KEY_LONG_PRESS_THRESHOLD_MS)
                {
                    Key_Select_State = Key_Select_State_Exiting;
                    Key_LongPress_Cnt = 0;
                }
            }
            else
            {
                Key_LongPress_Cnt = 0;
            }
            // 短按计数（只检测按下触发沿）
            if (KeyBoard.Key_Status == BSP_Key_Status_TRIG_FREE_PRESSED)
            {
                Key_Select_Press_Cnt++;
                Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_A6, 0.2f, 30);
            }
            break;

        case Key_Select_State_Exiting:
            // 等待松手确认退出
            if (KeyBoard.Key_Status == BSP_Key_Status_TRIG_PRESSED_FREE)
            {
                // 根据短按次数确定比赛项目
                switch (Key_Select_Press_Cnt)
                {
                    case 0:
                        Competition_Mode = Competition_Mode_None;
                        PC_Transmit_Frame.Retry_Flag = 0x00; // 正常执行程序
                        break;
                    case 1:
                        PC_Transmit_Frame.Retry_Flag = 0x01; // 从武馆重试到梅林 短按零次
                        Competition_Mode = Competition_Mode_1;
                        break;
                    case 2:
                        PC_Transmit_Frame.Retry_Flag = 0x02; // 单项赛上3区代码 短按一次
                        Competition_Mode = Competition_Mode_Single_3Zone;
                        break;
                    case 3:
                        PC_Transmit_Frame.Retry_Flag = 0x03; // 对抗赛3区代码 短按两次
                        Competition_Mode = Competition_Mode_Battle_3ZoneLeft;
                        break;
                    case 4:
                        PC_Transmit_Frame.Retry_Flag = 0x04; // 对抗赛3区代码 短按三次
                        Competition_Mode = Competition_Mode_Battle_3ZoneMid;
                        break;
                    case 5:
                        PC_Transmit_Frame.Retry_Flag = 0x05;
                        Competition_Mode = Competition_Mode_Battle_3ZoneRight; //短按四次
                        break;
                }

                // 选择有效比赛项目后，确保处于Stop等待用户双击启动
                if (Competition_Mode != Competition_Mode_None)
                {
                    Robot_Mode = Robot_Mode_Stop;
                }

                // 退出提示音
                Buzzer_Play_Once_NonBlocking(BUZZER_FREQUENCY_A6, 0.5f, 50);

                // 回到空闲状态
                Key_Select_State = Key_Select_State_Idle;
            }
            break;

        default:
            Key_Select_State = Key_Select_State_Idle;
            break;
    }
}

void Motor_CanMessage_Transmit()
{
    /* 发送前强制清零，防止缓冲区被意外污染导致脏数据发送 */
    // memset(CAN1_0x200_Tx_Data, 0, 8);

        CAN_Transmit_Data(&hfdcan1,0x200,CAN1_0x200_Tx_Data,8);
       
	    // CAN_Transmit_Data(&hfdcan2,0x200,CAN2_0x1ff_Tx_Data,8);
    // memset(CAN3_0x1ff_Tx_Data, 0, 8);
        CAN_Transmit_Data(&hfdcan3,0x1FF,CAN3_0x1ff_Tx_Data,8);

        Motor_DM_Normal_Output(&DM_Motor_Instances[0]);
        Motor_DM_Normal_Output(&DM_Motor_Instances[1]);
        Motor_DM_Normal_Output(&DM_Motor_Instances[2]);
        Motor_DM_Normal_Output(&DM_Motor_Instances[3]);  
}

void Remote_Valt_OutputControl()
{
    if(rc_channels.ch[8]<=0)
    {
        BSP_Power_Set_DC24_0(false);
    }
    else
    {
        BSP_Power_Set_DC24_0(true);
    }
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


#define DOUBLE_CLICK_TIMEOUT_MS  400     // 双击判定时间窗口（ms）

void Robot_Mode_Change_Check()
{
    static uint8_t  click_count = 0;
    static uint16_t click_timer = 0;

    // 模式选择期间，屏蔽双击切换
    if (Key_Select_State != Key_Select_State_Idle)
    {
        click_count = 0;
        click_timer = 0;
        return;
    }

    // 检测按下触发沿
    if (KeyBoard.Key_Status == BSP_Key_Status_TRIG_FREE_PRESSED)
    {
        click_count++;
        if (click_count == 1)
        {
            click_timer = 0;  // 第一次按下，复位计时器
        }
    }

    // 有点击时开始计时
    if (click_count > 0)
    {
        click_timer++;

        if (click_count >= 2)
        {
            // 检测到双击 → 切换 Stop ↔ Auto
            if (Robot_Mode == Robot_Mode_Stop)
            {
                Robot_Mode = Robot_Mode_Auto;
            }
            else if (Robot_Mode == Robot_Mode_Auto)
            {
                Robot_Mode = Robot_Mode_Stop;
            }
            click_count = 0;
            click_timer = 0;
        }
        else if (click_timer >= DOUBLE_CLICK_TIMEOUT_MS)
        {
            // 超时，单次点击忽略
            click_count = 0;
            click_timer = 0;
        }
    }
}

void Servo_Motor_Control()
{
    switch(Robot_Mode)
    {
        case Robot_Mode_Stop:
        Servo_Angle1 = 200.0f;//200
        Servo_Angle2 = 0.0f;
        break;
        case Robot_Mode_Manual:
        if(rc_channels.ch[7] < 0 )
        {
            Servo_Angle1 = 68.0f;
            Servo_Angle2 = 0.0f;
        }
        else if(rc_channels.ch[7] == 0 )
        {
            Servo_Angle1 = 68.0f;
            Servo_Angle2 = 180.0f;
        }
        else if(rc_channels.ch[7]>0 && rc_channels.ch[5]>0)
        {
            Servo_Angle1 = 200.0f;
            Servo_Angle2 = 0.0f;
        }
        else if(rc_channels.ch[7] > 0 )
        {
            Servo_Angle1 = 200.0f;
            Servo_Angle2 = 180.0f;
        }
        break;
        case Robot_Mode_Auto:
            switch(GetWeapon_State_t.state)
            {
                case GetWeapon_Idle:
                case GetWeapon_RuntoPosition1:
                case GetWeapon_Process0:
                case GetWeapon_Process1:
                Servo_Angle1 = 73.0f;  //68.0
                Servo_Angle2 = 0.0f;
                break;
                case GetWeapon_Process2:
                Servo_Angle1 = 73.0f;
                Servo_Angle2 = 180.0f;
                break;
                case GetWeapon_Process3:
                Servo_Angle1 = 200.0f;
                Servo_Angle2 = 180.0f;
                break;
                case GetWeapon_Process4:
                Servo_Angle1 = 200.0f;
                Servo_Angle2 = 180.0f;
                break;
                case GetWeapon_Process5:
                case GetWeapon_Process6:
                Servo_Angle1 = 200.0f;
                Servo_Angle2 = 180.0f;
                break;
                case GetWeapon_Process7:
                Servo_Angle1 = 200.0f;
                Servo_Angle2 = 0.0f;
                break;
                case GetWeapon_TurnBack:
                case GetWeapon_Done:
                Servo_Angle1 = 73.0f;
                Servo_Angle2 = 0.0f;
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
    PC_rx_timeout_1ms_process();
    MeasureFSM_Run();
    GetKFS_FSM_Run();
    LiftFSM_Run();
    Check_IO_INPUT();
    GetWeapon_FSM_Run();
    KeyBoard_TIM_1ms_Process_PeriodElapsedCallback();
    // 按键模式选择处理（长按500ms选择比赛项目）
    KeyBoard_Mode_Select_Process();
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
        // if(Robot_Mode == Robot_Mode_Manual){
        // Remote_Valt_OutputControl();}
        Servo_Motor_Control();
        Robot_Calibration_Check();
        Remote_Status_Update(&ch9_status, 9);
    }

    BSP_Key_TIM_1ms_Process_PeriodElapsedCallback();
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        // 处理按键状态
        BSP_Key_TIM_50ms_Read_PeriodElapsedCallback();
        //处理键盘状态
        KeyBoard_TIM_50ms_Read_PeriodElapsedCallback();
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
    UART_Init(&huart1,TFmini_GetDistanceFront1);
    UART_Init(&huart8,TFmini_GetDistanceDownF);
    UART_Init(&huart9,TFmini_GetDistanceDownB);

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
    BSP_Power_Init(true,true,true);


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
   
    //键盘初始化
    KeyBoard_Init();
   
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
