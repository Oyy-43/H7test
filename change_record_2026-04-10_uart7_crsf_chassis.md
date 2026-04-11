# 2026-04-10 变更记录（UART7/CRSF/底盘逆解）

## 目标
- 修复 UART7 反复报错（ErrorCallback 频发）
- 修复遥控器侧电压遥测收不到
- 调整底盘 4/5 号电机逆解为跟随前排麦轮 X 向分量

## 实际改动文件
- Core/Src/usart.c
- User_Files/1_Middleware/Driver/CRSF/crsf.c
- User_Files/1_Middleware/Driver/UART/drv_uart.c
- User_Files/3_App/Tele_task/tele_task.c
- User_Files/3_App/Chassis/chassis.c

## 关键修改摘要
1) Core/Src/usart.c
- UART7 GPIO 上拉由 GPIO_NOPULL 改为 GPIO_PULLUP，降低空闲线抖动导致的串口错误。

2) User_Files/1_Middleware/Driver/CRSF/crsf.c
- CRSF 发包时补写首字节目标地址：CRSF_ADDRESS_CRSF_RECEIVER。
- crsf_send_buffer 定义统一使用 RAM_D2_BUFFER。

3) User_Files/1_Middleware/Driver/UART/drv_uart.c
- UART_Init 后禁用 RX DMA 半传输中断（DMA_IT_HT），仅保留 IDLE 回调触发。
- HAL_UART_ErrorCallback 中补充 ORE/FE/NE/PE 标志清除、AbortReceive 后 Reinit，并再次关闭 HT 中断。

4) User_Files/3_App/Tele_task/tele_task.c
- 电压采样流程改为：HAL_ADC_Start -> HAL_ADC_PollForConversion -> HAL_ADC_GetValue -> HAL_ADC_Stop。

5) User_Files/3_App/Chassis/chassis.c
- 4/5 号电机速度由全向轮公式改为跟随 0/3 号轮的 X 向分量。
- Chassis_Control 输入缩放参数已调整。
- Chassis_Task 恢复调用 Chassis_Control() 并 1ms 周期执行。

## 之后如何快速改回
### 方式A：按文件回滚（推荐）
在仓库根目录执行：

git restore Core/Src/usart.c

git restore User_Files/1_Middleware/Driver/CRSF/crsf.c

git restore User_Files/1_Middleware/Driver/UART/drv_uart.c

git restore User_Files/3_App/Tele_task/tele_task.c

git restore User_Files/3_App/Chassis/chassis.c

### 方式B：一次性回滚上述5个文件
在仓库根目录执行：

git restore Core/Src/usart.c User_Files/1_Middleware/Driver/CRSF/crsf.c User_Files/1_Middleware/Driver/UART/drv_uart.c User_Files/3_App/Tele_task/tele_task.c User_Files/3_App/Chassis/chassis.c

## 备注
- 当前工程还存在与本次改动无直接关系的 FreeRTOS 链接问题（osDelay/osThreadNew 未解析），回滚上述文件不会自动修复该问题。
