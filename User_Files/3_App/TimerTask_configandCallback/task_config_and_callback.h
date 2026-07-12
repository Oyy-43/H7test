#ifndef TASK_CONFIG_AND_CALLBACK_H
#define TASK_CONFIG_AND_CALLBACK_H

#include "main.h"
#include "sys_timestamp.h"
#include "drv_uart.h"
#include "cmsis_os2.h"
#include "bsp_buzzer.h"
#include "bsp_ws2812.h"
#include "drv_spi.h"
#include "bsp_key.h"
#include "drv_can.h"
#include "drv_motor_dm.h"
#include "ctrl_motor_dm.h"
#include "bsp_power.h"
#include "drv_usb.h"
#include "drv_motor_lk.h"
#include "ctrl_motor_lk.h"
#include "withPC.h"
#include "drv_motor_dji.h"
#include "ctrl_motor_dji.h"
#include "tele_task.h"
#include "lift.h"
#include "drv_motor_dji.h"
#include "ctrl_motor_dji.h"
#include "drv_motor_dm.h"
#include "ctrl_motor_dm.h"
#include "robot_def.h"
#include "drv_tfmini.h"
#include "Hl12H1_Ml1_000.h"
#include "drv_servo_motor.h"
#include "arm.h"
#include "buzzer_music.h"

// #define RedTeam
#define BlueTeam

void Task_Init();
void Task1ms_Callback();
void Task3600s_Callback();

extern bool IO_Status[4];
extern uint16_t Banding_Flag;
#endif // TASK_CONFIG_AND_CALLBACK_H