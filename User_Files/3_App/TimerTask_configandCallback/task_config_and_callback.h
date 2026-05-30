#ifndef TASK_CONFIG_AND_CALLBACK_H
#define TASK_CONFIG_AND_CALLBACK_H

#include "main.h"
#include "lift.h"
#include "drv_motor_dji.h"
#include "ctrl_motor_dji.h"
#include "drv_motor_dm.h"
#include "ctrl_motor_dm.h"

void Task_Init();
void Task1ms_Callback();
void Task3600s_Callback();


#endif // TASK_CONFIG_AND_CALLBACK_H