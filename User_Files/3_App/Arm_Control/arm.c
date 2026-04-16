/**
 * @file arm.c
 * @author Oyyp
 * @brief description
 * @version 0.1
 * @date 2026-04-12 0.1 init
 *
 * @copyright Copyright
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "arm.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
void Power_Set()
{
    if(rc_channels.ch[12]<=0)
    {
        BSP_Power_Set_DC24_0(false);
    }
    else
    {
        BSP_Power_Set_DC24_0(true);
    }
    if(rc_channels.ch[13]<=0)
    {
        BSP_Power_Set_DC24_1(false);
    }
    else
    {
        BSP_Power_Set_DC24_1(true);
    }
}

void arm_target_update()
{
    if(rc_channels.ch[5]<=0)
    {
        DM_Motor_Instances[0].Target_Angle = 0.0f;
        
    }
    else
    {
        DM_Motor_Instances[0].Target_Angle = rc_channels.ch[10]/200.0f;
    }
}


/* Function prototypes -------------------------------------------------------*/
void Arm_Control(void *argument)
{
    for(;;)
    {
        Power_Set();
        arm_target_update();
        osDelay(1);
    }
}

