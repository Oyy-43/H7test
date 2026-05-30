#ifndef __TELE_TASK_H
#define __TELE_TASK_H
/* Includes ------------------------------------------------------------------*/
#include "crsf.h"
#include "cmsis_os.h"
#include "drv_uart.h"
#include "robot_def.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum 
{
    CH_Status_FREE = 0,
    CH_Status_TRIG_FREE_PRESSED,
    CH_Status_TRIG_PRESSED_FREE,
    CH_Status_PRESSED,
}Enum_CH_Status;

typedef enum 
{
    BSP_Channel_Zero = 0,
    BSP_Channel_Minus,
    BSP_Channel_Plus,
}Enum_channel_status;

typedef struct
{
    Enum_channel_status Pre_GPIO_State;
    Enum_channel_status Now_GPIO_State;
    Enum_CH_Status Key_Status;
}CH_tatus_S;
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern CH_tatus_S ch9_status;

/* Exported function declarations --------------------------------------------*/
float get_vbat_voltage(void);
void Remote_Status_Update(CH_tatus_S *ch_status, uint8_t ch_index);

#endif /* __TELE_TASK_H */
