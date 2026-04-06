#ifndef __TELE_TASK_H
#define __TELE_TASK_H
/* Includes ------------------------------------------------------------------*/
#include "crsf.h"
#include "cmsis_os.h"
#include "drv_uart.h"
#include "robot_def.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
float get_vbat_voltage(void);

#endif /* __TELE_TASK_H */
