#ifndef __BUZZER_MUSIC_H
#define __BUZZER_MUSIC_H
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "bsp_buzzer.h"
#include "cmsis_os2.h"
#include "drv_uart.h"
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

typedef enum
{
    unInit=0,
    Inited,
    working,
}BoardStatus_e;

typedef enum
{
    BUZZER_DELAY_OS = 0,
    BUZZER_DELAY_HAL,
} BuzzerDelayMode_e;

bool Buzzer_Play_RTTTL(const char *rtttl, float loudness, BuzzerDelayMode_e delay_mode);
void Buzzer_Play_Once_NonBlocking(float frequency, float loudness, uint32_t duration_ms);




#endif /* __BUZZER_MUSIC_H */