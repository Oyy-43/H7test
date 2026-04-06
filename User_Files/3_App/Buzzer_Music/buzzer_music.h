#ifndef __BUZZER_MUSIC_H
#define __BUZZER_MUSIC_H

#include <stdbool.h>

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




#endif /* __BUZZER_MUSIC_H */