#include "buzzer_music.h"

#include "bsp_buzzer.h"
#include "cmsis_os2.h"
#include "drv_uart.h"
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static uint32_t parse_u32(const char **p)
{
    uint32_t value = 0;

    while (isdigit((unsigned char)**p))
    {
        value = value * 10U + (uint32_t)(**p - '0');
        (*p)++;
    }

    return value;
}

static int8_t note_to_semitone(char note)
{
    switch (note)
    {
    case 'c': return 0;
    case 'd': return 2;
    case 'e': return 4;
    case 'f': return 5;
    case 'g': return 7;
    case 'a': return 9;
    case 'b': return 11;
    default: return -1;
    }
}

static void buzzer_delay_ms(uint32_t ms, BuzzerDelayMode_e delay_mode)
{
    if (ms == 0U)
    {
        return;
    }

    if (delay_mode == BUZZER_DELAY_HAL)
    {
        HAL_Delay(ms);
    }
    else
    {
        osDelay(ms);
    }
}

bool Buzzer_Play_RTTTL(const char *rtttl, float loudness, BuzzerDelayMode_e delay_mode)
{
    uint32_t default_duration = 4U;
    uint32_t default_octave = 6U;
    uint32_t bpm = 63U;
    uint32_t wholenote_ms = 0U;
    float clamped_loudness = loudness;
    const char *defaults_start = NULL;
    const char *notes_start = NULL;
    const char *p = NULL;

    if (rtttl == NULL)
    {
        return false;
    }

    if (clamped_loudness < 0.0f)
    {
        clamped_loudness = 0.0f;
    }
    if (clamped_loudness > 1.0f)
    {
        clamped_loudness = 1.0f;
    }

    p = strchr(rtttl, ':');
    if (p == NULL)
    {
        return false;
    }
    defaults_start = p + 1;

    p = strchr(defaults_start, ':');
    if (p == NULL)
    {
        notes_start = defaults_start;
    }
    else
    {
        const char *d = defaults_start;
        notes_start = p + 1;

        while (d < p)
        {
            while (d < p && (isspace((unsigned char)*d) || *d == ','))
            {
                d++;
            }

            if (d >= p)
            {
                break;
            }

            if (tolower((unsigned char)*d) == 'd')
            {
                d++;
                if (d < p && *d == '=')
                {
                    uint32_t v;
                    d++;
                    v = parse_u32(&d);
                    if (v != 0U)
                    {
                        default_duration = v;
                    }
                }
            }
            else if (tolower((unsigned char)*d) == 'o')
            {
                d++;
                if (d < p && *d == '=')
                {
                    uint32_t v;
                    d++;
                    v = parse_u32(&d);
                    if (v <= 8U)
                    {
                        default_octave = v;
                    }
                }
            }
            else if (tolower((unsigned char)*d) == 'b')
            {
                d++;
                if (d < p && *d == '=')
                {
                    uint32_t v;
                    d++;
                    v = parse_u32(&d);
                    if (v != 0U)
                    {
                        bpm = v;
                    }
                }
            }
            else
            {
                d++;
            }
        }
    }

    if (bpm == 0U)
    {
        bpm = 63U;
    }
    wholenote_ms = (60U * 1000U * 4U) / bpm;

    p = notes_start;
    while (*p != '\0')
    {
        uint32_t duration_div = 0U;
        uint32_t octave = default_octave;
        uint32_t duration_ms = 0U;
        uint32_t on_time_ms = 0U;
        bool dotted = false;
        bool pause = false;
        char note_ch;
        int8_t semitone;

        while (*p == ' ' || *p == ',' || *p == '\t' || *p == '\r' || *p == '\n')
        {
            p++;
        }

        if (*p == '\0')
        {
            break;
        }

        duration_div = parse_u32(&p);
        if (duration_div == 0U)
        {
            duration_div = default_duration;
        }

        note_ch = (char)tolower((unsigned char)*p);
        if (note_ch == '\0')
        {
            break;
        }
        p++;

        if (note_ch == 'p')
        {
            pause = true;
            semitone = -1;
        }
        else
        {
            semitone = note_to_semitone(note_ch);
            if (semitone < 0)
            {
                while (*p != '\0' && *p != ',')
                {
                    p++;
                }
                continue;
            }
        }

        if (!pause && *p == '#')
        {
            semitone++;
            p++;
        }

        if (isdigit((unsigned char)*p))
        {
            octave = parse_u32(&p);
            if (octave > 8U)
            {
                octave = 8U;
            }
        }

        if (*p == '.')
        {
            dotted = true;
            p++;
        }

        duration_ms = wholenote_ms / duration_div;
        if (dotted)
        {
            duration_ms += duration_ms / 2U;
        }
        if (duration_ms == 0U)
        {
            duration_ms = 1U;
        }

        if (pause)
        {
            Buzzer_Set_Sound(0.0f, 0.0f);
            buzzer_delay_ms(duration_ms, delay_mode);
        }
        else
        {
            uint32_t midi = (octave + 1U) * 12U + (uint32_t)semitone;
            float frequency = 440.0f * powf(2.0f, ((float)midi - 69.0f) / 12.0f);

            on_time_ms = (duration_ms * 9U) / 10U;
            if (on_time_ms == 0U)
            {
                on_time_ms = 1U;
            }

            Buzzer_Set_Sound(frequency, clamped_loudness);
            buzzer_delay_ms(on_time_ms, delay_mode);
            Buzzer_Set_Sound(0.0f, 0.0f);
            buzzer_delay_ms(duration_ms - on_time_ms, delay_mode);
        }

        if (*p == ',')
        {
            p++;
        }
    }

    Buzzer_Set_Sound(0.0f, 0.0f);
    return true;
}

BoardStatus_e BoardStatus;
bool Last_InitFlag=false;

void Init_Check(void)
{
   if(Last_InitFlag==false && init_finished==true)
   {
       if(BoardStatus==unInit)
       {
           BoardStatus=Inited;
       }
   }
    Last_InitFlag=init_finished;
}

void Music_Scan(void)
{
    Init_Check();
    switch (BoardStatus)
    {
    case unInit:
    Buzzer_Set_Sound(0.0f, 0.0f); // 不响
    break;
    case Inited:
    Buzzer_Set_Sound(Init_Music[0], 1.0f); // C5音符
    osDelay(250);
    Buzzer_Set_Sound(Init_Music[1], 1.0f); // D5音符
    osDelay(250);
    Buzzer_Set_Sound(Init_Music[2], 1.0f); // G5音符
    osDelay(500);
    BoardStatus = working;
        break;
    case working:
    Buzzer_Set_Sound(0.0f, 0.0f); // 暂时不响
        break;
    default:
        break;
    }
}


void MusicTask_func(void *argument)
{
    for(;;)
    {
       Music_Scan();
       osDelay(1);
    }
}