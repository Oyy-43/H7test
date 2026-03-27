#include "buzzer_music.h"

#include "bsp_buzzer.h"
#include "cmsis_os2.h"
#include "bsp_uart.h"
#include <stdbool.h>

BoardStuatus_e BoardStuatus;
bool Last_InitFlag=false;

void Init_Check(void)
{
   if(Last_InitFlag==false && init_finished==true)
   {
       if(BoardStuatus==unInit)
       {
           BoardStuatus=Inited;
       }
   }
    Last_InitFlag=init_finished;
}

void Music_Scan(void)
{
    Init_Check();
    switch (BoardStuatus)
    {
    case unInit:
    Buzzer_Set_Sound(0.0f, 0.0f); // 不响
        break;
    case Inited:
    Buzzer_Set_Sound(BUZZER_FREQUENCY_C5, 1.0f); // C5音符, 中等响度
    osDelay(250);
    Buzzer_Set_Sound(BUZZER_FREQUENCY_D5, 1.0f); // D5音符, 中等响度
    osDelay(250);
    Buzzer_Set_Sound(BUZZER_FREQUENCY_G5, 1.0f); // G5音符, 中等响度
    osDelay(500);
    BoardStuatus = working;
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