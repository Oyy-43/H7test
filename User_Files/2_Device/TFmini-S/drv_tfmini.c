/**
 * @file drv_tfmini.c
 * @author Oyyp
 * @brief TFmini激光测距模块的驱动
 * @brief description
 * @version 0.1
 * @date 2026-04-20 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_tfmini.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
TFminiRxData_S TFmini_RxData[4];

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
uint8_t CheckSum(uint8_t *buf)
{
    uint8_t i;
    uint8_t checksum = 0;
    for (i = 0; i < 8; i++)
    {
        checksum += buf[i];
    }
    return checksum;
}

void TFmini_DataProcess(uint8_t *Buffer, uint16_t Length,TFminiRxData_S *tfmini_S)
{
    if (Length >= 9 &&Buffer[0] == 0x59 && Buffer[1] == 0x59)
    {
        if (CheckSum(Buffer) == Buffer[8])
        {   
            tfmini_S->Last_Distance = tfmini_S->Distance;
            tfmini_S->Frame_Haead1 = Buffer[0];
            tfmini_S->Frame_Haead2 = Buffer[1];
            tfmini_S->Distance = GET16(&Buffer[2]);
            tfmini_S->Strength = GET16(&Buffer[4]);
            tfmini_S->Temperature = GET16(&Buffer[6]);
        }
    }
}

void TFmini_GetDistanceFront1(uint8_t *Buffer, uint16_t Length)
{
    TFmini_DataProcess(Buffer, Length, &TFmini_RxData[0]);
}

void TFmini_GetDistanceDownF(uint8_t *Buffer, uint16_t Length)
{
    TFmini_DataProcess(Buffer, Length, &TFmini_RxData[1]);
}

void TFmini_GetDistanceDownB(uint8_t *Buffer, uint16_t Length)
{
    TFmini_DataProcess(Buffer, Length, &TFmini_RxData[2]);
}
/* Function prototypes -------------------------------------------------------*/

