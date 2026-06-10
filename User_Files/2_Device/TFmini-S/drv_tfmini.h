#ifndef _DRV_TFMINI_H
#define _DRV_TFMINI_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "robot_def.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint8_t Frame_Haead1;  // 固定为0x59
    uint8_t Frame_Haead2;  // 固定为0x59
    uint16_t Distance;      // 的值范围为0-1200。实际使用过程中，当信号强度值Strength<100或等于65535（信号过曝)时,Dist 的测量值被认为不可信，默认输出-1
    uint16_t Strength;      //默认输出值会在0-65535之间。当测距档位一定时，测距越远，信号强度越低；目标物反射率越低，信号强度越低
    uint16_t Temperature;     // 温度，摄氏度 =Temp/8-256
}TFminiRxData_S;


/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern TFminiRxData_S TFmini_RxData[3];

/* Exported function declarations --------------------------------------------*/
void TFmini_DataProcess(uint8_t *Buffer, uint16_t Length, TFminiRxData_S *tfmini_S);
void TFmini_GetDistanceFront(uint8_t *Buffer, uint16_t Length);
void TFmini_GetDistanceDown(uint8_t *Buffer, uint16_t Length);

#endif // _DRV_TFMINI_H