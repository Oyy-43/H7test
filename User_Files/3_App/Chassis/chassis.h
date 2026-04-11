#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crsf.h"

/* Exported macros -----------------------------------------------------------*/
//0号电机的坐标（a,b)
#define chassis_a (0.2842f) //电机0到中心的y向距离  竖向
#define chassis_b (0.3418f) //电机0到中心的x向距离  横向
#define chassis_d (0.152f)   //轮子直径

// 第4、5号全向轮参数（位于0-1中点与2-3中点）
#define chassis_omni_b (chassis_b) //全向轮到中心的y向距离
#define chassis_omni_d (0.152f)    //全向轮直径


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void Chassis_Omega_update(float vx, float vy, float vz);

#endif /* __CHASSIS_H */