#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crsf.h"

/* Exported macros -----------------------------------------------------------*/
#define chassis_a (0.2015f) //电机0到中心的y向距离  竖向
#define chassis_b (0.2105f) //电机0到中心的x向距离  横向
#define chassis_d (0.154f)   //轮子直径

// 第4、5号全向轮参数（位于0-1中点与2-3中点）
#define chassis_omni_b (chassis_b) //全向轮到中心的y向距离
#define chassis_omni_d (0.154f)    //全向轮直径

// 电机正方向修正系数，安装方向相反时改为(-1.0f)
#define chassis_motor0_dir (1.0f)
#define chassis_motor1_dir (1.0f)
#define chassis_motor2_dir (1.0f)
#define chassis_motor3_dir (1.0f)
#define chassis_motor4_dir (1.0f)
#define chassis_motor5_dir (1.0f)

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void Chassis_Omegaupdate(float vx, float vy, float vz);

#endif /* __CHASSIS_H */