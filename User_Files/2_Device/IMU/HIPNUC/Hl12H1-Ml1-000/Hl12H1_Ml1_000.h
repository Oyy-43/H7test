#ifndef _HL12H1_ML1_000_H_
#define _HL12H1_ML1_000_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "drv_uart.h"
#include "cmsis_os2.h"        
#include "usart.h" 

/* Exported macros -----------------------------------------------------------*/
#define HIPNUC_UART &huart10
#define offest 6 // 帧头数据偏移
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
#define I2(p) (*((int16_t *)(p)))

static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;};
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;};
static int32_t I4(uint8_t *p) {int32_t u; memcpy(&u,p,4); return u;};
static float R4(uint8_t *p) {float r; memcpy(&r,p,4); return r;};
/* Exported types ------------------------------------------------------------*/
typedef struct
{
uint8_t tag;        /* Item tag: 0x91 */
float acc[3];       /* Acceleration */
float gyr[3];       /* Angular velocity */
float mag[3];       /* Magnetic field */
float eul[3];       /* Attitude: Euler angle */
float quat[4];      /* Attitude: quaternion */
float pressure;     /* Air pressure */
uint32_t timestamp; /* Timestamp */
float Begin_Yaw;    /* 初始偏航角, 用于重置欧拉角 */
}hipnuc_imu_data_t;

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern hipnuc_imu_data_t hipnuc_imu_data;

/* Exported function declarations --------------------------------------------*/
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t
lengthInBytes);
void hipnuc_data_unpacked(uint8_t *buf, uint16_t length);
void HIPNUC_uart_init(void);
const hipnuc_imu_data_t *get_hipnuc_imu_point(void);
void Reset_euler_angle(void);
void HIPNUC_YAW_init(void);

#endif /* _HL12H1_ML1_000_H_ */