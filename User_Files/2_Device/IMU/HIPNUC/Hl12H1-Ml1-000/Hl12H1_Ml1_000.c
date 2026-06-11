/**
 * @file Hl12H1_Ml1_000.c
 * @author oyyp
 * @brief 超核电子Hl12H1_Ml1_000型号IMU的驱动文件
 * @version 0.1
 * @date 2026-06-10 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "Hl12H1_Ml1_000.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
hipnuc_imu_data_t hipnuc_imu_data;
int16_t payload_len;       //数据段长度
static const char hip_cmd1[] = "CONFIG ATT RST 1\r\n";
// static const char hip_cmd2[] = "CONFIG ATT RST 2\r\n";

const hipnuc_imu_data_t *get_hipnuc_imu_point(void)
{
    return &hipnuc_imu_data;
}

/*
currectCrc: previous crc value, set 0 if it's first section
src: source stream data
lengthInBytes: length
*/
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t
lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
            temp ^= 0x1021;
            }
            crc = temp;
        }
    }
*currectCrc = crc;
}

void hipnuc_data_unpacked(uint8_t *buf, uint16_t length)
{
    uint16_t crc;              //crc校验值
    crc = 0;
    if (buf == NULL || length < 4) return;

    // 校验tag
    if (buf[0] != 0x5A || buf[1] != 0xA5) return;

    // 获取数据段长度
    payload_len = U2(&buf[2]);

    // 计算CRC
    crc16_update(&crc, buf, 4);
    crc16_update(&crc, buf + offest, payload_len);

    // 获取接收到的CRC值
    uint16_t received_crc = U2(&buf[4]);

    // 校验CRC
    if (crc != received_crc) return;

    // 解析数据
    hipnuc_imu_data.tag = U1(&buf[0]+offest+0);
    hipnuc_imu_data.pressure = R4(&buf[0]+offest+4);
    hipnuc_imu_data.timestamp = U4(&buf[0]+offest+8);
    for (int i = 0; i < 3; i++)
    {
        hipnuc_imu_data.acc[i] = R4(&buf[0]+offest+12 + i * 4);
        hipnuc_imu_data.gyr[i] = R4(&buf[0]+offest+24 + i * 4);
        hipnuc_imu_data.mag[i] = R4(&buf[0]+offest+36 + i * 4);
        hipnuc_imu_data.eul[i] = R4(&buf[0]+offest+48 + i * 4);
    }
    for (int i = 0; i < 4; i++)
    {
        hipnuc_imu_data.quat[i] = R4(&buf[0]+offest+60 + i * 4);
    }
}

void Reset_euler_angle(void)
{
    HAL_UART_Transmit_DMA(HIPNUC_UART, (uint8_t *)hip_cmd1, sizeof(hip_cmd1) - 1);
}


/* Function prototypes -------------------------------------------------------*/

