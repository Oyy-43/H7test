/**
 * @file crsf.c
 * @author Zomfly
 * @brief 伟大的中南爷写的CRSF协议
 * @version 0.1
 * @date 2026-04-05 0.1 init
 *
 * @copyright Copyright
 *
 */

 /* Includes ------------------------------------------------------------------*/
 #include "crsf.h"
 

 /* Private macros ------------------------------------------------------------*/
 
 /* Private types -------------------------------------------------------------*/
__attribute__((section(".ram_d2"), aligned(32))) uint8_t crsf_send_buffer[64] = {CRSF_ADDRESS_FLIGHT_CONTROLLER};

crsf_boardcast_frame_t crsf_frame = {0};

crsf_channels_t rc_channels;
 
 /* Private variables ---------------------------------------------------------*/
 
 /* Private function declarations ---------------------------------------------*/
 __STATIC_INLINE uint8_t crsf_calculate_crc(const uint8_t * ptr, const uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}

__STATIC_INLINE uint8_t crsf_rx_msg_check(const uint8_t * msg, const uint8_t len)
{
    if (len < 4 || len > 64)
        return 0;
    if (msg[1] != len - 2)
        return 0;
    if (crsf_calculate_crc(msg + 2, len - 3) != msg[len - 1])
        return 0;
    return 1;
}

void crsf_rx_idle_callback(uint8_t *buf, uint16_t length)
{
    if (!crsf_rx_msg_check(buf, length))
    {
        return;
    }
    
    crsf_frame.addr = buf[0];
    crsf_frame.length = buf[1];
    crsf_frame.type = buf[2];
    memcpy(crsf_frame.payload, &buf[3], crsf_frame.length - 2);
    crsf_frame.crc = buf[length - 1];

    switch (crsf_frame.addr)
    {
    case CRSF_ADDRESS_FLIGHT_CONTROLLER:
        switch (crsf_frame.type)
        {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            // 16ch 遥控
            crsf_unpack_flight_controller(&crsf_frame);
            break;
        
        default:
            break;
        }
        break;
    
    default:
        break;
    }
}

void __send_crsf_packet(const CRSF_FRAMETYPE_t frame_type, const uint8_t * payload, const uint8_t size_of_payload)
{
    crsf_send_buffer[1] = size_of_payload + 2;
    crsf_send_buffer[2] = frame_type;
    memcpy(&crsf_send_buffer[3], payload, size_of_payload);
    crsf_send_buffer[size_of_payload + (4 - 1)] = crsf_calculate_crc(&crsf_send_buffer[2], size_of_payload + 1);
    UART_Transmit_Data(&CRSF_UART, crsf_send_buffer, size_of_payload + 4);
}

// 发送函数指针
void (*send_crsf_packet)(const CRSF_FRAMETYPE_t, const uint8_t *, const uint8_t) = __send_crsf_packet;

 /* Function prototypes -------------------------------------------------------*/
 
 
