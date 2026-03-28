/* Includes ------------------------------------------------------------------*/
#include "bsp_ws2812.h"
#include "string.h"


/* Private variables ---------------------------------------------------------*/
// 纯黑
const Struct_WS2812_Color WS2812_COLOR_BLACK = {0, 0, 0};
// 纯白
const Struct_WS2812_Color WS2812_COLOR_WHITE = {255, 255, 255};

// 纯红
const Struct_WS2812_Color WS2812_COLOR_RED = {255, 0, 0};
// 纯绿
const Struct_WS2812_Color WS2812_COLOR_GREEN = {0, 255, 0};
// 纯蓝
const Struct_WS2812_Color WS2812_COLOR_BLUE = {0, 0, 255};

// 黄色
const Struct_WS2812_Color WS2812_COLOR_YELLOW = {255, 255, 0};
// 青色
const Struct_WS2812_Color WS2812_COLOR_CYAN = {0, 255, 255};
// 品红
const Struct_WS2812_Color WS2812_COLOR_MAGENTA = {255, 0, 255};

Struct_WS2812_Config BSP_WS2812;


void WS2812_Init(const uint8_t __Red, const uint8_t __Green, const uint8_t __Blue)
{
    BSP_WS2812.SPI_Manage_Object = &SPI6_Manage_Object;

    BSP_WS2812.Color.Red = __Red;
    BSP_WS2812.Color.Green = __Green;
    BSP_WS2812.Color.Blue = __Blue;
}

void TIM_10ms_Write_PeriodElapsedCallback()
{
    uint8_t *tmp_buffer = BSP_WS2812.SPI_Manage_Object->Tx_Buffer;

    memset(tmp_buffer, 0, SPI_BUFFER_SIZE);
    for (uint8_t i = 0; i < 8; i++)
    {
        tmp_buffer[7 - i] = (BSP_WS2812.Color.Green & (1 << i)) ? LEVEL_1 : LEVEL_0;
        tmp_buffer[15 - i] = (BSP_WS2812.Color.Red & (1 << i)) ? LEVEL_1 : LEVEL_0;
        tmp_buffer[23 - i] = (BSP_WS2812.Color.Blue & (1 << i)) ? LEVEL_1 : LEVEL_0;
    }

    SPI_Transmit_Data(BSP_WS2812.SPI_Manage_Object->SPI_Handler,NULL, 0, GPIO_PIN_SET, 25);
}
