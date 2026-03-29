#ifndef __BSP_WS2812_H
#define __BSP_WS2812_H

#include "main.h"
#include "spi.h"
#include "drv_spi.h"

// 灯珠的逻辑码
static const uint8_t LEVEL_0 = 0x60U;
static const uint8_t LEVEL_1 = 0x78U;

typedef struct
{
    uint8_t Red;
    uint8_t Green;
    uint8_t Blue;
}Struct_WS2812_Color;

typedef struct
{
    Struct_SPI_Manage_Object *SPI_Manage_Object;
    Struct_WS2812_Color Color;
}Struct_WS2812_Config;

/* Exported variables --------------------------------------------------------*/

extern const Struct_WS2812_Color WS2812_COLOR_BLACK;
extern const Struct_WS2812_Color WS2812_COLOR_WHITE;
extern const Struct_WS2812_Color WS2812_COLOR_RED;
extern const Struct_WS2812_Color WS2812_COLOR_GREEN;
extern const Struct_WS2812_Color WS2812_COLOR_BLUE;
extern const Struct_WS2812_Color WS2812_COLOR_YELLOW;
extern const Struct_WS2812_Color WS2812_COLOR_CYAN;
extern const Struct_WS2812_Color WS2812_COLOR_MAGENTA;

extern Struct_WS2812_Config BSP_WS2812;

/* Exported function declarations --------------------------------------------*/

void WS2812_Init(uint8_t red, uint8_t green, uint8_t blue);
void WS2812_Set_Red(uint8_t red);
void WS2812_Set_Green(uint8_t green);
void WS2812_Set_Blue(uint8_t blue);
void WS2812_Set_RGB(uint8_t red, uint8_t green, uint8_t blue);
void WS2812_Set_Color(Struct_WS2812_Color color, const float brightness);
void TIM_10ms_Write_PeriodElapsedCallback(void);



#endif // __BSP_WS2812_H