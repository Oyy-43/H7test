#ifndef _ROBOT_DEF_H
#define _ROBOT_DEF_H



/**
 * @brief 机器人手动模式和自动模式的枚举定义
 * 
 */
typedef enum Enum_Robot_Mode
{
    Robot_Mode_Stop=0,
    Robot_Mode_Manual,
    Robot_Mode_Auto,
}Enum_Robot_Mode;

extern Enum_Robot_Mode Robot_Mode;
/**
 * @brief 选择开启C++的宏定义
 * 
 * 
 */

// #define CPP_ENABLE

/**
 * @brief 按字节的地址读取的宏定义
 * 
 */
#define  GET16(ADDR) *((int16_t*)(ADDR))
#define  GETU16(ADDR) *((uint16_t*)(ADDR))
#define  GET32(ADDR) *((int32_t*)(ADDR))
#define  GETU32(ADDR) *((uint32_t*)(ADDR))

#endif