#ifndef _ROBOT_DEF_H
#define _ROBOT_DEF_H

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