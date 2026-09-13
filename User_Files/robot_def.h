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
 * @brief 机器人比赛项目枚举定义
 *        通过按键选择确定当前比赛项目
 */
typedef enum Enum_Competition_Mode
{
    Competition_Mode_None = 0,            // 未选择
    Competition_Mode_1,                   // 2区梅林重试
    Competition_Mode_Single_3Zone,        // 2次按键  -> 单项赛上3区
    Competition_Mode_Battle_3ZoneLeft,        // 3次按键  -> 对抗赛3区放左边代码
    Competition_Mode_Battle_3ZoneMid,         // 4次按键  -> 对抗赛3区放中间代码
    Competition_Mode_Battle_3ZoneRight        // 5次按键  -> 对抗赛3区放右边代码
}Enum_Competition_Mode;

extern Enum_Competition_Mode Competition_Mode;

/**
 * @brief 按键模式选择状态机枚举
 */
typedef enum
{
    Key_Select_State_Idle = 0,       // 空闲状态
    Key_Select_State_Entering,       // 长按进入中（等待松手确认）
    Key_Select_State_Counting,       // 计数短按中
    Key_Select_State_Exiting,        // 长按退出中（等待松手确认）
}Enum_Key_Select_State;

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