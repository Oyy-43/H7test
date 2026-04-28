#ifndef _ALG_FSM_H
#define _ALG_FSM_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdint.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 有限状态机结构体的事件
 * 
 */
typedef struct Event 
{
    uint16_t sig; //sginal
}Event;

/**
 * @brief 带参数的事件
 * 
 */
typedef struct Eventwithparam 
{
    Event super; //base event
    uint16_t param; //parameter
}Eventwp;

/**
 * @brief 有限状态机结构体
 * 
 */
typedef struct 
{
    uint8_t state;
    float state_time;
}FSMstate;


/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void FSM_Init(FSMstate *const me);






#endif /* __ALG_FSM_H */