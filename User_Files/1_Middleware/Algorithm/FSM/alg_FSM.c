/**
 * @file alg_FSM.c
 * @author Oyyp
 * @brief 自己按网上定义写的有限状态机
 * @version 0.1
 * @date 2026-04-19 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "alg_FSM.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 初始化有限状态机
 * 
 * @param //传入一个有限状态机结构体 
 */
void FSM_Init(FSMstate *const me)
{
     me->state = 0;
     me->state_time = 0.0f;
}

// void FSM_dispatch(void)
// {
//     // 状态机调度逻辑
// }
/* Function prototypes -------------------------------------------------------*/

