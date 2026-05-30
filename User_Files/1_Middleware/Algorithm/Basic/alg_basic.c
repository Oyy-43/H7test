/**
 * @file alg_basic.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些极其简易的数学
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-10 1.1 修改成cpp
 * @date 2025-09-23 2.1 引入NaN判断
 *
 * @copyright Copyright (c) 2023-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include <stddef.h>
#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// rpm换算到rad/s
const float BASIC_MATH_RPM_TO_RADPS = 2.0f * PI / 60.0f;
// deg换算到rad
const float BASIC_MATH_DEG_TO_RAD = PI / 180.0f;
// 摄氏度换算到开氏度
const float BASIC_MATH_CELSIUS_TO_KELVIN = 273.15f;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 布尔值反转
 *
 * @param Value 布尔值地址
 */
void Basic_Math_Boolean_Logical_Not(bool *Value)
{
    if (!*Value)
    {
        *Value = true;
    }
    else if (*Value)
    {
        *Value = false;
    }
}

/**
 * @brief 16位大小端转换
 *
 * @param Source 源数据地址
 * @param Destination 目标存储地址
 * @return uint16_t 结果
 */
uint16_t Basic_Math_Endian_Reverse_16(void *Source, void *Destination)
{
    uint8_t *tmp_address_8;
    uint16_t tmp_value_16;
    tmp_address_8 = (uint8_t *) Source;
    tmp_value_16 = tmp_address_8[0] << 8 | tmp_address_8[1];

    if (Destination != NULL)
    {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t *) Source;
        tmp_destination = (uint8_t *) Destination;
        tmp_destination[0] = tmp_source[1];
        tmp_destination[1] = tmp_source[0];
    }

    return (tmp_value_16);
}

/**
 * @brief 32位大小端转换
 *
 * @param Source 源数据地址
 * @param Destination 目标存储地址
 * @return uint32_t 结果
 */
uint32_t Basic_Math_Endian_Reverse_32(void *Source, void *Destination)
{
    uint8_t *tmp_address_8;
    uint32_t tmp_value_32;
    tmp_address_8 = (uint8_t *) Source;
    tmp_value_32 = tmp_address_8[0] << 24 | tmp_address_8[1] << 16 | tmp_address_8[2] << 8 | tmp_address_8[3];

    if (Destination != NULL)
    {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t *) Source;
        tmp_destination = (uint8_t *) Destination;
        tmp_destination[0] = tmp_source[3];
        tmp_destination[1] = tmp_source[2];
        tmp_destination[2] = tmp_source[1];
        tmp_destination[3] = tmp_source[0];
    }

    return (tmp_value_32);
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint8_t 结果
 */
uint8_t Basic_Math_Sum_8(const uint8_t *Address, uint32_t Length)
{
    uint8_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint16_t 结果
 */
uint16_t Basic_Math_Sum_16(const uint16_t *Address, uint32_t Length)
{
    uint16_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint32_t 结果
 */
uint32_t Basic_Math_Sum_32(const uint32_t *Address, uint32_t Length)
{
    uint32_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief sinc函数的实现
 *
 * @param x 输入
 * @return float 输出
 */
float Basic_Math_Sinc(float x)
{
    // 分母为0则按极限求法
    if (Basic_Math_Abs(x) <= 2.0f * FLT_EPSILON)
    {
        return (1.0f);
    }

    return (sinf(x) / x);
}

/**
 * @brief 将浮点数映射到整型
 *
 * @param x 浮点数
 * @param Float_1 浮点数1
 * @param Float_2 浮点数2
 * @param Int_1 整型1
 * @param Int_2 整型2
 * @return int32_t 整型
 */
int32_t Basic_Math_Float_To_Int(float x, float Float_1, float Float_2, int32_t Int_1, int32_t Int_2)
{
    float tmp = (x - Float_1) / (Float_2 - Float_1);
    int32_t out = (int32_t)(tmp * (float) (Int_2 - Int_1) + (float) (Int_1));
    return (out);
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param Int_1 整型1
 * @param Int_2 整型2
 * @param Float_1 浮点数1
 * @param Float_2 浮点数2
 * @return float 浮点数
 */
float Basic_Math_Int_To_Float(int32_t x, int32_t Int_1, int32_t Int_2, float Float_1, float Float_2)
{
    float tmp = (float) (x - Int_1) / (float) (Int_2 - Int_1);
    float out = tmp * (Float_2 - Float_1) + Float_1;
    return (out);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief 判断浮点数是否为无效浮点数
 *
 * @param x 浮点数
 * @return 是否为NaN
 */
bool Basic_Math_Is_Invalid_Float(float x)
{
    uint32_t exp = (*(uint32_t *) (&x) >> 23) & 0xff;
    uint32_t frac = (*(uint32_t *) (&x) & 0x7fffff);
    if (exp == 0x00)
    {
        if (frac == 0x00)
        {
            // 正负零
            return (false);
        }
        // 次正规数和非正规数
        return (true);
    }
    if (exp == 0xFF)
    {
        // 无穷大和NaN
        return (true);
    }
    // 正规数
    return (false);
}

/**
 * @brief 求取模归化
 *
 * @param x 传入数据
 * @param modulus 模数
 * @return 返回的归化数, 介于 ±modulus / 2 之间
 */
float Basic_Math_Modulus_Normalization(float x, float modulus)
{
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f)
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}

void Basic_Math_Modulus_Init(Normali_S *config,int32_t x,int32_t y)
{
    if (config == NULL)
        return;

    if (x >= y)
    {
        config->Max = x;
        config->Min = y;
    }
    else
    {
        config->Max = y;
        config->Min = x;
    }
}

float Basic_Math_Modulus_Return(Normali_S *config,int32_t now)
{
    if (config == NULL)
        return (0.0f);

    /* 避免除以0 */
    if (config->Max == config->Min)
        return (0.0f);

    float span = (float)(config->Max - config->Min);
    float pos = ((float)(now - config->Min)) / span;

    /* 将结果限制在 [0,1] 范围内 */
    if (pos < 0.0f)
        pos = 0.0f;
    else if (pos > 1.0f)
        pos = 1.0f;

    return (pos);
}

int32_t Basic_Math_Number_Return(Normali_S *config,float part)
{
    if (config == NULL)
        return (0);

    /* 将 part 限制在 [0,1] 范围内 */
    if (part < 0.0f)
        part = 0.0f;
    else if (part > 1.0f)
        part = 1.0f;

    float span = (float)(config->Max - config->Min);
    int32_t result = config->Min + (int32_t)(part * span);

    return (result);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/