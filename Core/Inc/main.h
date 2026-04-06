/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DMA_RAM_SECTION ".RAM_D2_Section"
#if defined(DMA_RAM_SECTION)
#define RAM_D2_BUFFER __attribute__((section(DMA_RAM_SECTION), aligned(32)))
#else
#define RAM_D2_BUFFER
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DC24_1__OUTPUT_Pin GPIO_PIN_13
#define DC24_1__OUTPUT_GPIO_Port GPIOC
#define DC24_0__OUTPUT_Pin GPIO_PIN_14
#define DC24_0__OUTPUT_GPIO_Port GPIOC
#define DC5__OUTPUT_Pin GPIO_PIN_15
#define DC5__OUTPUT_GPIO_Port GPIOC
#define BMI088_RX__SPI_MOSI_Pin GPIO_PIN_1
#define BMI088_RX__SPI_MOSI_GPIO_Port GPIOC
#define BMI088_TX__SPI_MISO_Pin GPIO_PIN_2
#define BMI088_TX__SPI_MISO_GPIO_Port GPIOC
#define WS2812_RX__SPI_MOSI_Pin GPIO_PIN_7
#define WS2812_RX__SPI_MOSI_GPIO_Port GPIOA
#define BMI088_CLK__SPI_CLK_Pin GPIO_PIN_13
#define BMI088_CLK__SPI_CLK_GPIO_Port GPIOB
#define KEY__INPUT_Pin GPIO_PIN_15
#define KEY__INPUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
