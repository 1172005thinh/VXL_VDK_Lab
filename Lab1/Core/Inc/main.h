/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define _1_Pin GPIO_PIN_1
#define _1_GPIO_Port GPIOA
#define _2_Pin GPIO_PIN_2
#define _2_GPIO_Port GPIOA
#define _3_Pin GPIO_PIN_3
#define _3_GPIO_Port GPIOA
#define _4_Pin GPIO_PIN_4
#define _4_GPIO_Port GPIOA
#define _5_Pin GPIO_PIN_5
#define _5_GPIO_Port GPIOA
#define _6_Pin GPIO_PIN_6
#define _6_GPIO_Port GPIOA
#define _7_Pin GPIO_PIN_7
#define _7_GPIO_Port GPIOA
#define _8_Pin GPIO_PIN_8
#define _8_GPIO_Port GPIOA
#define _9_Pin GPIO_PIN_9
#define _9_GPIO_Port GPIOA
#define _10_Pin GPIO_PIN_10
#define _10_GPIO_Port GPIOA
#define _11_Pin GPIO_PIN_11
#define _11_GPIO_Port GPIOA
#define _12_Pin GPIO_PIN_12
#define _12_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
