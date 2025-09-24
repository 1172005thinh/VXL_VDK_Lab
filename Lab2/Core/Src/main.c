/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef *port;
	uint16_t pin;
} PINMAP;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PA1   pinMap[0]
#define PA2   pinMap[1]
#define PA3   pinMap[2]
#define PA4   pinMap[3]
#define PA5   pinMap[4]
#define PA6   pinMap[5]
#define PA7   pinMap[6]
#define PA8   pinMap[7]
#define PA9   pinMap[8]
#define PA10  pinMap[9]
#define PA11  pinMap[10]
#define PA12  pinMap[11]
#define PA15  pinMap[12] //LED-SYS
#define PB0   pinMap[13]
#define PB1   pinMap[14]
#define PB2   pinMap[15]
#define PB3   pinMap[16]
#define PB4   pinMap[17]
#define PB5   pinMap[18]
#define PB6   pinMap[19]
#define PB7   pinMap[20]
#define PB8   pinMap[21]
#define PB9   pinMap[22]
#define PB10  pinMap[23]
#define PB11  pinMap[24]
#define PB12  pinMap[25]
#define PB13  pinMap[26]
#define PB14  pinMap[27]
#define PB15  pinMap[28]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//DEFINE PINMAP
const PINMAP pinMap[] = {
	//PA1 to PA12
  {_1_GPIO_Port,  _1_Pin},   	// indexPin = 1
  {_2_GPIO_Port,  _2_Pin},  	// indexPin = 2
  {_3_GPIO_Port,  _3_Pin},  	// indexPin = 3
  {_4_GPIO_Port,  _4_Pin},   	// indexPin = 4
  {_5_GPIO_Port,  _5_Pin},   	// indexPin = 5
  {_6_GPIO_Port,  _6_Pin},   	// indexPin = 6
  {_7_GPIO_Port,  _7_Pin},   	// indexPin = 7
  {_8_GPIO_Port,  _8_Pin},   	// indexPin = 8
  {_9_GPIO_Port,  _9_Pin},   	// indexPin = 9
  {_10_GPIO_Port, _10_Pin},  	// indexPin = 10
  {_11_GPIO_Port, _11_Pin},  	// indexPin = 11
  {_12_GPIO_Port, _12_Pin},  	// indexPin = 12
  //PA13 to PA14 are reserved
  //PA15 for LED-SYS
  {_13_GPIO_Port, _13_Pin},  	// indexPin = 13
	//PB0 to PB15
  {_14_GPIO_Port, _14_Pin},  	// indexPin = 14
  {_15_GPIO_Port, _15_Pin},  	// indexPin = 15
  {_16_GPIO_Port, _16_Pin},  	// indexPin = 16
  {_17_GPIO_Port, _17_Pin},  	// indexPin = 17
  {_18_GPIO_Port, _18_Pin},  	// indexPin = 18
  {_19_GPIO_Port, _19_Pin},  	// indexPin = 19
  {_20_GPIO_Port, _20_Pin},  	// indexPin = 20
  {_21_GPIO_Port, _21_Pin},  	// indexPin = 21
  {_22_GPIO_Port, _22_Pin},  	// indexPin = 22
  {_23_GPIO_Port, _23_Pin},  	// indexPin = 23
  {_24_GPIO_Port, _24_Pin},  	// indexPin = 24
  {_25_GPIO_Port, _25_Pin},  	// indexPin = 25
  {_26_GPIO_Port, _26_Pin},  	// indexPin = 26
  {_27_GPIO_Port, _27_Pin},  	// indexPin = 27
  {_28_GPIO_Port, _28_Pin},  	// indexPin = 28
  {_29_GPIO_Port, _29_Pin},  	// indexPin = 29
};
//END OF DEFINE PINMAP

//SEGMENT MAP FOR COMMON ANODE 7-SEGMENT DISPLAY
static const uint8_t segMapAnode[11] = {
  //abcdefg
  0b0000001, //0
  0b1001111, //1
  0b0010010, //2
  0b0000110, //3
  0b1001100, //4
  0b0100100, //5
  0b0100000, //6
  0b0001111, //7
  0b0000000, //8
  0b0000100, //9
  0b1001111  //E - ERROR
};
//END OF SEGMENT MAP

//DEDICATED PINS
static const PINMAP LED_SYS = PA15;

static const PINMAP segMent_1[7] = {
  PB0,
  PB1,
  PB2,
  PB3,
  PB4,
  PB5,
  PB6
};

static const PINMAP segMent_2[7] = {
  PB7,
  PB8,
  PB9,
  PB10,
  PB11,
  PB12,
  PB13
};
//END OF DEDICATED PINS

//VARIABLES

//END OF VARIABLES

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void delay(double sec);
void togglePin(PINMAP indexPin, bool state);
void toggleSeg(PINMAP segment[7], uint8_t map[11], uint8_t number);
void blinkLED_SYS();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(double sec) {
  HAL_Delay(sec * 1000);
}

void togglePin(PINMAP pin, bool state) {
  //safety check
  if (pin.pin < 1 || pin.pin >= (sizeof(pinMap) / sizeof(pinMap[0]) + 1)) {
    return;
  }

  HAL_GPIO_WritePin(pin.port, pin.pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void toggleSeg(PINMAP segment[7], uint8_t map[11], uint8_t number) {
  //safety check
  if (number < 0 || number > 10) {
    number = 10; //E - ERROR
  }

  uint8_t state[7];
  for (int i = 0; i < 7; i++) {
    state[i] = (map[number] >> (6 - i)) & 0x01;
    togglePin(segment[i], state[i]);
  }
}

void blinkLED_SYS() {
  HAL_GPIO_TogglePin(LED_SYS.port, LED_SYS.pin);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, _1_Pin|_2_Pin|_3_Pin|_4_Pin
                          |_5_Pin|_6_Pin|_7_Pin|_8_Pin
                          |_9_Pin|_10_Pin|_11_Pin|_12_Pin
                          |_13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, _14_Pin|_15_Pin|_16_Pin|_24_Pin
                          |_25_Pin|_26_Pin|_27_Pin|_28_Pin
                          |_29_Pin|_17_Pin|_18_Pin|_19_Pin
                          |_20_Pin|_21_Pin|_22_Pin|_23_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : _1_Pin _2_Pin _3_Pin _4_Pin
                           _5_Pin _6_Pin _7_Pin _8_Pin
                           _9_Pin _10_Pin _11_Pin _12_Pin
                           _13_Pin */
  GPIO_InitStruct.Pin = _1_Pin|_2_Pin|_3_Pin|_4_Pin
                          |_5_Pin|_6_Pin|_7_Pin|_8_Pin
                          |_9_Pin|_10_Pin|_11_Pin|_12_Pin
                          |_13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : _14_Pin _15_Pin _16_Pin _24_Pin
                           _25_Pin _26_Pin _27_Pin _28_Pin
                           _29_Pin _17_Pin _18_Pin _19_Pin
                           _20_Pin _21_Pin _22_Pin _23_Pin */
  GPIO_InitStruct.Pin = _14_Pin|_15_Pin|_16_Pin|_24_Pin
                          |_25_Pin|_26_Pin|_27_Pin|_28_Pin
                          |_29_Pin|_17_Pin|_18_Pin|_19_Pin
                          |_20_Pin|_21_Pin|_22_Pin|_23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int counter = 100;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  counter--;
  if (counter <= 0) {
    counter = 100;
    blinkLED_SYS();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
