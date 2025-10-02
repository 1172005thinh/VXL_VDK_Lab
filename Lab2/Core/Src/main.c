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
#include "stm32f103c6.h"
#include "soft_timer.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
const int MAX_LED = 4;
int index_led = 0;
int led_buffer[4] = {1, 2, 3, 4};

// Clock variables
int hour = 23;
int minute = 58;
int second = 58;

// LED Matrix variables (LOW ACTIVE for ROWs)
const int MAX_LED_MATRIX = 8;
int index_led_matrix = 0;
unsigned char matrix_buffer[8] = {
  //ROWx
  //x=
  //76543210
  //A
  0b00000000, 
  0b11111100, 
  0b00010010, 
  0b00010001, 
  0b00010001, 
  0b00010010, 
  0b11111100, 
  0b00000000
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void update7SEG(int index);
void updateClockBuffer();
void updateLEDMatrix(int row);
void setRowData(unsigned char row_data);
void setColumnEnable(int column, int enable);
void shiftMatrixBufferLeft();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void update7SEG(int index){
    togglePin(PA6, 1);
    togglePin(PA7, 1);
    togglePin(PA8, 1);
    togglePin(PA9, 1);
    
    int digitToShow = led_buffer[index];
    
    toggleSeg(segMent_1, segMapAnode, digitToShow);
    
    switch(index) {
        case 0:
            togglePin(PA6, 0);
            break;
        case 1:
            togglePin(PA7, 0);
            break;
        case 2:
            togglePin(PA8, 0);
            break;
        case 3:
            togglePin(PA9, 0);
            break;
        default:
            togglePin(PA6, 1);
            togglePin(PA7, 1);
            togglePin(PA8, 1);
            togglePin(PA9, 1);
            break;
    }
}

void updateClockBuffer() {
    // Extract individual digits from hour and minute
    led_buffer[0] = hour / 10;        // Hours tens digit
    led_buffer[1] = hour % 10;        // Hours units digit  
    led_buffer[2] = minute / 10;      // Minutes tens digit
    led_buffer[3] = minute % 10;      // Minutes units digit
    
    // Handle time increment (increment every second)
    second++;
    if (second >= 60) {
        second = 0;
        minute++;
        if (minute >= 60) {
            minute = 0;
            hour++;
            if (hour >= 24) {
                hour = 0;
            }
        }
    }
}

// LED Matrix Functions
void setRowData(unsigned char row_data) {
    // Set ROW0-ROW7 - LOW ACTIVE
    togglePin(PB8, !((row_data >> 0) & 0x01));      // ROW0
    togglePin(PB9, !((row_data >> 1) & 0x01));      // ROW1
    togglePin(PB10, !((row_data >> 2) & 0x01));     // ROW2
    togglePin(PB11, !((row_data >> 3) & 0x01));     // ROW3
    togglePin(PB12, !((row_data >> 4) & 0x01));     // ROW4
    togglePin(PB13, !((row_data >> 5) & 0x01));     // ROW5
    togglePin(PB14, !((row_data >> 6) & 0x01));     // ROW6
    togglePin(PB15, !((row_data >> 7) & 0x01));     // ROW7
}

void setColumnEnable(int column, int enable) {
    // Set ENM0-ENM7 - HIGH ACTIVE
    switch(column) {
        case 0: togglePin(PA1, !enable); break;     // ENM0
        case 1: togglePin(PA2, !enable); break;     // ENM1
        case 2: togglePin(PA3, !enable); break;     // ENM2
        case 3: togglePin(PA4, !enable); break;     // ENM3
        case 4: togglePin(PA10, !enable); break;    // ENM4
        case 5: togglePin(PA11, !enable); break;    // ENM5
        case 6: togglePin(PA12, !enable); break;    // ENM6
        case 7: togglePin(PB7, !enable); break;     // ENM7
    }
}

void updateLEDMatrix(int index) {
    // Turn off all columns first
    for(int col = 0; col < 8; col++) {
        setColumnEnable(col, 0);
    }

    // Set row data and enable the current column
    if(index < MAX_LED_MATRIX) {
        setRowData(matrix_buffer[index]);
        setColumnEnable(index, 1);
    }
}

void shiftMatrixBufferLeft() {
    unsigned char temp = matrix_buffer[0];
    
    for (int i = 0; i <= MAX_LED_MATRIX - 2; i++) {
        matrix_buffer[i] = matrix_buffer[i + 1];
    }
    
    matrix_buffer[MAX_LED_MATRIX - 1] = temp;
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
  //LED_SYS SETUP
  int durLED_SYS = 100;
  setTimerLED_SYS(durLED_SYS);
  //LED_SYS SETUP
  
  //DIGITAL CLOCK SETUP (Restore 7-segment functionality)
  int multiplexTimer = 5;  // 250ms per display (25 * 10ms = 250ms) (speed up 5x for better visual)
  int colonTimer = 100;     // 1 second for DOT blinking (100 * 10ms = 1000ms)
  int clockTimer = 10;     // 1 second for clock update (100 * 10ms = 1000ms) (speed up 10x for demo)
  int currentDisplay = 0;
  
  // Initialize clock buffer with current time
  updateClockBuffer();  
  setTimer1(multiplexTimer);  // Restore 7-segment multiplexing
  setTimer2(colonTimer);
  setTimer3(clockTimer);
  //DIGITAL CLOCK SETUP
  
  //LED MATRIX SETUP
  int matrixTimer = 1;      // 10ms for matrix display (1 * 10ms = 10ms)
  int animationTimer = 10; // 100ms for animation shift (10 * 10ms = 100ms)
  setTimer4(matrixTimer);
  setTimer5(animationTimer);
  //LED MATRIX SETUP
  while (1)
  {
    //LED_SYS OPERATION
    blinkLED_SYS_TIM(durLED_SYS);
    //LED_SYS OPERATION

    //DIGITAL CLOCK OPERATION
    if (timer1_flag == 1)
    {
      setTimer1(multiplexTimer);
      
      update7SEG(currentDisplay);
      
      currentDisplay = (currentDisplay + 1) % 4;
    }
    
    if (timer2_flag == 1)
    {
      setTimer2(colonTimer);
      blinkPin(PA5);
    }
    
    if (timer3_flag == 1)
    {
      setTimer3(clockTimer);
      updateClockBuffer();
    }
    //DIGITAL CLOCK OPERATION
    
    //LED MATRIX OPERATION  
    if (timer4_flag == 1)
    {
      setTimer4(matrixTimer);
      updateLEDMatrix(index_led_matrix);
      index_led_matrix = (index_led_matrix + 1) % MAX_LED_MATRIX;
    }
    
    //LED MATRIX ANIMATION
    if (timer5_flag == 1)
    {
      setTimer5(animationTimer);
      shiftMatrixBufferLeft();
    }
    //LED MATRIX ANIMATION
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  timerRun();
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
