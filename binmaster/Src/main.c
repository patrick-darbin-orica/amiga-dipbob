/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Driver Variables */
UnitState state = IDLE;
DriveState drive_state = STOP;

uint32_t count = 0;
uint16_t depth = 0;

uint16_t encoder_timeout[4] = {TIMEOUT_0, TIMEOUT_1, TIMEOUT_2, TIMEOUT_3};

uint8_t ascent_rate = ASCENT_RATE;
uint8_t descent_rate = DESCENT_RATE;

uint16_t ramp_rate = RAMP_INCREMENT;

uint16_t current_tim_compare = 0;
uint16_t steady_tim_compare = 0;

uint32_t prev_count = 0;
uint16_t tension_buffer_pos = 0;
int16_t tension_buffer[TENSION_BUFFER_LEN];

/* UART Variables */
uint8_t uart_rx_buffer[UART_RX_BUFFER_LEN];
uint16_t uart_rx_length;
volatile bool uart_rx_complete;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
//void read_spi_amt(AMT22Command command, uint8_t *buf);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Implement a uart receive with idle line detection
 *
 * Non-Blocking Function
 *
 * @param huart   UART handle.
 * @param pData   Pointer to data buffer.
 * @param Size    Amount of data to be received.
 * @retval None
 */
void UARTReceiveIdleIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t bufferSize) {
    uart_rx_complete = false;
    uart_rx_length = 0;
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_IT(huart, pData, bufferSize);
}

/**
  * @brief  UART Idle callback.
  *
  * @retval None
  */
void UART_IdleCallback(void) {
    if (uart_rx_complete) {
        processProtocolCommand(uart_rx_buffer, uart_rx_length);
        UARTReceiveIdleIT(TERMINAL_UART, uart_rx_buffer, UART_RX_BUFFER_LEN);
    }
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  /* Allow AMT22 to power on */
  HAL_Delay(1000);

  /* Set Timer Rate */
  __HAL_TIM_SET_AUTORELOAD(&htim7, TENSION_FREQ);

  /* Clear timer interrupt */
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_SR_UIF);

  /* Enable Driver */
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
  __HAL_TIM_SET_AUTORELOAD(&htim1, MOTOR_FREQ);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __ENABLE_DRIVER();

  /* Begin serial prompt */
  UARTReceiveIdleIT(TERMINAL_UART, uart_rx_buffer, UART_RX_BUFFER_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#pragma clang diagnostic pop
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BLO_Pin|ALO_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UP_Pin */
  GPIO_InitStruct.Pin = UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin BLO_Pin ALO_Pin EN_Pin */
  GPIO_InitStruct.Pin = CS_Pin|BLO_Pin|ALO_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_CUT_Pin */
  GPIO_InitStruct.Pin = ENCODER_CUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_CUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DOWN_Pin */
  GPIO_InitStruct.Pin = DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DOWN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/** Control Functions */

void motor_up(uint8_t speed, bool ramp) {
  HAL_GPIO_WritePin(BLO_GPIO_Port, BLO_Pin, GPIO_PIN_SET);

  if (ramp) {
    steady_tim_compare = (uint16_t) (MOTOR_FREQ * (speed / 100.));
    current_tim_compare = steady_tim_compare / 10;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_tim_compare);
    drive_state = UP_RAMP;
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  } else {
    uint16_t duty_cycle = (uint16_t) (MOTOR_FREQ * (speed / 100.));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
   drive_state = UP;
  }
}

void motor_down(uint8_t speed, bool ramp) {
  HAL_GPIO_WritePin(ALO_GPIO_Port, ALO_Pin, GPIO_PIN_SET);

  if (ramp) {
    steady_tim_compare = (uint16_t) (MOTOR_FREQ * (speed / 100.));
    current_tim_compare = steady_tim_compare / 10;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, current_tim_compare);
    drive_state = DOWN_RAMP;
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  } else {
    uint16_t duty_cycle = (uint16_t) (MOTOR_FREQ * (speed / 100.));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle);
    drive_state = DOWN;
  }
}

void motor_stop(void) {
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  HAL_GPIO_WritePin(ALO_GPIO_Port, ALO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLO_GPIO_Port, BLO_Pin, GPIO_PIN_RESET);
  drive_state = STOP;
}

void dipping_descend(void) {
  if (state == IDLE) {
    state = DIPPING_DESCEND_INIT;
    prev_count = 0;
    count = 0; // TODO compare this to a hardware timer. See if the interrupt is missing pulses etc.
    depth = 0;
    tension_buffer_pos = 0;
    motor_down(descent_rate, true);
    __HAL_TIM_SET_AUTORELOAD(&htim6, encoder_timeout[0]);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
  }
}

void dipping_ascend(void) {
  if (state == DIPPING_DESCEND) {
    state = DIPPING_ASCEND_INIT;
    motor_up(ascent_rate, true);
    __HAL_TIM_SET_AUTORELOAD(&htim6, encoder_timeout[2]);
    HAL_TIM_Base_Start_IT(&htim6);
  }
}

/**
 * @brief Handle Up Callback
 *
 * @retval None
 *
 */
void up_interrupt(void) {
  if (HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin)) { // Pin not pressed
    motor_stop();
  } else { // Pin is pressed
    motor_up(50, false);
  }
}

/**
 * @brief Handle Down Callback
 *
 * @retval None
 *
 */
void down_interrupt(void) {
  if (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin)) { // Pin not pressed
    motor_stop();
  } else { // Pin is pressed
    motor_down(50, false);
  }
}

/**
 * @brief Handle Encoder Cut Callback
 *
 * @retval None
 *
 */
void encoder_cut_interrupt(void) {
  if (state == DIPPING_DESCEND || state == DIPPING_ASCEND) {
    __HAL_TIM_SET_COUNTER(&htim6, 0);
  }
  count++; // TODO Validate (should count in init phases also)
}

void read_spi_amt(AMT22Command command, uint8_t *buf) {
  uint8_t output[2] = {0};
  output[1] = (uint8_t) command;
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, output, buf, 1, 0xFFFF);
  HAL_SPI_TransmitReceive(&hspi1, &output[1], &buf[1], 1, 0xFFFF);
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

int16_t get_abs_position_amt(void) {
  uint8_t spi_buf[2];
  read_spi_amt(AMT22_NOP, spi_buf);
  int16_t value = (int16_t) ((spi_buf[0] << 10) | (spi_buf[1] << 2)); //TODO parity check
  return value;
}

/** Interrupts */

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
    case UP_Pin:
      up_interrupt();
      break;
    case DOWN_Pin:
      down_interrupt();
      break;
    case ENCODER_CUT_Pin:
      encoder_cut_interrupt();
      break;
    default:
      break;
  }
}

/**
 * @brief Timer elapsed interrupts
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    HAL_TIM_Base_Stop_IT(&htim6);

    switch (state) {
      case DIPPING_DESCEND_INIT:
        __HAL_TIM_SET_AUTORELOAD(&htim6, encoder_timeout[1]);
        HAL_TIM_Base_Start_IT(&htim6);
        state = DIPPING_DESCEND;
        break;
      case DIPPING_DESCEND:
        motor_stop();
        depth = (uint16_t) (count * 1.91); // TODO is this the best place for this?
        dipping_ascend();
        break;
      case DIPPING_ASCEND_INIT:
        __HAL_TIM_SET_AUTORELOAD(&htim6, encoder_timeout[3]);
        HAL_TIM_Base_Start_IT(&htim6);
        state = DIPPING_ASCEND;
        break;
      case DIPPING_ASCEND:
        motor_stop();
        state = IDLE;
        HAL_TIM_Base_Stop_IT(&htim7);
        __HAL_TIM_CLEAR_FLAG(&htim7, TIM_SR_UIF);

        /* If DMA is still running from last buffer send */
        if ((TERMINAL_UART)->gState != HAL_UART_STATE_READY) {
          while (!READ_BIT((TERMINAL_UART)->Instance->CR1, USART_CR1_TCIE)) {}
          CLEAR_BIT((TERMINAL_UART)->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
          (TERMINAL_UART)->gState = HAL_UART_STATE_READY;
        }

        uint16_t terminator = 0xFFFF;
        HAL_UART_Transmit(TERMINAL_UART, (uint8_t*) &terminator, 2, 0xFFFF);
        break;
      default:
        break;
    }

    /* Clear timer interrupt */
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
  } else if (htim == &htim1) {
    if (current_tim_compare < (steady_tim_compare - ramp_rate)) {
      current_tim_compare += ramp_rate;

      if (drive_state == UP_RAMP) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_tim_compare);
      } else if (drive_state == DOWN_RAMP) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, current_tim_compare);
      }
    } else {
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
      if (drive_state == UP_RAMP) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, steady_tim_compare);
        drive_state = UP;
      } else if (drive_state == DOWN_RAMP) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, steady_tim_compare);
        drive_state = DOWN;
      }
    }
  } else if (htim == &htim7) {
    /* Get current tension arm angle */
    if (count > prev_count) { // If distance encoder count has increased pack bit
      tension_buffer[tension_buffer_pos] = (int16_t) __REV16(get_abs_position_amt() | 0x2);
      prev_count = count;
    } else {
      tension_buffer[tension_buffer_pos] = (int16_t) __REV16(get_abs_position_amt());
    }
    tension_buffer_pos++;

    /* Send buffer out UART if required */
    if (tension_buffer_pos == HALF_TENSION_BUFFER_LEN) {
      HAL_UART_Transmit_DMA(TERMINAL_UART, (uint8_t*) tension_buffer, TENSION_BUFFER_LEN); // 2 x Half Buffer (16 bit)
    } else if (tension_buffer_pos == TENSION_BUFFER_LEN) {
      HAL_UART_Transmit_DMA(TERMINAL_UART, (uint8_t*) &tension_buffer[HALF_TENSION_BUFFER_LEN], TENSION_BUFFER_LEN);
      tension_buffer_pos = 0;
    }
  }
}

/** Protogen Functions */

/**
 * @brief Cycle Unit
 *
 * @retval None
 *
 */
void execute_cycle(void) {
  dipping_descend();
}

/**
 * @brief Ascent rate (percent)
 *
 * @param data    Data from stream
 * @param length  Length of data
 * @retval None
 *
 */
void execute_set_ascent_rate(uint8_t *data, uint16_t length) {
  ascent_rate = data[0];
}

/**
 * @brief Descent rate (percent)
 *
 * @param data    Data from stream
 * @param length  Length of data
 * @retval None
 *
 */
void execute_set_descent_rate(uint8_t *data, uint16_t length) {
  descent_rate = data[0];
}

/**
 * @brief Timeout (10us)
 *
 * @param data    Data from stream
 * @param length  Length of data
 * @retval None
 *
 */
void execute_set_timeout(uint8_t *data, uint16_t length) {
  encoder_timeout[data[0]] = (uint16_t) __REV16(*(uint16_t*) &data[1]);
}

/**
 * @brief Ramp rate
 *
 * @param data    Data from stream
 * @param length  Length of data
 * @retval None
 *
 */
void execute_set_ramp_rate(uint8_t *data, uint16_t length) {
  ramp_rate = (uint16_t) __REV16(*(uint16_t*) data);
}

/**
 * @brief Get ascent rate (percent)
 *
 * @retval None
 *
 */
void execute_get_ascent_rate(void) {
  HAL_UART_Transmit(TERMINAL_UART, &ascent_rate, 1, 0xFFFF);
}

/**
 * @brief Get descent rate (percent)
 *
 * @retval None
 *
 */
void execute_get_descent_rate(void) {
  HAL_UART_Transmit(TERMINAL_UART, &descent_rate, 1, 0xFFFF);
}

/**
 * @brief Timeout (10us)
 *
 * @param data    Data from stream
 * @param length  Length of data
 * @retval None
 *
 */
void execute_get_timeout(uint8_t *data, uint16_t length) {
  uint16_t timeout = (uint16_t) __REV16(encoder_timeout[data[0]]);
  HAL_UART_Transmit(TERMINAL_UART, (uint8_t*) &timeout, 2, 0xFFFF);
}

/**
 * @brief Ramp rate
 *
 * @retval None
 *
 */
void execute_get_ramp_rate(void) {
  uint16_t ramp_increment = (uint16_t) __REV16(ramp_rate);
  HAL_UART_Transmit(TERMINAL_UART, (uint8_t*) &ramp_increment, 2, 0xFFFF);
}

/**
 * @brief Water level and Bottom level
 *
 * @retval None
 *
 */
void execute_get_previous_level(void) {
  uint16_t levels[2] = {0, 0};
  levels[1] = (uint16_t) __REV16(depth);
  HAL_UART_Transmit(TERMINAL_UART, (uint8_t*) levels, 4, 0xFFFF);
}

void execute_send_motor_up(void) {
  motor_up(ascent_rate, true);
}

void execute_send_motor_down(void) {
  motor_down(descent_rate, true);
}

void execute_send_motor_stop(void) {
	if (state == IDLE){
		motor_stop();
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
