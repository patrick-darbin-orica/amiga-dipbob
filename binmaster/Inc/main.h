/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "binmaster_protocol.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void UART_IdleCallback(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_Pin GPIO_PIN_0
#define ENCODER_GPIO_Port GPIOA
#define UP_Pin GPIO_PIN_1
#define UP_GPIO_Port GPIOA
#define UP_EXTI_IRQn EXTI1_IRQn
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define SCLK_Pin GPIO_PIN_5
#define SCLK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define ENCODER_CUT_Pin GPIO_PIN_0
#define ENCODER_CUT_GPIO_Port GPIOB
#define ENCODER_CUT_EXTI_IRQn EXTI0_IRQn
#define AHI_Pin GPIO_PIN_8
#define AHI_GPIO_Port GPIOA
#define BHI_Pin GPIO_PIN_9
#define BHI_GPIO_Port GPIOA
#define BLO_Pin GPIO_PIN_10
#define BLO_GPIO_Port GPIOA
#define ALO_Pin GPIO_PIN_11
#define ALO_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_12
#define EN_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define DOWN_Pin GPIO_PIN_5
#define DOWN_GPIO_Port GPIOB
#define DOWN_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
/** Control Macros */
#define __ENABLE_DRIVER() HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET)
#define __DISABLE_DRIVER() HAL_GPIO_WritePin(EN__GPIO_Port, EN__Pin, GPIO_PIN_RESET)

/** Enums */
typedef enum {
  IDLE = 0,
  DIPPING_DESCEND_INIT,
  DIPPING_DESCEND,
  DIPPING_ASCEND_INIT,
  DIPPING_ASCEND
} UnitState;

typedef enum {
  STOP = 0,
  UP_RAMP,
  UP,
  DOWN_RAMP,
  DOWN,
} DriveState;

typedef enum {
  AMT22_NOP = 0x00,
  AMT22_RESET = 0x60,
  AMT22_ZERO = 0x70
} AMT22Command;

/** Variable Defaults */
#define TIMEOUT_0 5000 // Initial Descent
#define TIMEOUT_1 100 // Continuous Descent
#define TIMEOUT_2 5000 // Initial Ascent
#define TIMEOUT_3 100 // Continuous Ascent

#define ASCENT_RATE 50 // 50% PWM
#define DESCENT_RATE 50 // 50% PWM

#define MOTOR_FREQ 16000 // ~5kHz switching freq

#define TENSION_FREQ 16000 // ~5kHz

#define RAMP_INCREMENT 3 // Crude soft start

/** Control Port */
#define STLINK_UART

#ifdef STLINK_UART
#define TERMINAL_UART &huart2
#else
#define TERMINAL_UART &huart1
#endif

/** Buffer Lengths */
#define UART_RX_BUFFER_LEN 64
#define HALF_TENSION_BUFFER_LEN 64
#define TENSION_BUFFER_LEN 128
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
