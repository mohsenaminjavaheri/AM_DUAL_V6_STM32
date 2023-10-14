/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define TxTune_Pin GPIO_PIN_0
#define TxTune_GPIO_Port GPIOA
#define Alarm_Pin GPIO_PIN_5
#define Alarm_GPIO_Port GPIOA
#define Tx_En_Pin GPIO_PIN_6
#define Tx_En_GPIO_Port GPIOA
#define ZC_Out_Pin GPIO_PIN_0
#define ZC_Out_GPIO_Port GPIOB
#define ZCD_Pin GPIO_PIN_1
#define ZCD_GPIO_Port GPIOB
#define ZCD_EXTI_IRQn EXTI0_1_IRQn
#define TX_Pin GPIO_PIN_11
#define TX_GPIO_Port GPIOA
#define TX_EXTI_IRQn EXTI4_15_IRQn
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define Key_Pin GPIO_PIN_15
#define Key_GPIO_Port GPIOA
#define Key_EXTI_IRQn EXTI4_15_IRQn
#define Relay1_Pin GPIO_PIN_3
#define Relay1_GPIO_Port GPIOB
#define Relay2_Pin GPIO_PIN_4
#define Relay2_GPIO_Port GPIOB
#define Relay3_Pin GPIO_PIN_5
#define Relay3_GPIO_Port GPIOB
#define Relay4_Pin GPIO_PIN_6
#define Relay4_GPIO_Port GPIOB
#define Relay5_Pin GPIO_PIN_7
#define Relay5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
