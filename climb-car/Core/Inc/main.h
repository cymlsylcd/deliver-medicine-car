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
#include "stm32f4xx_hal.h"

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
#define Z_Pin GPIO_PIN_2
#define Z_GPIO_Port GPIOC
#define k2_Pin GPIO_PIN_8
#define k2_GPIO_Port GPIOD
#define k3_Pin GPIO_PIN_9
#define k3_GPIO_Port GPIOD
#define AIN1_Pin GPIO_PIN_6
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_7
#define AIN2_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_8
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_9
#define BIN2_GPIO_Port GPIOC
#define k1_Pin GPIO_PIN_15
#define k1_GPIO_Port GPIOA
#define key_Pin GPIO_PIN_1
#define key_GPIO_Port GPIOD
#define reset_Pin GPIO_PIN_2
#define reset_GPIO_Port GPIOD
#define l1_Pin GPIO_PIN_4
#define l1_GPIO_Port GPIOD
#define l2_Pin GPIO_PIN_5
#define l2_GPIO_Port GPIOD
#define l3_Pin GPIO_PIN_6
#define l3_GPIO_Port GPIOD
#define l4_Pin GPIO_PIN_7
#define l4_GPIO_Port GPIOD
#define r1_Pin GPIO_PIN_5
#define r1_GPIO_Port GPIOB
#define r2_Pin GPIO_PIN_6
#define r2_GPIO_Port GPIOB
#define r3_Pin GPIO_PIN_7
#define r3_GPIO_Port GPIOB
#define r4_Pin GPIO_PIN_8
#define r4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
