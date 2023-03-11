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
#include "stm32f3xx_hal.h"

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
#define FUNCTION_KEY1_Pin GPIO_PIN_0
#define FUNCTION_KEY1_GPIO_Port GPIOA
#define FUNCTION_KEY2_Pin GPIO_PIN_1
#define FUNCTION_KEY2_GPIO_Port GPIOA
#define FUNCTION_KEY3_Pin GPIO_PIN_2
#define FUNCTION_KEY3_GPIO_Port GPIOA
#define FUNCTION_KEY4_Pin GPIO_PIN_3
#define FUNCTION_KEY4_GPIO_Port GPIOA
#define EC11_B_Pin GPIO_PIN_6
#define EC11_B_GPIO_Port GPIOC
#define EC11_A_Pin GPIO_PIN_7
#define EC11_A_GPIO_Port GPIOC
#define EC11_A_EXTI_IRQn EXTI9_5_IRQn
#define KNOB_Pin GPIO_PIN_15
#define KNOB_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_5
#define KEY2_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_6
#define KEY3_GPIO_Port GPIOB
#define KEY4_Pin GPIO_PIN_7
#define KEY4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
