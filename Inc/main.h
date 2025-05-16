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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define encoder2_A_Pin GPIO_PIN_0
#define encoder2_A_GPIO_Port GPIOA
#define encoder2_B_Pin GPIO_PIN_1
#define encoder2_B_GPIO_Port GPIOA
#define encoder2_X_Pin GPIO_PIN_2
#define encoder2_X_GPIO_Port GPIOA
#define encoder2_X_EXTI_IRQn EXTI2_IRQn
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define encoder1_A_Pin GPIO_PIN_6
#define encoder1_A_GPIO_Port GPIOA
#define encoder1_B_Pin GPIO_PIN_7
#define encoder1_B_GPIO_Port GPIOA
#define encoder1_X_Pin GPIO_PIN_4
#define encoder1_X_GPIO_Port GPIOC
#define encoder1_X_EXTI_IRQn EXTI4_IRQn
#define md1_dir_Pin GPIO_PIN_1
#define md1_dir_GPIO_Port GPIOB
#define md1_pwm_Pin GPIO_PIN_2
#define md1_pwm_GPIO_Port GPIOB
#define md2_dir_Pin GPIO_PIN_7
#define md2_dir_GPIO_Port GPIOC
#define md2_pwm_Pin GPIO_PIN_8
#define md2_pwm_GPIO_Port GPIOC
#define md3_dir_Pin GPIO_PIN_9
#define md3_dir_GPIO_Port GPIOC
#define md3_pwm_Pin GPIO_PIN_8
#define md3_pwm_GPIO_Port GPIOA
#define encoder3_X_Pin GPIO_PIN_5
#define encoder3_X_GPIO_Port GPIOB
#define encoder3_X_EXTI_IRQn EXTI9_5_IRQn
#define encoder3_A_Pin GPIO_PIN_6
#define encoder3_A_GPIO_Port GPIOB
#define encoder3_B_Pin GPIO_PIN_7
#define encoder3_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
