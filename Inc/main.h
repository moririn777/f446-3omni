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
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart4;
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
/*TODO:iocファイルを編集した場合変更*/
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
#define PULSES_PER_REV 2048 // エンコーダの1回転あたりのパルス数

// Motor 1 (TIM2_CH4, PB1)
#define MD1_PWM_TIMER &htim2
#define MD1_PWM_CHANNEL TIM_CHANNEL_4
#define MD1_DIR_PORT GPIOB
#define MD1_DIR_PIN GPIO_PIN_1

// Motor 2 (TIM3_CH3, PC7)
#define MD2_PWM_TIMER &htim3
#define MD2_PWM_CHANNEL TIM_CHANNEL_3
#define MD2_DIR_PORT GPIOC
#define MD2_DIR_PIN GPIO_PIN_7

// Motor 3 (TIM1_CH1, PC9)
#define MD3_PWM_TIMER &htim1
#define MD3_PWM_CHANNEL TIM_CHANNEL_1
#define MD3_DIR_PORT GPIOC
#define MD3_DIR_PIN GPIO_PIN_9

// エンコーダタイマー定義
#define ENC1_TIMER &htim3 // PA6, PA7
#define ENC2_TIMER &htim2 // PA0, PA1
#define ENC3_TIMER &htim4 // PB6, PB7

// X相のピン定義 (EXTI割り込み用)
#define ENC1_X_PIN GPIO_PIN_4 // PC4
#define ENC1_X_PORT GPIOC
#define ENC2_X_PIN GPIO_PIN_2 // PA2
#define ENC2_X_PORT GPIOA
#define ENC3_X_PIN GPIO_PIN_5 // PB5
#define ENC3_X_PORT GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
