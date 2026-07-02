/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define TIM1_CH1_PWM_R_Pin GPIO_PIN_0
#define TIM1_CH1_PWM_R_GPIO_Port GPIOC
#define TIM1_CH2_PWM_G_Pin GPIO_PIN_1
#define TIM1_CH2_PWM_G_GPIO_Port GPIOC
#define TIM1_CH1_PWM_B_Pin GPIO_PIN_2
#define TIM1_CH1_PWM_B_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_4
#define LD1_GPIO_Port GPIOA
#define DOWN_BUTTON_Pin GPIO_PIN_1
#define DOWN_BUTTON_GPIO_Port GPIOB
#define UP_BUTTON_Pin GPIO_PIN_2
#define UP_BUTTON_GPIO_Port GPIOB
#define DEBUG_LED_Pin GPIO_PIN_11
#define DEBUG_LED_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define WIZ_RSTn_Pin GPIO_PIN_8
#define WIZ_RSTn_GPIO_Port GPIOC
#define WIZ_INT_Pin GPIO_PIN_9
#define WIZ_INT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
