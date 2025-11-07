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
#define TEST_LED_Pin GPIO_PIN_13
#define TEST_LED_GPIO_Port GPIOC
#define LED_DIGIT_3_Pin GPIO_PIN_1
#define LED_DIGIT_3_GPIO_Port GPIOA
#define LED_DIGIT_2_Pin GPIO_PIN_2
#define LED_DIGIT_2_GPIO_Port GPIOA
#define LED_DIGIT_1_Pin GPIO_PIN_3
#define LED_DIGIT_1_GPIO_Port GPIOA
#define LED_DONT_USE_Pin GPIO_PIN_4
#define LED_DONT_USE_GPIO_Port GPIOA
#define LED_LBL_G_Pin GPIO_PIN_5
#define LED_LBL_G_GPIO_Port GPIOA
#define LED_LBL_C_Pin GPIO_PIN_6
#define LED_LBL_C_GPIO_Port GPIOA
#define LED_LBL_DF_Pin GPIO_PIN_7
#define LED_LBL_DF_GPIO_Port GPIOA
#define LED_LBL_D_Pin GPIO_PIN_0
#define LED_LBL_D_GPIO_Port GPIOB
#define LED_LBL_E_Pin GPIO_PIN_1
#define LED_LBL_E_GPIO_Port GPIOB
#define ENCODER_SW_Pin GPIO_PIN_2
#define ENCODER_SW_GPIO_Port GPIOB
#define LED_LBL_B_Pin GPIO_PIN_12
#define LED_LBL_B_GPIO_Port GPIOB
#define LED_LBL_F_Pin GPIO_PIN_13
#define LED_LBL_F_GPIO_Port GPIOB
#define LED_LBL_A_Pin GPIO_PIN_14
#define LED_LBL_A_GPIO_Port GPIOB
#define ENCODER_DT_Pin GPIO_PIN_4
#define ENCODER_DT_GPIO_Port GPIOB
#define ENCODER_CLK_Pin GPIO_PIN_5
#define ENCODER_CLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
