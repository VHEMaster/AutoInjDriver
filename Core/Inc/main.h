/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define OUT_INJ_EN_Pin GPIO_PIN_0
#define OUT_INJ_EN_GPIO_Port GPIOF
#define IN_INJ_EN_Pin GPIO_PIN_1
#define IN_INJ_EN_GPIO_Port GPIOF
#define IN_INJ_1_Pin GPIO_PIN_0
#define IN_INJ_1_GPIO_Port GPIOA
#define IN_INJ_2_Pin GPIO_PIN_1
#define IN_INJ_2_GPIO_Port GPIOA
#define OUT_INJ_1_Pin GPIO_PIN_4
#define OUT_INJ_1_GPIO_Port GPIOA
#define OUT_INJ_2_Pin GPIO_PIN_5
#define OUT_INJ_2_GPIO_Port GPIOA
#define OUT_INJ_3_Pin GPIO_PIN_6
#define OUT_INJ_3_GPIO_Port GPIOA
#define OUT_INJ_4_Pin GPIO_PIN_7
#define OUT_INJ_4_GPIO_Port GPIOA
#define IN_INJ_4_Pin GPIO_PIN_1
#define IN_INJ_4_GPIO_Port GPIOB
#define IN_INJ_3_Pin GPIO_PIN_11
#define IN_INJ_3_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
