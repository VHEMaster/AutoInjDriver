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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM2_CH1_INJ1_Pin GPIO_PIN_0
#define TIM2_CH1_INJ1_GPIO_Port GPIOA
#define TIM2_CH2_INJ2_Pin GPIO_PIN_1
#define TIM2_CH2_INJ2_GPIO_Port GPIOA
#define TIM2_CH3_INJ3_Pin GPIO_PIN_2
#define TIM2_CH3_INJ3_GPIO_Port GPIOA
#define TIM2_CH4_INJ4_Pin GPIO_PIN_3
#define TIM2_CH4_INJ4_GPIO_Port GPIOA
#define IN_INJ_1_Pin GPIO_PIN_4
#define IN_INJ_1_GPIO_Port GPIOA
#define IN_INJ_2_Pin GPIO_PIN_5
#define IN_INJ_2_GPIO_Port GPIOA
#define IN_INJ_3_Pin GPIO_PIN_6
#define IN_INJ_3_GPIO_Port GPIOA
#define IN_INJ_4_Pin GPIO_PIN_7
#define IN_INJ_4_GPIO_Port GPIOA
#define OUT_RSVD1_Pin GPIO_PIN_10
#define OUT_RSVD1_GPIO_Port GPIOB
#define OUT_RSVD2_Pin GPIO_PIN_11
#define OUT_RSVD2_GPIO_Port GPIOB
#define OUT_RSVD3_Pin GPIO_PIN_12
#define OUT_RSVD3_GPIO_Port GPIOB
#define OUT_RSVD4_Pin GPIO_PIN_13
#define OUT_RSVD4_GPIO_Port GPIOB
#define OUT_INJ_NEN_Pin GPIO_PIN_14
#define OUT_INJ_NEN_GPIO_Port GPIOB
#define IN_INJ_EN_Pin GPIO_PIN_15
#define IN_INJ_EN_GPIO_Port GPIOB
#define CAN_LBK_Pin GPIO_PIN_15
#define CAN_LBK_GPIO_Port GPIOA
#define SPI1_NSS_ADC_Pin GPIO_PIN_6
#define SPI1_NSS_ADC_GPIO_Port GPIOB
#define ADC_NRST_Pin GPIO_PIN_7
#define ADC_NRST_GPIO_Port GPIOB
#define IN_RSVD1_Pin GPIO_PIN_8
#define IN_RSVD1_GPIO_Port GPIOB
#define IN_RSVD2_Pin GPIO_PIN_9
#define IN_RSVD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
