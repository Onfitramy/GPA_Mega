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
void MX_FREERTOS_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FLASH_CS_Pin GPIO_PIN_0
#define FLASH_CS_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_1
#define SD_CS_GPIO_Port GPIOC
#define RGB2_PWM_Pin GPIO_PIN_1
#define RGB2_PWM_GPIO_Port GPIOA
#define VD1_Pin GPIO_PIN_2
#define VD1_GPIO_Port GPIOA
#define VD2_Pin GPIO_PIN_3
#define VD2_GPIO_Port GPIOA
#define F4_INT_Pin GPIO_PIN_4
#define F4_INT_GPIO_Port GPIOA
#define NTC1_Pin GPIO_PIN_4
#define NTC1_GPIO_Port GPIOC
#define NTC_BAT_Pin GPIO_PIN_5
#define NTC_BAT_GPIO_Port GPIOC
#define ACS_Pin GPIO_PIN_0
#define ACS_GPIO_Port GPIOB
#define CAMS_Pin GPIO_PIN_1
#define CAMS_GPIO_Port GPIOB
#define Recovery_Pin GPIO_PIN_2
#define Recovery_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_7
#define Buzzer_GPIO_Port GPIOC
#define NRF24_CE_Pin GPIO_PIN_15
#define NRF24_CE_GPIO_Port GPIOA
#define NRF24_INT_Pin GPIO_PIN_2
#define NRF24_INT_GPIO_Port GPIOD
#define NRF24_INT_EXTI_IRQn EXTI2_IRQn
#define NRF24_CS_Pin GPIO_PIN_3
#define NRF24_CS_GPIO_Port GPIOB
#define GPIO21_Pin GPIO_PIN_4
#define GPIO21_GPIO_Port GPIOB
#define GPIO22_Pin GPIO_PIN_5
#define GPIO22_GPIO_Port GPIOB
#define GPIO23_Pin GPIO_PIN_6
#define GPIO23_GPIO_Port GPIOB
#define GPIO24_Pin GPIO_PIN_7
#define GPIO24_GPIO_Port GPIOB
#define M2_LED_Pin GPIO_PIN_8
#define M2_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
