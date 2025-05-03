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
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern QueueHandle_t InterruptQueue;
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
#define IMU1_CS_Pin GPIO_PIN_4
#define IMU1_CS_GPIO_Port GPIOE
#define MAG_CS_Pin GPIO_PIN_13
#define MAG_CS_GPIO_Port GPIOC
#define IMU1_INT1_Pin GPIO_PIN_14
#define IMU1_INT1_GPIO_Port GPIOC
#define IMU1_INT1_EXTI_IRQn EXTI15_10_IRQn
#define IMU1_INT2_Pin GPIO_PIN_15
#define IMU1_INT2_GPIO_Port GPIOC
#define IMU1_INT2_EXTI_IRQn EXTI15_10_IRQn
#define P1_READ_Pin GPIO_PIN_0
#define P1_READ_GPIO_Port GPIOC
#define P2_READ_Pin GPIO_PIN_1
#define P2_READ_GPIO_Port GPIOC
#define PYRO1_Pin GPIO_PIN_2
#define PYRO1_GPIO_Port GPIOC
#define PYRO2_Pin GPIO_PIN_3
#define PYRO2_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_2
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_3
#define PWM4_GPIO_Port GPIOA
#define F4_INT_Pin GPIO_PIN_4
#define F4_INT_GPIO_Port GPIOA
#define F4_INT_EXTI_IRQn EXTI4_IRQn
#define GPIO11_Pin GPIO_PIN_0
#define GPIO11_GPIO_Port GPIOB
#define GPIO12_Pin GPIO_PIN_1
#define GPIO12_GPIO_Port GPIOB
#define GPIO13_Pin GPIO_PIN_2
#define GPIO13_GPIO_Port GPIOB
#define GPIO14_Pin GPIO_PIN_7
#define GPIO14_GPIO_Port GPIOE
#define GNSS_INT_Pin GPIO_PIN_10
#define GNSS_INT_GPIO_Port GPIOE
#define GNSS_TP_Pin GPIO_PIN_11
#define GNSS_TP_GPIO_Port GPIOE
#define GNSS_TP_EXTI_IRQn EXTI15_10_IRQn
#define GNSS_RST_Pin GPIO_PIN_12
#define GNSS_RST_GPIO_Port GPIOE
#define GNSS_TX_RDY_Pin GPIO_PIN_13
#define GNSS_TX_RDY_GPIO_Port GPIOE
#define GNSS_TX_RDY_EXTI_IRQn EXTI15_10_IRQn
#define GNSS_SCL_Pin GPIO_PIN_10
#define GNSS_SCL_GPIO_Port GPIOB
#define GNSS_SDA_Pin GPIO_PIN_11
#define GNSS_SDA_GPIO_Port GPIOB
#define EXT2_CS_Pin GPIO_PIN_12
#define EXT2_CS_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_12
#define PWM5_GPIO_Port GPIOD
#define PWM6_Pin GPIO_PIN_13
#define PWM6_GPIO_Port GPIOD
#define PWM7_Pin GPIO_PIN_14
#define PWM7_GPIO_Port GPIOD
#define PWM8_Pin GPIO_PIN_15
#define PWM8_GPIO_Port GPIOD
#define RGB1_PWM_Pin GPIO_PIN_6
#define RGB1_PWM_GPIO_Port GPIOC
#define BMP_INT_Pin GPIO_PIN_8
#define BMP_INT_GPIO_Port GPIOC
#define BMP_INT_EXTI_IRQn EXTI9_5_IRQn
#define IMU2_CS_Pin GPIO_PIN_0
#define IMU2_CS_GPIO_Port GPIOD
#define IMU2_DEN_Pin GPIO_PIN_1
#define IMU2_DEN_GPIO_Port GPIOD
#define IMU2_DEN_EXTI_IRQn EXTI1_IRQn
#define IMU2_INT_Pin GPIO_PIN_2
#define IMU2_INT_GPIO_Port GPIOD
#define IMU2_INT_EXTI_IRQn EXTI2_IRQn
#define EXT1_CS_Pin GPIO_PIN_4
#define EXT1_CS_GPIO_Port GPIOD
#define NRF_INT_Pin GPIO_PIN_5
#define NRF_INT_GPIO_Port GPIOD
#define NRF_INT_EXTI_IRQn EXTI9_5_IRQn
#define NRF_CE_Pin GPIO_PIN_6
#define NRF_CE_GPIO_Port GPIOD
#define NRF_CS_Pin GPIO_PIN_7
#define NRF_CS_GPIO_Port GPIOD
#define M1_LED_Pin GPIO_PIN_8
#define M1_LED_GPIO_Port GPIOB
#define MAG_DRDY_Pin GPIO_PIN_9
#define MAG_DRDY_GPIO_Port GPIOB
#define MAG_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define MAG_INT_Pin GPIO_PIN_0
#define MAG_INT_GPIO_Port GPIOE
#define MAG_INT_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */
typedef struct {
  double accel[3];  // X, Y, Z
  double gyro[3];   // X, Y, Z
} StateVector;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
