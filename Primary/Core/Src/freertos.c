/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "queue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include "LSM6DSR.h"
#include "ISM330DHCX.h"
#include "LIS3MDL.h"
#include "bmp390.h"
#include "SAM-M8Q.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t InterruptQueue;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t InterruptHandlerTaskHandle;
const osThreadAttr_t InterruptHandlerTask_attributes = {
  .name = "InterruptHandlerTask",
  .stack_size = 128 * 48,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void StartInterruptHandlerTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  uint8_t stackOverflow = 1;
  for(;;)
  {
    
  }
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */
/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  InterruptQueue = xQueueCreate(10, sizeof(uint8_t)); // Queue for 10 bytes
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  InterruptHandlerTaskHandle = osThreadNew(StartInterruptHandlerTask, NULL, &InterruptHandlerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  HAL_Delay(200); // Wait for USB and other Peripherals to initialize
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t R = 255;
  uint8_t G = 0;
  uint8_t B = 0;

  uint8_t SelfTest_Bitfield = 0; //Bitfield for external Devices 0: IMU1, 1: IMU2, 2: MAG, 3: BARO

  /* Infinite loop */
  for(;;)
  {
    // Cycle through colors for a more intense "disco" effect
    /*R = (R + 50) % 255;
    G = (G + 25) % 255;
    B = (B + 15) % 255;
    Set_LED(0, R, G, B);
    Set_Brightness(45);
    WS2812_Send();*/
    SelfTest_Bitfield |= LSM6DSR_SelfTest();
    SelfTest_Bitfield |= LSM6DSR_SelfTest(); //Only works when called twice again???????
    SelfTest_Bitfield |= (ISM330DHCX_SelfTest()<<1);
    SelfTest_Bitfield |= (ISM330DHCX_SelfTest()<<1); //Only works when called twice???????
    SelfTest_Bitfield |= (LIS3MDL_SelfTest()<<2);
    SelfTest_Bitfield |= (BMP390_SelfTest()<<3);
    SelfTest_Bitfield |= (GPS_VER_CHECK()<<4); //Check if GPS is connected and working

    if(SelfTest_Bitfield == 0b11111){
      R = 0;
      G = 255;
      B = 0;
      Set_LED(0, R, G, B);
      Set_Brightness(45);
      WS2812_Send();
    }
    else{
      R = 255;
      G = 0;
      B = 0;
      Set_LED(0, R, G, B);
      Set_Brightness(45);
      WS2812_Send();
    }

    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN InterruptHandlerTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END InterruptHandlerTask */
void StartInterruptHandlerTask(void *argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t receivedData;
  char GPS_Buffer[100]; // Buffer for GPS data
  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); Disabled due to always triggering when GPS is not connected
  /* Infinite loop */
  for(;;)
  { 
    if (xQueueReceive(InterruptQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
      if(receivedData == 0x10) {
        ublox_ReadOutput(GPS_Buffer); //Read the GPS output and decode it
      }
    }
    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

