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

/*File description*/
/*This file contains the FreeRTOS Task callers*/
/*It also provides the FreeRTOS related hooks, Queues and Semaphores*/

/* Includes ------------------------------------------------------------------*/
/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cli_app.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "queue.h"

#include "dts.h"

/*Internal Libraries*/
#include "_libraries.h"

#include "statemachine.h"

#include "main_app.h"

#include "adc.h"

/* FreeRTOS Variables, shared with all tasks */
extern StreamBufferHandle_t xStreamBuffer;
extern QueueHandle_t InterruptQueue;
extern QueueHandle_t InterBoardCom_Queue;
extern QueueHandle_t USB_Tx_Queue;

/*Task Handles*/
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Hz10TaskHandle;
const osThreadAttr_t Hz10Task_attributes = {
  .name = "10HzTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t Hz100TaskHandle;
const osThreadAttr_t Hz100Task_attributes = {
  .name = "100HzTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t cmdLineTaskHandle;
const osThreadAttr_t cmdLineTask_attributes = {
  .name = "cmdLineTask", // defined in cli_app.c
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 32,
};

osThreadId_t InterruptHandlerTaskHandle;
const osThreadAttr_t InterruptHandlerTask_attributes = {
  .name = "InterruptHandlerTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityNormal,
};

/*Tiny low priority USB thread*/
osThreadId_t USBTaskHandle;
const osThreadAttr_t USBTask_attributes = {
  .name = "USBTask",
  .stack_size = 128 * 12,
  .priority = (osPriority_t) osPriorityLow,
};

/*Private function prototypes */
void ReadInternalADC(uint32_t* temperature, uint32_t* v_ref);
void SensorStatus_Reset(SensorStatus *sensor_status);
extern void MX_USB_DEVICE_Init(void);

/*Task Function Prototypes*/
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void StartInterruptHandlerTask(void *argument);
void Start10HzTask(void *argument);
void Start100HzTask(void *argument);
void StartUSBTask(void *argument);

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/*FreeRTOS Hooks*/
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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {

  /*RTOS_MUTEX */
  /* add mutexes, ... */

  /*RTOS_SEMAPHORES */
  xStreamBuffer = xStreamBufferCreate(50, 1); // 1-byte trigger level
  if (xStreamBuffer == NULL) {
    // Handle stream buffer creation failure
  }

  /*RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /*RTOS_QUEUES */
  InterruptQueue = xQueueCreate(10, sizeof(uint8_t)); // Queue for 10 bytes
  InterBoardCom_Queue = xQueueCreate(50, sizeof(InterBoardPacket_t));
  USB_Tx_Queue = xQueueCreate(20, sizeof(InterBoardPacket_t));

  /* RTOS Thread creation */
  defaultTaskHandle = osThreadNew(Task1000Hz, NULL, &defaultTask_attributes);
  Hz10TaskHandle = osThreadNew(Task10Hz, NULL, &Hz10Task_attributes);
  Hz100TaskHandle = osThreadNew(Task100Hz, NULL, &Hz100Task_attributes);

  cmdLineTaskHandle = osThreadNew(vCommandConsoleTask, NULL, &cmdLineTask_attributes);
  InterruptHandlerTaskHandle = osThreadNew(InterruptTask, NULL, &InterruptHandlerTask_attributes);
  USBTaskHandle = osThreadNew(USBTask, NULL, &USBTask_attributes);

  /* RTOS_THREADS */
  /* add threads, ... */

  /* RTOS_EVENTS */
  /* add events, ... */

}


/* Private Functions */
//Find correct place for these functions

void ReadInternalADC(uint32_t* temperature, uint32_t* v_ref) {
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 0xFFFF);
  uint32_t vrefint_raw = HAL_ADC_GetValue(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 0xFFFF);
  uint32_t temp_raw = HAL_ADC_GetValue(&hadc3);
  uint32_t vdda_voltage = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint_raw, ADC_RESOLUTION_12B);

  *v_ref = vdda_voltage; // in mV
  *temperature = __HAL_ADC_CALC_TEMPERATURE(vdda_voltage, temp_raw, ADC_RESOLUTION_12B);
  HAL_ADC_Stop(&hadc3);
}

void SensorStatus_Reset(SensorStatus *sensor_status) {
  sensor_status->hal_status = HAL_OK;
  sensor_status->active = true;
}

/* USER CODE END Application */

