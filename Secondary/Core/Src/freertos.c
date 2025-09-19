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
//#include "cli_app.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "queue.h"

#include "ws2812.h"
#include "W25Q1.h"
#include "xBee.h"
#include "VoltageReader.h"
#include "InterBoardCom.h"
#include "status.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern UART_HandleTypeDef huart1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
QueueHandle_t InterBoardPacketQueue;
QueueHandle_t XBeeDataQueue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t Counter_100Hz = 0;

/* Definitions for defaultTask */

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Hz10TaskHandle;
const osThreadAttr_t Hz10Task_attributes = {
  .name = "10HzTask",
  .stack_size = 128 * 48,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t InterBoardComHandle;
const osThreadAttr_t InterBoardCom_attributes = {
  .name = "InterBoardCom",
  .stack_size = 128 * 64,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t InterruptTaskHandle;
const osThreadAttr_t InterruptTask_attributes = {
  .name = "InterruptTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityRealtime,
};

extern double TemperatureAMS;
extern double voltage5V0bus;
extern double voltageBATbus;

uint8_t XBee_Temp;
int8_t secondary_status = 0;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Start10HzTask(void *argument);
void StartInterBoardComTask(void *argument);
void StartInterruptTask(void *argument);

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  InterBoardPacketQueue = xQueueCreate(64, sizeof(InterBoardPacket_t)); // Queue for 64 InterBoardPacket_t packets
  XBeeDataQueue = xQueueCreate(4, sizeof(xbee_frame_t)); // Queue for 4 XBee data packets
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  Hz10TaskHandle = osThreadNew(Start10HzTask, NULL, &Hz10Task_attributes);
  InterBoardComHandle = osThreadNew(StartInterBoardComTask, NULL, &InterBoardCom_attributes);
  InterruptTaskHandle = osThreadNew(StartInterruptTask, NULL, &InterruptTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

uint8_t temperatureSaved[128];
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10; //100 Hz

  uint8_t transmitPayload[32] = {1, 2, 3, 4, 5};

  /* Infinite loop */
  for(;;) {
    Counter_100Hz++;
    TemperatureAMS = readTemperature(1);
    voltage5V0bus = readVoltage(1) * (10 + 10) / 10;
    voltageBATbus = readVoltage(2) * (10 + 2.2) / 2.2;

    ShowStatus(RGB_SECONDARY, secondary_status, 1, 100);

    //XBee_Transmit(transmitPayload, 5, 0x00);

    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 100Hz
  }

  /* USER CODE END StartDefaultTask */
}

void Start10HzTask(void *argument){
  /* USER CODE BEGIN Start10HzTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1000; // 1 Hz
  InterBoardPacket_t packet = InterBoardCom_CreatePacket(InterBoardPACKET_ID_SELFTEST);

  /* Infinite loop */
  for(;;) {
    //InterBoardCom_SendPacket(packet);
    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 10Hz
  }

  /* USER CODE END Start10HzTask */
}

void StartInterBoardComTask(void *argument)
{
  /* USER CODE BEGIN StartInterBoardComTask */
  W25Q_GetConfig();
  InterBoardCom_ActivateReceive();
  /* Infinite loop */
  for(;;) {
    InterBoardPacket_t packet;
    if (xQueueReceive(InterBoardPacketQueue, &packet, portMAX_DELAY) == pdPASS) {
        HAL_GPIO_TogglePin(M2_LED_GPIO_Port, M2_LED_Pin); // Toggle M2 LED to indicate packet received
        InterBoardCom_ParsePacket(packet);
    }
  }
  /* USER CODE END StartInterBoardComTask */
}

uint8_t transmitStatus;
uint8_t test = 0;
void StartInterruptTask(void *argument)
{
  /* USER CODE BEGIN StartInterruptTask */
  /* Infinite loop */
  for(;;) {
    xbee_frame_t packet;
    if (xQueueReceive(XBeeDataQueue, (uint8_t*)&packet, portMAX_DELAY) == pdPASS) {
      if (packet.frame_data[0] == 0x88) { // Local AT Command Response

        //Temperature Response
        if (packet.frame_data[2] == 'T' && packet.frame_data[3] == 'P') { // TP command
          // Extract temperature from the response
          if (packet.frame_data[4] == 0x00) { // Check if command was successful
            uint16_t rawTemp = packet.frame_data[6];
            XBee_Temp = rawTemp;
          } else {
            // Handle error
            XBee_Temp = 255; // Invalid temperature
          }
        
        //Device Identifier Response
        } else if (packet.frame_data[2] == 'D' && packet.frame_data[3] == 'D') { // DD command
          // Device Identifier Response
          if (packet.frame_data[4] == 0x00) { // Check if command was successful
            // Process device identifier if needed
          } else {
            // Handle error
          }
        } 
      }  else if (packet.frame_data[0] == 0x90) { // RX Packet
          // Process received RF data
          uint8_t* rfData = &packet.frame_data[12]; // RF data starts at byte 12
          uint16_t rfDataLength = (packet.frame_length[0] << 8 | packet.frame_length[1]) - 12; // Length of RF data
          test += 1;
          // Handle rfData as needed
      } else if (packet.frame_data[0] == 0x8B) { // Transmit Status
          // Process transmit status
          uint8_t frameID = packet.frame_data[1];
          transmitStatus = packet.frame_data[5];
          // Handle transmit status as needed
      }
    }
  }
  /* USER CODE END StartInterruptTask */
}

/* USER CODE END Application */

