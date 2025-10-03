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
#include "string.h"
#include "fatfs.h"

#include "ws2812.h"
#include "W25Q1.h"
#include "xBee.h"
#include "VoltageReader.h"
#include "InterBoardCom.h"
#include "status.h"
#include "PowerUnit.h"
#include "SD.h"

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
/* Definitions for defaultTask */

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 48,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Hz10TaskHandle;
const osThreadAttr_t Hz10Task_attributes = {
  .name = "10HzTask",
  .stack_size = 128 * 84,
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
  .stack_size = 128 * 48,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t SDTaskHandle;
const osThreadAttr_t SDTask_attributes = {
  .name = "SDTask",
  .stack_size = 128 * 48,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

uint8_t XBee_Temp;
int8_t secondary_status = 0;

uint32_t Counter_100Hz = 0;
uint32_t Counter_10Hz = 0;

struct {
  struct {
    float reg_3V3;
    float battery;
  } temperature;
  struct {
    float bus_pu_bat;
    float bus_gpa_bat;
    float bus_5V;
    float shunt_pu;
  } voltage;
  struct {
    float out_pu;
  } current;
  struct {
    float out_pu;
  } power;
} health;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Start10HzTask(void *argument);
void StartInterBoardComTask(void *argument);
void StartInterruptTask(void *argument);
void StartSDTask(void *argument);

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
  SDTaskHandle = osThreadNew(StartSDTask, NULL, &SDTask_attributes);
  InterruptTaskHandle = osThreadNew(StartInterruptTask, NULL, &InterruptTask_attributes);
  InterBoardComHandle = osThreadNew(StartInterBoardComTask, NULL, &InterBoardCom_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

uint32_t StartTime = 0;
uint8_t temperatureSaved[128];
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
uint32_t RunTime = 0;
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10; //100 Hz

  /* Infinite loop */
  for(;;) {
    Counter_100Hz++;
    InterBoardPacket_t packet = InterBoardCom_CreatePacket(INTERBOARD_OP_NONE);
    InterBoardCom_FillRaw(&packet, 32, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);
    memcpy((uint8_t *)&packet.Data, &health.temperature.reg_3V3, sizeof(float));
    //InterBoardCom_SendPacket(&packet);

    // show low battery level

    ShowStatus(RGB_SECONDARY, secondary_status, 1, 100);

    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 100Hz
  }

  /* USER CODE END StartDefaultTask */
}

void Start10HzTask(void *argument){
  /* USER CODE BEGIN Start10HzTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; // 10 Hz

  uint8_t XBEEtransmitBuffer[32] = {7, 222, 241, 33, 0, 81, 100, 65, 188, 82, 195, 93, 59, 65, 83, 112, 63, 0}; // Test data
  /* Infinite loop */
  for(;;) {
    Counter_10Hz++;
    switch(secondary_status) {
      // critically low battery condition
      case -5:
        if(INA219_readBusVoltage(&health.voltage.bus_pu_bat) != HAL_OK) secondary_status = -1;
        break;

      // low battery condition
      case -4:
        if(INA219_readBusVoltage(&health.voltage.bus_pu_bat) != HAL_OK) secondary_status = -1;
        if(health.voltage.bus_pu_bat < 6) secondary_status = -5;
        break;

      // PU connection loss fault
      case -1:

      // init
      case 0:
        if(INA219_init() == HAL_OK) secondary_status = 1;
        break;

      // normal operations
      case 1:
        health.temperature.battery = readTemperature(2);
        if(INA219_readBusVoltage(&health.voltage.bus_pu_bat) != HAL_OK) secondary_status = -1;
        if(INA219_readShuntVoltage(&health.voltage.shunt_pu) != HAL_OK) secondary_status = -1;
        if(INA219_readPower(&health.power.out_pu) != HAL_OK) secondary_status = -1;
        if(INA219_readCurrent(&health.current.out_pu) != HAL_OK) secondary_status = -1;
        if(health.voltage.bus_pu_bat < 6.6) secondary_status = -4;
        break;

    }

    health.temperature.reg_3V3 = readTemperature(1);
    health.voltage.bus_5V = readVoltage(1) * (10 + 10) / 10;
    health.voltage.bus_gpa_bat = readVoltage(2) * (10 + 2.2) / 2.2;

    XBee_GetTemperature();
    //XBee_Transmit(XBEEtransmitBuffer, 32, 0); // Transmit 32 bytes of test data

    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 10Hz
  }

  /* USER CODE END Start10HzTask */
}

uint16_t receivedPackets = 0;

void StartInterBoardComTask(void *argument)
{
  /* USER CODE BEGIN StartInterBoardComTask */
  uint32_t DMA_ReRun_time = HAL_GetTick() + 6; // Timestamp of the next time the DMA should be reactivated, nominaly every 6ms after the last packet was received

  W25Q_GetConfig();

  InterBoardCom_Init();
  InterBoardCom_ActivateReceive();
  /* Infinite loop */
  for(;;) {
    InterBoardPacket_t packet;
    // Calculate 0.1ms in ticks based on configTICK_RATE_HZ
    if (xQueueReceive(InterBoardPacketQueue, &packet, 1) == pdPASS) { // 1ms delay
      InterBoardCom_ParsePacket(packet);
      #ifdef DEBUG
      DMA_ReRun_time = HAL_GetTick() + 6; //Set the next reactivation time to 6ms in the future
      HAL_GPIO_TogglePin(M2_LED_GPIO_Port, M2_LED_Pin); // Toggle M2 LED to indicate packet received
      #endif
    }

    if (HAL_GetTick() > DMA_ReRun_time) {
      //Re-activate the DMA every 6ms to ensure it is always active
      InterBoardCom_ActivateReceive();
      DMA_ReRun_time = HAL_GetTick() + 6;
    }
  }
  /* USER CODE END StartInterBoardComTask */
}

uint8_t transmitStatus;
uint8_t XBEE_TransmittedReceived = 0;
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
        } else if (packet.frame_data[2] == 'E' && packet.frame_data[3] == 'R'){ //Received Error Count
          // Error Response
        }
      }  else if (packet.frame_data[0] == 0x90) { // RX Packet
          // Process received RF data
          uint8_t* rfData = &packet.frame_data[12]; // RF data starts at byte 12
          uint16_t rfDataLength = (packet.frame_length[0] << 8 | packet.frame_length[1]) - 12; // Length of RF data

          XBEE_TransmittedReceived += 1;
          // Handle rfData as needed
      } else if (packet.frame_data[0] == 0x8B) { // Transmit Status
          // Process transmit status
          XBEE_TransmittedReceived += 1;
          uint8_t frameID = packet.frame_data[1];
          transmitStatus = packet.frame_data[5];
          // Handle transmit status as needed
      }
    }
  }
  /* USER CODE END StartInterruptTask */
}

void StartSDTask(void *argument)
{
  /* USER CODE BEGIN StartSDTask */
  SD_Mount();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; // 10 Hz
  /* Infinite loop */
  for(;;) {
    SD_SaveBuffer();
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartSDTask */
}

/* USER CODE END Application */

