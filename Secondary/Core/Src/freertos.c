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
#include "stream_buffer.h"
#include "semphr.h"
#include "queue.h"
#include "string.h"
#include "fatfs.h"

#include "ws2812.h"
#include "W25Q1.h"
#include "xBee.h"
#include "NRF24L01P.h"
#include "VoltageReader.h"
#include "InterBoardCom.h"
#include "status.h"
#include "SD.h"
#include "PowerUnit.h"
#include "statemachine.h"
#include "radio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern UART_HandleTypeDef huart1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
QueueHandle_t InterBoardPacketQueue;
QueueHandle_t XBeeDataQueue;
QueueHandle_t InterruptQueue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
SemaphoreHandle_t flashSemaphore;

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

uint32_t Counter_100Hz = 0;
uint32_t Counter_10Hz = 0;

health_t health;
health_t health_filtered;

extern radio_info_t radio_info;

uint8_t rx_recieve_buf[NRF24L01P_PAYLOAD_LENGTH] = {0};

uint64_t XBee_transmit_addr = 0x0013a200426e530e; // XBee transmit address (to groundstation)
//uint64_t XBee_transmit_addr = 0x0013a200426b848b; // XBee transmit address (to flight controller)
//uint64_t XBee_transmit_addr = 0x0013a200426e52e7; // XBee transmit address (to flight controller)
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
  flashSemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  InterBoardPacketQueue = xQueueCreate(64, sizeof(InterBoardPacket_t)); // Queue for 64 InterBoardPacket_t packets
  XBeeDataQueue = xQueueCreate(4, sizeof(xbee_frame_t)); // Queue for 4 XBee data packets
  InterruptQueue = xQueueCreate(10, sizeof(uint8_t)); // Queue for 10 interrupt signals
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

    StateMachine_DoActions(&pu_sm, 100);

    ShowStatus(pu_sm.currentState, 1, 100);
    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 100Hz
  }

  /* USER CODE END StartDefaultTask */
}

void Start10HzTask(void *argument){
  /* USER CODE BEGIN Start10HzTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; // 10 Hz

  /* Infinite loop */
  for(;;) {
    Counter_10Hz++;

    health.temperature.reg_3V3 = readTemperature(1);
    health.temperature.battery = readTemperature(2);
    health.voltage.bus_5V = readVoltage(1) * (10 + 10) / 10;
    health.voltage.bus_gpa_bat = readVoltage(2) * (10 + 2.2) / 2.2;
    //FilterLP(&health, &health_filtered);

    StateMachine_DoActions(&pu_sm, 10);

    DataPacket_t powerData;
    powerData.Packet_ID = PACKET_ID_POWER; // Power Packet ID

    UpdatePowerPacket(&powerData,
                      xTaskGetTickCount() * portTICK_PERIOD_MS,
                      health.voltage.bus_pu_bat,
                      health.power.out_pu,
                      health.current.out_pu,
                      health.voltage.bus_5V,
                      health.voltage.bus_gpa_bat);

    InterBoardPacket_t powerPacket = InterBoardCom_CreatePacket(INTERBOARD_OP_SAVE_SEND | INTERBOARD_TARGET_MCU);
    InterBoardCom_FillData(&powerPacket, &powerData);
    InterBoardCom_SendPacket(&powerPacket);

    XBee_TransmitQueue(XBee_transmit_addr); // Transmit data at 10Hz

    // transmit data
    uint8_t tx_buf[NRF24L01P_PAYLOAD_LENGTH] = {0}; // Initialize to zero
    tx_buf[0] = 0xAA; // Packet ID or something meaningful
    tx_buf[1] = HAL_GetTick() & 0xFF;
    tx_buf[2] = (HAL_GetTick() >> 8) & 0xFF;
    radioSend(tx_buf);

    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 10Hz
  }

  /* USER CODE END Start10HzTask */
}

uint16_t receivedPackets = 0;

void StartInterBoardComTask(void *argument)
{
  /* USER CODE BEGIN StartInterBoardComTask */
  W25Q_GetConfig();
  
  //XBee_Init();

  InterBoardCom_Init();
  /* Infinite loop */
  for(;;) {
    InterBoardPacket_t packet;
    // Calculate 0.1ms in ticks based on configTICK_RATE_HZ
    if (xQueueReceive(InterBoardPacketQueue, &packet, 1) == pdPASS) { // 1ms delay
      InterBoardCom_ParsePacket(packet);
    }
  }
  /* USER CODE END StartInterBoardComTask */
}

uint8_t transmitStatus;
uint32_t XBEE_TransmittedReceived = 0;
uint32_t TransmissionErrors = 0;
uint16_t radioTime = 0;
void StartInterruptTask(void *argument)
{
  /* USER CODE BEGIN StartInterruptTask */

  QueueSetHandle_t xQueueSet = xQueueCreateSet(20); // Total items from both queues
  xQueueAddToSet(InterruptQueue, xQueueSet);
  xQueueAddToSet(XBeeDataQueue, xQueueSet);
  /* Infinite loop */
  for(;;) {

     // Wait on the queue set with timeout (blocks efficiently)
    QueueSetMemberHandle_t xActivatedMember = xQueueSelectFromSet(xQueueSet, portMAX_DELAY);

    if (xActivatedMember == InterruptQueue) {
      uint8_t int_pin;
      //HAL_GPIO_TogglePin(M2_LED_GPIO_Port, M2_LED_Pin);
      if (xQueueReceive(InterruptQueue, &int_pin, 0) == pdPASS) {
        if (int_pin == NRF24_INT_Pin) {
          uint8_t status = nrf24l01p_get_status();
          nrf24l01p_clear_known_irqs(status);
          if (status & 0x40) { // Data Ready RX FIFO interrupt
            nrf24l01p_read_rx_fifo(rx_recieve_buf);
            radioTime = (rx_recieve_buf[1] << 8) | rx_recieve_buf[2];
          } else if (status & 0x20) { // Data Sent TX FIFO interrupt
            uint8_t fifo_status = nrf24l01p_get_fifo_status();
            if ((fifo_status & 0x10) && radio_info.mode == RADIO_MODE_TRANSCEIVER) { // TX FIFO empty
              nrf24l01p_rxMode(); // Switch back to RX mode
            }
          }
        }
      }
    } else if (xActivatedMember == XBeeDataQueue) {
      xbee_frame_t packet;
      if (xQueueReceive(XBeeDataQueue, (uint8_t*)&packet, 0) == pdPASS) {
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
          } else if (packet.frame_data[2] == 'S' && packet.frame_data[3] == 'L') { // DD command
            // Device Identifier Response
            if (packet.frame_data[4] == 0x00) { // Check if command was successful
              // Process device identifier if needed
              uint8_t deviceID = packet.frame_data[6];
            } else {
              // Handle error
            }
          } else if (packet.frame_data[2] == 'B' && packet.frame_data[3] == 'R'){ //Received Channel Mask (Change this to whatever Packet you are interested in)
            // Check if command was successful
            if (packet.frame_data[4] == 0x00) {
                uint16_t channelMask = (packet.frame_data[6] << 8) | packet.frame_data[7];
                xBee_changeState(0); // Set state to OK
                // Process channel mask as needed
            }
          }
        }  else if (packet.frame_data[0] == 0x90) { // RX Packet
          XBEE_TransmittedReceived += 1;
          HAL_GPIO_TogglePin(M2_LED_GPIO_Port, M2_LED_Pin); // Toggle M2 LED to indicate transmitting
          XBee_parseReceivedRFFrame(&packet);
          // Handle rfData as needed
        } else if (packet.frame_data[0] == 0x8B) { // Transmit Status
          // Process transmit status
          HAL_GPIO_TogglePin(M2_LED_GPIO_Port, M2_LED_Pin); // Toggle M2 LED to indicate transmitting
          uint8_t frameID = packet.frame_data[1];
          transmitStatus = packet.frame_data[5];
          if (transmitStatus != 0x00) {
            TransmissionErrors += 1;
          } else {
            // Transmission successful
            XBEE_TransmittedReceived += 1;
          }
          // Handle transmit status as needed
        }
      }
    }
  }
  /* USER CODE END StartInterruptTask */
}

void StartSDTask(void *argument)
{
  /* USER CODE BEGIN StartSDTask */
  if (SD_Mount() == FR_OK) {
    // Successfully mounted SD card
  } else {
    // Failed to mount SD card
    // vTaskDelete(NULL); // Delete this task if SD card cannot be mounted
  }

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; // 10 Hz
  /* Infinite loop */
  for(;;) {
    // SD_SaveBuffer();

    if (xSemaphoreTake(flashSemaphore, portMAX_DELAY) == pdTRUE) {
      W25Q_WriteFlashBuffer();
    }
  }
  /* USER CODE END StartSDTask */
}

/* USER CODE END Application */

