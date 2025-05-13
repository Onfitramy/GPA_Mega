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
#include "cli_app.h"
#include "stream_buffer.h"
#include "armMathAddon.h"
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
#include "SERVO.h"
#include "NRF24L01P.h"

#include "signalPlotter.h"
#include "calibration_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
LSM6DSR_Data_t imu1_data;
ISM330DHCX_Data_t imu2_data;
LIS3MDL_Data_t mag_data;
UBX_NAV_PVT gps_data;
float alpha;
StateVector GPA_SV;
uint32_t pressure_raw;
float yaw = 0;

uint8_t SelfTest_Bitfield = 0; //Bitfield for external Devices 0: IMU1, 1: IMU2, 2: MAG, 3: BARO, 4: GPS, 7:All checks passed

#ifdef TRANSMITTER
  uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH] = {0}; //Bit(Payload Lenght) array to store sending data
#endif
#ifdef RECEIVER
  uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = {0};
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
StreamBufferHandle_t xStreamBuffer;
QueueHandle_t InterruptQueue;
/* USER CODE END Variables */
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
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t Hz100TaskHandle;
const osThreadAttr_t Hz100Task_attributes = {
  .name = "100HzTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/*Commandline Handler*/
osThreadId_t cmdLineTaskHandle; // new command line task
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t SelfTest(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void StartInterruptHandlerTask(void *argument);

void Start10HzTask(void *argument);

void Start100HzTask(void *argument);

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
  xStreamBuffer = xStreamBufferCreate(50, 1); // 1-byte trigger level
  if (xStreamBuffer == NULL) {
    // Handle stream buffer creation failure
  }
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
  Hz10TaskHandle = osThreadNew(Start10HzTask, NULL, &Hz10Task_attributes);
  Hz100TaskHandle = osThreadNew(Start100HzTask, NULL, &Hz100Task_attributes);

  cmdLineTaskHandle = osThreadNew(vCommandConsoleTask, NULL, &cmdLineTask_attributes);
  InterruptHandlerTaskHandle = osThreadNew(StartInterruptHandlerTask, NULL, &InterruptHandlerTask_attributes);
  if (InterruptHandlerTaskHandle == NULL) {
    InterruptHandlerTaskHandle = NULL;
  }

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
  GPS_Init(); //Initialize the GPS module
  /* USER CODE BEGIN StartDefaultTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1; //1000 Hz
  signalPlotter_setSignalName(0, "MAG_X");
  signalPlotter_setSignalName(1, "MAG_Y");
  signalPlotter_setSignalName(2, "MAG_Z");
  signalPlotter_setSignalName(3, "ACC_X");
  signalPlotter_setSignalName(4, "ACC_Y");
  signalPlotter_setSignalName(5, "ACC_Z");
  signalPlotter_setSignalName(6, "GYR_X");
  signalPlotter_setSignalName(7, "GYR_Y");
  signalPlotter_setSignalName(8, "GYR_Z");
  signalPlotter_setSignalName(9, "yaw");
  signalPlotter_setSignalName(31, "delta Time");

  /* Infinite loop */
  for(;;) {
    TimeMeasureStart(); // Start measuring time
    SelfTest(); // Run self-test on startup
    BMP_GetPressureRaw(&pressure_raw);
    IMU1_ReadSensorData(&imu1_data);
    arm_sub_f32(imu1_data.accel, IMU1_offset, imu1_data.accel, 3);
    arm_mult_f32(imu1_data.accel, IMU1_scale, imu1_data.accel, 3);
    IMU2_ReadSensorData(&imu2_data);

    if(MAG_VerifyDataReady() & 0b00000001) {
      MAG_ReadSensorData(&mag_data);
      arm_sub_f32(mag_data.field, MAG_offset, mag_data.field, 3);
      arm_mult_f32(mag_data.field, MAG_scale, mag_data.field, 3);
    }

    yaw += imu1_data.gyro[2] * 0.001;

    alpha = atan2(imu2_data.accel[1], imu2_data.accel[2])*180/M_PI;
    SERVO_MoveToAngle(1, 2*alpha);
    TimeMeasureStop(); // Stop measuring time
    vTaskDelayUntil( &xLastWakeTime, xFrequency); // Delay for 1ms (1000Hz)
  }

  
  /* USER CODE END StartDefaultTask */
}

void Start100HzTask(void *argument) {
  /* USER CODE BEGIN Start100HzTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10; //100 Hz
  /* Infinite loop */
  for(;;) {
    signalPlotter_sendData(0, mag_data.field[0]);
    signalPlotter_sendData(1, mag_data.field[1]);
    signalPlotter_sendData(2, mag_data.field[2]);
    signalPlotter_sendData(3, imu1_data.accel[0]);
    signalPlotter_sendData(4, imu1_data.accel[1]);
    signalPlotter_sendData(5, imu1_data.accel[2]);
    signalPlotter_sendData(6, imu1_data.gyro[0]);
    signalPlotter_sendData(7, imu1_data.gyro[1]);
    signalPlotter_sendData(8, imu1_data.gyro[2]);
    signalPlotter_sendData(9, yaw);

    signalPlotter_executeTransmission(HAL_GetTick());
    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 100Hz
  }
  /* USER CODE END Start10HzTask */
}

void Start10HzTask(void *argument) {
  /* USER CODE BEGIN Start10HzTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; //10 Hz
  /* Infinite loop */
  for(;;) {
    GPS_ReadSensorData(&gps_data);

    #ifdef TRANSMITTER
      nrf24l01p_tx_transmit(tx_data);
    #endif

    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 10Hz
  }
  /* USER CODE END Start10HzTask */
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
  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //Disabled due to always triggering when GPS is not connected
  /* Infinite loop */
  for(;;)
  { 
    if (xQueueReceive(InterruptQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
      if(receivedData == 0x10) {
        ublox_ReadOutput(GPS_Buffer); //Read the GPS output and decode it
      }else if (receivedData == 0x11) {
        HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
        #ifdef RECEIVER
          nrf24l01p_rx_receive(rx_data);
          if(nrf24l01p_get_receivedPower()) {
            Set_LED(0, 0, 255, 0);
            Set_Brightness(45);
            WS2812_Send();
          } else {
            Set_LED(0, 255, 0, 0);
            Set_Brightness(45);
            WS2812_Send();
          }
        #endif
    
        #ifdef TRANSMITTER
          nrf24l01p_tx_irq();
        #endif
      }
    }
    osDelay(5);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

uint8_t SelfTest(void) {
  uint8_t R = 255;
  uint8_t G = 0;
  uint8_t B = 0;

  if((SelfTest_Bitfield == 0b11111) && (SelfTest_Bitfield != 0b10011111)){
    R = 0;
    G = 255;
    B = 0;
    Set_LED(0, R, G, B);
    Set_Brightness(5);
    WS2812_Send();
    SelfTest_Bitfield |= (1<<7);  //All checks passed
  }
  else if(SelfTest_Bitfield != 0b10011111){
    SelfTest_Bitfield |= IMU1_SelfTest();
    SelfTest_Bitfield |= (IMU2_SelfTest()<<1);
    SelfTest_Bitfield |= (MAG_SelfTest()<<2);
    SelfTest_Bitfield |= (BMP_SelfTest()<<3);
    SelfTest_Bitfield |= (GPS_VER_CHECK()<<4); //Check if GPS is connected and working
    
    R = 255;
    G = 0;
    B = 0;
    Set_LED(0, R, G, B);
    Set_Brightness(5);
    WS2812_Send();
  }

  return SelfTest_Bitfield;
}

/* USER CODE END Application */

