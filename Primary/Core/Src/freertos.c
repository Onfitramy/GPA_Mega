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
#include "arm_math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include "LSM6DSR.h"
#include "ISM330DHCX.h"
#include "LIS3MDL.h"
#include "bmp390.h"
#include "SAM-M8Q.h"
#include "SERVO.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
LSM6DSR_Data_t imu1_data;
ISM330DHCX_Data_t imu2_data;
LIS3MDL_Data_t mag_data;
float alpha;
StateVector GPA_SV;

uint8_t SelfTest_Bitfield = 0; //Bitfield for external Devices 0: IMU1, 1: IMU2, 2: MAG, 3: BARO, 4: GPS, 7:All checks passed
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
StreamBufferHandle_t xStreamBuffer;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 24,
  .priority = (osPriority_t) osPriorityNormal,
};

/*Commandline Handler*/
osThreadId_t cmdLineTaskHandle; // new command line task
const osThreadAttr_t cmdLineTask_attributes = {
  .name = "cmdLineTask", // defined in cli_app.c
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 32
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t SelfTest(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  cmdLineTaskHandle = osThreadNew(vCommandConsoleTask, NULL, &cmdLineTask_attributes);
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
  // Example matrix multiplication using arm_math
  arm_matrix_instance_f32 matA, matB, matC;
  float32_t dataA[4] = {1.0, 2.0, 3.0, 4.0}; // 2x2 matrix
  float32_t dataB[4] = {5.0, 6.0, 7.0, 8.0}; // 2x2 matrix
  float32_t dataC[4]; // Resultant 2x2 matrix

  // Initialize matrices
  arm_mat_init_f32(&matA, 2, 2, dataA);
  arm_mat_init_f32(&matB, 2, 2, dataB);
  arm_mat_init_f32(&matC, 2, 2, dataC);

  // Perform matrix multiplication
  if (arm_mat_mult_f32(&matA, &matB, &matC) == ARM_MATH_SUCCESS) {
    uint8_t test = 1;// Matrix multiplication successful, dataC contains the result
  } else {
    // Handle error
  }
  /* Infinite loop */
  for(;;) {
    SelfTest(); // Run self-test on startup

    IMU1_ReadSensorData(&imu1_data);
    IMU2_ReadSensorData(&imu2_data);

    if(MAG_VerifyDataReady() & 0b00000001) {
      MAG_ReadSensorData(&mag_data);
    }

    alpha = atan2(imu2_data.accel[1], imu2_data.accel[2])*180/M_PI;
    SERVO_MoveToAngle(1, 2*alpha);
  }

  
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

uint8_t SelfTest(void) {
  uint8_t R = 255;
  uint8_t G = 0;
  uint8_t B = 0;

  if(SelfTest_Bitfield == 0b11111 && SelfTest_Bitfield != 0b10011111){
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

