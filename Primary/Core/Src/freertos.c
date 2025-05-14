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
ubx_nav_posllh_t gps_data;
StateVector GPA_SV;
uint32_t pressure_raw;

uint8_t SelfTest_Bitfield = 0; //Bitfield for external Devices 0: IMU1, 1: IMU2, 2: MAG, 3: BARO, 4: GPS, 7:All checks passed

float M_rot_data[9];
arm_matrix_instance_f32 M_rot = {3, 3, M_rot_data};
float M_rotUpdate_data[9];
arm_matrix_instance_f32 M_rotUpdate = {3, 3, M_rotUpdate_data};
float M_rot_new_data[9];
arm_matrix_instance_f32 M_rot_new = {3, 3, M_rot_new_data};
float M_rot_fix_data[9];
arm_matrix_instance_f32 M_rot_fix = {3, 3, M_rot_fix_data};
float M_rot_inv_data[9];
arm_matrix_instance_f32 M_rot_inv = {3, 3, M_rot_inv_data};

// euler angles
float phi, theta, psi;
float euler_deg[3];

// euler angles from accelerations and magnetic field
float phi_fix, theta_fix, psi_fix;
float euler_fix[3];

float dt = 0.001;

float c1[3], c2[3], c3[3];

// unit basis vectors of inertial system expressed in body coordinates
float base_xi[3];
float base_yi[3];
float base_zi[3];

// normalized acceleration and magnetic field vectors
float a_norm[3];
float m_norm[3];

// Variables for rate gyro calibration
float gyr_sumup[3] = {0, 0, 0};
float gyr_offset[3] = {0, 0, 0};
int calib_counter = 0;

int isInitialized = 0;

// PID STUFF
// euler angles from control
float euler_set[3] = {90, 0, 0};

float K[3] = {1, 1, 0.2};
float Tv[3] = {1, 1, 1};
float Tn[3] = {1, 1, 100};

float Ki[3], Kd[3];

float d_euler[3];
float d_euler_prior[3];
float I_sum[3] = {0, 0, 0};
float P_term[3], I_term[3], D_term[3];

float PID_out[3];

float a_WorldFrame[3];              // Acceleration
float v_WorldFrame[3] = {0, 0, 0};  // Velocity
float r_WorldFrame[3] = {0, 0, 0};  // Position

float V3B[3];

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
  .stack_size = 128 * 48,
  .priority = (osPriority_t) osPriorityHigh,
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
  arm_mat_set_entry_f32(&M_rot, 0, 0, 1);
  arm_mat_set_entry_f32(&M_rot, 1, 1, 1);
  arm_mat_set_entry_f32(&M_rot, 2, 2, 1);

  signalPlotter_setSignalName(0, "MAG_X");
  signalPlotter_setSignalName(1, "MAG_Y");
  signalPlotter_setSignalName(2, "MAG_Z");
  signalPlotter_setSignalName(3, "ACC_X");
  signalPlotter_setSignalName(4, "ACC_Y");
  signalPlotter_setSignalName(5, "ACC_Z");
  signalPlotter_setSignalName(6, "GYR_X");
  signalPlotter_setSignalName(7, "GYR_Y");
  signalPlotter_setSignalName(8, "GYR_Z");
  signalPlotter_setSignalName(9, "phi");
  signalPlotter_setSignalName(10, "theta");
  signalPlotter_setSignalName(11, "psi");
  signalPlotter_setSignalName(12, "phi fix");
  signalPlotter_setSignalName(13, "theta fix");
  signalPlotter_setSignalName(14, "psi fix");
  signalPlotter_setSignalName(15, "ACC_X World");
  signalPlotter_setSignalName(16, "ACC_Y World");
  signalPlotter_setSignalName(17, "ACC_Z World");
  signalPlotter_setSignalName(18, "VEL_X World");
  signalPlotter_setSignalName(19, "VEL_Y World");
  signalPlotter_setSignalName(20, "VEL_Z World");

  signalPlotter_setSignalName(31, "delta Time");
  
  arm_vec3_element_product_f32(K, Tv, Kd);
  Tn[0] = 1 / Tn[0];
  Tn[1] = 1 / Tn[1];
  Tn[2] = 1 / Tn[2];
  arm_vec3_element_product_f32(K, Tn, Ki);

  /* Infinite loop */
  for(;;) {
    TimeMeasureStart(); // Start measuring time
    SelfTest(); // Run self-test on startup
    //BMP_GetPressureRaw(&pressure_raw);
    if(IMU1_VerifyDataReady() & 0x03 == 0x03) {
      IMU1_ReadSensorData(&imu1_data);
      arm_sub_f32(imu1_data.accel, IMU1_offset, imu1_data.accel, 3);
      arm_mult_f32(imu1_data.accel, IMU1_scale, imu1_data.accel, 3);
      
      if (calib_counter < 1000) {
        calib_counter++;
        arm_vec3_add_f32(gyr_sumup, imu1_data.gyro, gyr_sumup);
      }

      else if (calib_counter == 1000) {
        calib_counter++;
        // calculate offset based on 1000 samples
        arm_vec3_scalar_mult_f32(gyr_sumup, 1. / 1000., gyr_offset);
      }
      arm_vec3_sub_f32(imu1_data.gyro, gyr_offset, imu1_data.gyro);
    }
    
    //IMU2_ReadSensorData(&imu2_data);

    if(MAG_VerifyDataReady() & 0b00000001) {
      MAG_ReadSensorData(&mag_data);
      arm_sub_f32(mag_data.field, MAG_offset, mag_data.field, 3);
      arm_mult_f32(mag_data.field, MAG_scale, mag_data.field, 3);
    }

    // attitude estimation using gyroscopes and rotation matrix
    arm_mat_set_entry_f32(&M_rotUpdate, 0, 0, 1);
    arm_mat_set_entry_f32(&M_rotUpdate, 0, 1, dt * imu1_data.gyro[2]);
    arm_mat_set_entry_f32(&M_rotUpdate, 0, 2, -dt * imu1_data.gyro[1]);
    arm_mat_set_entry_f32(&M_rotUpdate, 1, 0, -dt * imu1_data.gyro[2]);
    arm_mat_set_entry_f32(&M_rotUpdate, 1, 1, 1);
    arm_mat_set_entry_f32(&M_rotUpdate, 1, 2, dt * imu1_data.gyro[0]);
    arm_mat_set_entry_f32(&M_rotUpdate, 2, 0, dt * imu1_data.gyro[1]);
    arm_mat_set_entry_f32(&M_rotUpdate, 2, 1, -dt * imu1_data.gyro[0]);
    arm_mat_set_entry_f32(&M_rotUpdate, 2, 2, 1);

    arm_mat_mult_f32(&M_rotUpdate, &M_rot, &M_rot_new);
    arm_mat_copy_f32(&M_rot_new, &M_rot);

    arm_mat_get_column_f32(&M_rot, 0, c1);
    arm_mat_get_column_f32(&M_rot, 1, c2);
    arm_mat_get_column_f32(&M_rot, 2, c3);

    arm_vec3_cross_product_f32(c1, c2, c3);
    arm_vec3_cross_product_f32(c2, c3, c1);
    arm_vec3_normalize_f32(c1);
    arm_vec3_normalize_f32(c2);
    arm_vec3_normalize_f32(c3);

    arm_mat_set_column_f32(&M_rot, 0, c1);
    arm_mat_set_column_f32(&M_rot, 1, c2);
    arm_mat_set_column_f32(&M_rot, 2, c3);

    phi = atan2(arm_mat_get_entry_f32(&M_rot, 1, 2), arm_mat_get_entry_f32(&M_rot, 2, 2));
    theta = asin(-arm_mat_get_entry_f32(&M_rot, 0, 2));
    psi = atan2(arm_mat_get_entry_f32(&M_rot, 0, 1), arm_mat_get_entry_f32(&M_rot, 0, 0));

    euler_deg[0] = phi * 180. / M_PI;
    euler_deg[1] = theta * 180. / M_PI;
    euler_deg[2] = psi * 180. / M_PI;

    // attitude estimation using magnetometer and accelerometer
    // normalize a and m vectors
    arm_vec3_copy_f32(imu1_data.accel, base_zi);
    arm_vec3_copy_f32(mag_data.field, m_norm);
    arm_vec3_normalize_f32(base_zi);
    arm_vec3_normalize_f32(m_norm);

    // calculate unit basis vectors
    arm_vec3_cross_product_f32(base_zi, m_norm, base_yi);
    arm_vec3_normalize_f32(base_yi);
    arm_vec3_cross_product_f32(base_yi, base_zi, base_xi);

    arm_mat_set_column_f32(&M_rot_fix, 0, base_xi);
    arm_mat_set_column_f32(&M_rot_fix, 1, base_yi);
    arm_mat_set_column_f32(&M_rot_fix, 2, base_zi);

    euler_fix[0] = phi_fix = atan2(base_zi[1], base_zi[2]);
    euler_fix[1] = theta_fix = asin(-base_zi[0]);
    euler_fix[2] = psi_fix = atan2(base_yi[0], base_xi[0]);

    if(isInitialized == 0) {
      arm_mat_copy_f32(&M_rot_fix, &M_rot);
      isInitialized = 1;
    }

    arm_mat_trans_f32(&M_rot, &M_rot_inv);
    arm_mat_vec_mult_f32(&M_rot_inv, imu1_data.accel, a_WorldFrame);

    a_WorldFrame[2] -= 9.80665;
    arm_vec3_scalar_mult_f32(a_WorldFrame, dt, V3B);
    arm_vec3_add_f32(v_WorldFrame, V3B, v_WorldFrame);

    // PID __ phi = x | theta = z | psi = y
    arm_vec3_copy_f32(d_euler, d_euler_prior);
    arm_vec3_sub_f32(euler_set, euler_deg, d_euler);
    arm_vec3_add_f32(I_sum, d_euler, I_sum);

    arm_vec3_element_product_f32(K, d_euler, P_term);
    arm_vec3_sub_f32(d_euler, d_euler_prior, D_term);
    arm_vec3_scalar_mult_f32(D_term, 1 / dt, D_term);
    arm_vec3_element_product_f32(Kd, D_term, D_term);
    arm_vec3_scalar_mult_f32(I_sum, dt, I_term);
    arm_vec3_element_product_f32(Ki, I_term, I_term);

    arm_vec3_add_f32(P_term, I_term, PID_out);
    arm_vec3_add_f32(PID_out, D_term, PID_out);

    SERVO_TVC(PID_out[0], PID_out[1], PID_out[2]);

    #ifdef RECEIVER
      //nrf24l01p_rx_receive(rx_data);
      uint8_t test = nrf24l01p_get_status();
    #endif

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
    signalPlotter_sendData(9, phi);
    signalPlotter_sendData(10, theta);
    signalPlotter_sendData(11, psi);
    signalPlotter_sendData(12, phi_fix);
    signalPlotter_sendData(13, theta_fix);
    signalPlotter_sendData(14, psi_fix);
    signalPlotter_sendData(15, a_WorldFrame[0]);
    signalPlotter_sendData(16, a_WorldFrame[1]);
    signalPlotter_sendData(17, a_WorldFrame[2]);
    signalPlotter_sendData(18, v_WorldFrame[0]);
    signalPlotter_sendData(19, v_WorldFrame[1]);
    signalPlotter_sendData(20, v_WorldFrame[2]);

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

