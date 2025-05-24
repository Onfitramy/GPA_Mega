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

#include "navigation.h"
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
StateVector GPA_SV;
uint32_t pressure_raw;
uint32_t temperature_raw;
bmp390_handle_t bmp_handle;
float temperature, pressure;

int counter = 0;

uint8_t SelfTest_Bitfield = 0; //Bitfield for external Devices 0: IMU1, 1: IMU2, 2: MAG, 3: BARO, 4: GPS, 7:All checks passed

float M_rot_fix_data[9];
arm_matrix_instance_f32 M_rot_fix = {3, 3, M_rot_fix_data};

// euler angles
float phi, theta, psi;
float euler_deg[3];

// euler angles from accelerations and magnetic field
float phi_fix, theta_fix, psi_fix;

float c1[3], c2[3], c3[3];

// unit basis vectors of inertial system expressed in body coordinates
float base_xi[3];
float base_yi[3];
float base_zi[3];

// normalized acceleration and magnetic field vectors
float a_norm[3];
float m_norm[3];

// Variables for calibration
float gyr_sumup[3] = {0, 0, 0};
float gyr_offset[3] = {0, 0, 0};
float acc_sumup[3] = {0, 0, 0};
float mag_sumup[3] = {0, 0, 0};

int isInitialized = 0;

// PID STUFF
// euler angles from control
float euler_set[3] = {90, 0, 0};

float K[3] = {2, 2, 0.5};
float Tv[3] = {0.5, 0.5, 0.5};
float Tn[3] = {10, 10, 100};

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

float throttle_cmd = 0;

uint8_t offset_phi = 127;
uint8_t offset_theta = 127;

float phi_prior, theta_prior, psi_prior;
int phi_rot_count = 0, theta_rot_count = 0, psi_rot_count = 0;

// Kalman Filter
// time step
float dt = 0.001;

// state and output vectors
float x[6];
float dx[6];
float z[3];
float dz[3];

float A_data[36] = {0}; // 6x6
arm_matrix_instance_f32 A = {6, 6, A_data};
float Atrans_data[36];  // 6x6
arm_matrix_instance_f32 Atrans = {6, 6, Atrans_data};
float B_data[18] = {0}; // 6x3
arm_matrix_instance_f32 B = {6, 3, B_data};
float Q_data[36] = {0}; // 6x6
arm_matrix_instance_f32 Q = {6, 6, Q_data};
float P_data[36] = {0}; // 6x6
arm_matrix_instance_f32 P = {6, 6, P_data};
float Mtmp_data[36];
arm_matrix_instance_f32 Mtmp = {6, 6, Mtmp_data};
float Mdeuler_data[9] = {0};
arm_matrix_instance_f32 Mdeuler = {3, 3, Mdeuler_data};
float C_data[18] = {0}; // 3x6
arm_matrix_instance_f32 C = {3, 6, C_data};
float R_data[9] = {0};  // 3x3
arm_matrix_instance_f32 R = {3, 3, R_data};
float Ctrans_data[18];  // 6x3
arm_matrix_instance_f32 Ctrans = {6, 3, Ctrans_data};
float S_data[9];        // 3x3
arm_matrix_instance_f32 S = {3, 3, S_data};
float Sinv_data[9];     // 3x3
arm_matrix_instance_f32 Sinv = {3, 3, Sinv_data};
float Kgain_data[18];   // 6x3
arm_matrix_instance_f32 Kgain = {6, 3, Kgain_data};
float P_Ctrans_data[18];// 6x3
arm_matrix_instance_f32 P_Ctrans = {6, 3, P_Ctrans_data};
float C_P_data[18];     // 3x6
arm_matrix_instance_f32 C_P = {3, 6, C_P_data};

#pragma pack(push, 1)
typedef struct {
  float float1;
  float float2;
  float float3;
  float float4;
  float float5;
  float float6;
  float float7;
  float float8;
} Data_Package_Send;
#pragma pack(pop)

Data_Package_Send tx_data;

Data_Package_Receive rx_data;

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
  signalPlotter_setSignalName(9, "phi");
  signalPlotter_setSignalName(10, "theta");
  signalPlotter_setSignalName(11, "psi");
  signalPlotter_setSignalName(12, "phi fix");
  signalPlotter_setSignalName(13, "theta fix");
  signalPlotter_setSignalName(14, "psi fix");
  signalPlotter_setSignalName(15, "p offset");
  signalPlotter_setSignalName(16, "q offset");
  signalPlotter_setSignalName(17, "r offset");
  signalPlotter_setSignalName(18, "uncertainty");
  signalPlotter_setSignalName(19, "0");
  signalPlotter_setSignalName(20, "0");
  signalPlotter_setSignalName(21, "0");
  signalPlotter_setSignalName(22, "NRF timeout");
  signalPlotter_setSignalName(23, "delta Time");
  
  arm_vec3_element_product_f32(K, Tv, Kd);
  Tn[0] = 1 / Tn[0];
  Tn[1] = 1 / Tn[1];
  Tn[2] = 1 / Tn[2];
  arm_vec3_element_product_f32(K, Tn, Ki);

  for(int i = 0; i < 3; i++) {
    arm_mat_set_entry_f32(&P, i, i, 1);
    arm_mat_set_entry_f32(&P, i+3, i+3, 0.001);
    arm_mat_set_entry_f32(&Q, i, i, 1e-8);
    arm_mat_set_entry_f32(&Q, i+3, i+3, 1e-12);
    arm_mat_set_entry_f32(&C, i, i, 1);
    arm_mat_set_entry_f32(&R, i, i, 1);
  }

  arm_mat_set_entry_f32(&Mdeuler, 0, 0, 1);

  // calib 
  
  for(int calib_counter = 0; calib_counter < 1000; calib_counter++) { // imu
    while(IMU1_VerifyDataReady() & 0x03 != 0x03); // wait for IMU1 data
    IMU1_ReadSensorData(&imu1_data);
    arm_vec3_sub_f32(imu1_data.accel, IMU1_offset, imu1_data.accel);
    arm_vec3_element_product_f32(imu1_data.accel, IMU1_scale, imu1_data.accel);
    arm_vec3_add_f32(gyr_sumup, imu1_data.gyro, gyr_sumup);
    arm_vec3_add_f32(acc_sumup, imu1_data.accel, acc_sumup);
    HAL_Delay(1);
  }
  arm_vec3_scalar_mult_f32(gyr_sumup, 1. / 1000., gyr_offset);
  arm_vec3_scalar_mult_f32(acc_sumup, 1. / 1000., acc_sumup);
  arm_vec3_sub_f32(imu1_data.gyro, gyr_offset, imu1_data.gyro);

  for(int calib_counter = 0; calib_counter < 100; calib_counter++) { // mag
    while(!(MAG_VerifyDataReady() & 0b00000001)); // wait for MAG data
    MAG_ReadSensorData(&mag_data);
    arm_vec3_sub_f32(mag_data.field, MAG_offset, mag_data.field);
    arm_vec3_element_product_f32(mag_data.field, MAG_scale, mag_data.field);
    arm_vec3_add_f32(mag_sumup, mag_data.field, mag_sumup);
    HAL_Delay(1);
  }
  arm_vec3_scalar_mult_f32(mag_sumup, 1. / 100., mag_sumup);

  // attitude estimation using magnetometer and accelerometer
  // normalize a and m vectors
  arm_vec3_copy_f32(acc_sumup, base_zi);
  arm_vec3_copy_f32(mag_sumup, m_norm);
  arm_vec3_normalize_f32(base_zi);
  arm_vec3_normalize_f32(m_norm);

  // calculate unit basis vectors
  arm_vec3_cross_product_f32(base_zi, m_norm, base_yi);
  arm_vec3_normalize_f32(base_yi);
  arm_vec3_cross_product_f32(base_yi, base_zi, base_xi);

  arm_mat_set_column_f32(&M_rot_fix, 0, base_xi);
  arm_mat_set_column_f32(&M_rot_fix, 1, base_yi);
  arm_mat_set_column_f32(&M_rot_fix, 2, base_zi);

  x[0] = z[0] = phi_fix = atan2(base_zi[1], base_zi[2]);
  x[1] = z[1] = theta_fix = asin(-base_zi[0]);
  x[2] = z[2] = psi_fix = atan2(base_yi[0], base_xi[0]);

  x[3] = gyr_offset[0];
  x[4] = gyr_offset[1];
  x[5] = gyr_offset[2];

  /* Infinite loop */
  for(;;) {
    TimeMeasureStart(); // Start measuring time
    SelfTest();         // Run self-test on startup
    nrf_timeout++;      // to detect loss of signal (LOS)
    BMP_GetPressureRaw(&pressure_raw);  
    BMP_GetTemperatureRaw(&temperature_raw);

    // Kompensierte Temperatur berechnen (°C * 100)
    temperature = bmp390_compensate_temperature(temperature_raw, &bmp_handle);  // float, z.B. °C
    pressure = bmp390_compensate_pressure(pressure_raw, &bmp_handle);  // in Pa

    if(IMU1_VerifyDataReady() & 0x03 == 0x03) {
      IMU1_ReadSensorData(&imu1_data);
      arm_vec3_sub_f32(imu1_data.accel, IMU1_offset, imu1_data.accel);
      arm_vec3_element_product_f32(imu1_data.accel, IMU1_scale, imu1_data.accel);
    }
    
    //IMU2_ReadSensorData(&imu2_data);

    if(MAG_VerifyDataReady() & 0b00000001) {
      MAG_ReadSensorData(&mag_data);
      arm_vec3_sub_f32(mag_data.field, MAG_offset, mag_data.field);
      arm_vec3_element_product_f32(mag_data.field, MAG_scale, mag_data.field);
    }

    // KALMAN FILTER

    // initialize A and B matrix
    arm_mat_set_entry_f32(&Mdeuler, 0, 1, sin(phi)*tan(theta));
    arm_mat_set_entry_f32(&Mdeuler, 0, 2, cos(phi)*tan(theta));
    arm_mat_set_entry_f32(&Mdeuler, 1, 1, cos(phi));
    arm_mat_set_entry_f32(&Mdeuler, 1, 2, -sin(phi));
    arm_mat_set_entry_f32(&Mdeuler, 2, 1, sin(phi)/cos(theta));
    arm_mat_set_entry_f32(&Mdeuler, 2, 2, cos(phi)/cos(theta));

    for(int i = 0; i < 6; i++) {
      arm_mat_set_entry_f32(&A, i, i, 1);
    }
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        arm_mat_set_entry_f32(&A, i, j+3, -dt*arm_mat_get_entry_f32(&Mdeuler, i, j));
        arm_mat_set_entry_f32(&B, i, j, dt*arm_mat_get_entry_f32(&Mdeuler, i, j));
      }
    }

    // predicting state vector x
    arm_mat_vec_mult_f32(&A, x, x);
    arm_mat_vec_mult_f32(&B, imu1_data.gyro, dx);
    arm_vecN_add_f32(6, x, dx, x);

    phi = x[0];
    theta = x[1];
    psi = x[2];

    // normalize prediction
    if(phi > PI) {
      phi -= 2*PI;
    } else if(phi < -PI) {
      phi += 2*PI;
    }

    if(theta > PI) {
      theta -= 2*PI;
    } else if(theta < -PI) {
      theta += 2*PI;
    }

    if(psi > PI) {
      psi -= 2*PI;
    } else if(psi < -PI) {
      psi += 2*PI;
    }

    x[0] = phi;
    x[1] = theta;
    x[2] = psi;

    euler_deg[0] = phi * 180. / PI;
    euler_deg[1] = theta * 180. / PI;
    euler_deg[2] = psi * 180. / PI;

    // predicting state vector's covariance matrix P
    arm_mat_trans_f32(&A, &Atrans);
    arm_mat_mult_f32(&P, &Atrans, &Mtmp);
    arm_mat_mult_f32(&A, &Mtmp, &P);
    arm_mat_add_f32(&P, &Q, &P);

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

    phi_fix = atan2(base_zi[1], base_zi[2]);
    theta_fix = asin(-base_zi[0]);
    psi_fix = atan2(base_yi[0], base_xi[0]);

    phi_fix -= phi;
    if(phi_fix > PI) {
      phi_fix -= 2*PI;
    } else if(phi_fix < -PI) {
      phi_fix += 2*PI;
    }
    phi_fix += phi;

    theta_fix -= theta;
    if(theta_fix > PI) {
      theta_fix -= 2*PI;
    } else if(theta_fix < -PI) {
      theta_fix += 2*PI;
    }
    theta_fix += theta;

    psi_fix -= psi;
    if(psi_fix > PI) {
      psi_fix -= 2*PI;
    } else if(psi_fix < -PI) {
      psi_fix += 2*PI;
    }
    psi_fix += psi;

    z[0] = phi_fix;
    z[1] = theta_fix;
    z[2] = psi_fix;
    
    if(rx_data.BN == 0) {
      offset_phi = rx_data.PR;
      offset_theta = rx_data.PL;
    } else if(rx_data.BN == 1) {
      K[0] = K[1] = (float)rx_data.PL / 50.;
      Kd[0] = Kd[1] = (float)rx_data.PR / 50.;
    } else if(rx_data.BN == 2) {
      K[2] = (float)rx_data.PL / 50.;
      Kd[2] = (float)rx_data.PR / 50.;
    }

    if(nrf_timeout < 100) {
      if(rx_data.BN < 3) { // garbage filter
        euler_set[0] = (rx_data.JRY - offset_phi) / 5. + 90;
        euler_set[1] = (rx_data.JRX - offset_theta) / 5.;
        euler_set[2] += -2. * dt * (rx_data.JLX - 127);
        if(euler_set[2] > 180) {
          euler_set[2] -= 360;
        } else if(euler_set[2] < -180) {
          euler_set[2] += 360;
        }
        throttle_cmd = rx_data.JLY / 2.55;

        SERVO_MoveToAngle(PY_MOTOR, throttle_cmd * 1.8);
        SERVO_MoveToAngle(NY_MOTOR, throttle_cmd * 1.8);
      }
    } else {
      euler_set[0] = 90;
      euler_set[1] = 0;
      euler_set[2] = 0;

      SERVO_MoveToAngle(PY_MOTOR, 0);
      SERVO_MoveToAngle(NY_MOTOR, 0);
    }

    // PID __ phi = x | theta = z | psi = y
    arm_vec3_copy_f32(d_euler, d_euler_prior);
    arm_vec3_sub_f32(euler_set, euler_deg, d_euler);
    if(throttle_cmd > 10) {
      arm_vec3_add_f32(I_sum, d_euler, I_sum);
    } else {
      arm_vec3_sub_f32(I_sum, I_sum, I_sum);
    }

    arm_vec3_element_product_f32(K, d_euler, P_term);
    arm_vec3_sub_f32(d_euler, d_euler_prior, D_term);
    arm_vec3_scalar_mult_f32(D_term, 1 / dt, D_term);
    arm_vec3_element_product_f32(Kd, D_term, D_term);
    arm_vec3_scalar_mult_f32(I_sum, dt, I_term);
    arm_vec3_element_product_f32(Ki, I_term, I_term);
    arm_vec3_add_f32(P_term, I_term, PID_out);
    arm_vec3_add_f32(PID_out, D_term, PID_out);

    SERVO_TVC(PID_out[0], PID_out[1], PID_out[2]);

    TimeMeasureStop(); // Stop measuring time
    vTaskDelayUntil( &xLastWakeTime, xFrequency); // Delay for 1ms (1000Hz) Always at the end of the loop
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
    signalPlotter_sendData(15, x[3]);
    signalPlotter_sendData(16, x[4]);
    signalPlotter_sendData(17, x[5]);
    signalPlotter_sendData(19, 0);
    signalPlotter_sendData(20, 0);
    signalPlotter_sendData(21, 0);
    signalPlotter_sendData(22, (float)nrf_timeout);

    signalPlotter_executeTransmission(HAL_GetTick());

    // Kalman Filter correction step
    // Kalman Gain update
  /*
    arm_mat_trans_f32(&C, &Ctrans);               // 3x6 = 6x3
    arm_mat_mult_f32(&P, &Ctrans, &P_Ctrans);     // 6x6 * 6x3 = 6x3
    arm_mat_mult_f32(&C, &P_Ctrans, &Sinv);       // 3x6 * 6x3 = 3x3
    arm_mat_add_f32(&Sinv, &R, &S);               // 3x3 + 3x3 = 3x3
    arm_mat_inverse_f32(&S, &Sinv);               // 3x3 = 3x3
    arm_mat_mult_f32(&Ctrans, &Sinv, &P_Ctrans);  // 6x3 * 3x3 = 6x3
    arm_mat_mult_f32(&P, &P_Ctrans, &Kgain);      // 6x6 * 6x3 = 6x3

    // State Vector update
    arm_mat_vec_mult_f32(&C, x, dz);      // 3x6 * 6x1 = 3x1
    arm_vec3_sub_f32(z, dz, dz);          // 3x1 - 3x1 = 3x1
    arm_mat_vec_mult_f32(&Kgain, dz, dx); // 6x3 * 3x1 = 6x1
    arm_vecN_add_f32(6, x, dx, x);        // 6x1 + 6x1 = 6x1

    phi = x[0];
    theta = x[1];
    psi = x[2];

    // normalize update
    if(phi > PI) {
      phi -= 2*PI;
    }
    if(phi < PI) {
      phi += 2*PI;
    }

    if(theta > PI) {
      theta -= 2*PI;
    }
    if(theta < PI) {
      theta += 2*PI;
    }

    if(psi > PI) {
      psi -= 2*PI;
    }
    if(psi < PI) {
      psi += 2*PI;
    }

    x[0] = phi;
    x[1] = theta;
    x[2] = psi;

    // Covariance matrix update
    arm_mat_mult_f32(&C, &P, &C_P);         // 3x6 * 6x6 = 3x6
    arm_mat_mult_f32(&Kgain, &C_P, &Mtmp);  // 6x3 * 3x6 = 6x6
    arm_mat_sub_f32(&P, &Mtmp, &P);         // 6x6 - 6x6 = 6x6*/
  

    tx_data.float1 = phi;
    tx_data.float2 = theta;
    tx_data.float3 = psi;
    tx_data.float4 = phi_fix;
    tx_data.float5 = theta_fix;
    tx_data.float6 = psi_fix;

    // transmit data
    uint8_t tx_buf[NRF24L01P_TX_PAYLOAD_LENGTH] = {0};  
    memcpy(tx_buf, &tx_data, sizeof(tx_buf));
    nrf24l01p_sendOnce(tx_buf);
    HAL_Delay(2);
    nrf24l01p_startListening();

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
        
        if(nrf_mode) {
          nrf24l01p_tx_irq();
          Set_LED(0, 0, 0, 255);
          Set_Brightness(45);
          WS2812_Send();
        } else {
          uint8_t rx_buf[NRF24L01P_RX_PAYLOAD_LENGTH] = {0};
          nrf24l01p_rx_receive(rx_buf);
          if(rx_buf[4] != rx_buf[5]) { // discard trash data
            memcpy(&rx_data, rx_buf, sizeof(Data_Package_Receive));
          }        
          counter++;
          Set_LED(0, 255, 0, 255);
          Set_Brightness(45);
          WS2812_Send();
        }
      }
    }
    osDelay(5);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

uint8_t SelfTest(void) {
  if((SelfTest_Bitfield == 0b11111) && (SelfTest_Bitfield != 0b10011111)){
    Set_LED(0, 0, 255, 0);
    Set_Brightness(45);
    WS2812_Send();
    SelfTest_Bitfield |= (1<<7);  //All checks passed
  }
  else if(SelfTest_Bitfield != 0b10011111){
    SelfTest_Bitfield |= IMU1_SelfTest();
    SelfTest_Bitfield |= (IMU2_SelfTest()<<1);
    SelfTest_Bitfield |= (MAG_SelfTest()<<2);
    SelfTest_Bitfield |= (BMP_SelfTest()<<3);
    SelfTest_Bitfield |= (GPS_VER_CHECK()<<4); //Check if GPS is connected and working

    Set_LED(0, 255, 0, 0);
    Set_Brightness(45);
    WS2812_Send();
  }

  return SelfTest_Bitfield;
}

/* USER CODE END Application */

