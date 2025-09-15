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
#include "dts.h"
#include "cmsis_os.h"
#include "cli_app.h"
#include "stream_buffer.h"
#include "armMathAddon.h"
#include "semphr.h"
#include "queue.h"
#include <stdio.h>

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
#include "InterBoardCom.h"
#include "Packets.h"
#include "status.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"
#include "radio.h"
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
uint32_t pressure_raw;
uint32_t temperature_raw;
bmp390_handle_t bmp_handle;
float temperature, pressure;
static int32_t DTS_Temperature;

extern ADC_HandleTypeDef hadc3;
uint32_t ADC_Temperature, ADC_V_Ref;

uint32_t uid[3];

int8_t primary_status = 0;

double WGS84[3];
double WGS84_ref[3];

double ECEF[3];
double ECEF_ref[3];

double ENU[3];

uint8_t SelfTest_Bitfield = 0; //Bitfield for external Devices 0: IMU1, 1: IMU2, 2: MAG, 3: BARO, 4: GPS, 7:All checks passed

// euler angles
float phi, theta, psi;
float euler_deg[3];

// rotation matrix
float M_rot_data[9];
arm_matrix_instance_f32 M_rot = {3, 3, M_rot_data};
float M_rot_inv_data[9];
arm_matrix_instance_f32 M_rot_inv = {3, 3, M_rot_inv_data};

// euler angles from accelerations and magnetic field
float phi_fix, theta_fix, psi_fix;


float a_WorldFrame[3] = {0}; // Acceleration
float a_BodyFrame[3] = {0};
float a_abs;
float gravity_world_vec[3] = {0, 0, 9.8};
float gravity_body_vec[3];

float v_WorldFrame[3] = {0}; // Velocity
float v_BodyFrame[3] = {0};

uint8_t offset_phi = 127;
uint8_t offset_theta = 127;

uint8_t flight_status = 0; // 0 = awaiting gnss fix | 1 = align guidance | 2 = flight | 3 = abort

// time step
const float dt = 0.001;

// create new Kalman Filter instance for orientation
kalman_data_t EKF1;

// state and output vectors
float x1[x_size1] = {0};
float z1[z_size1] = {0};
float h1[z_size1] = {0};
float v1[z_size1] = {0};

float F1_data[x_size1*x_size1] = {0}; // 6x6
arm_matrix_instance_f32 F1 = {x_size1, x_size1, F1_data};
float B1_data[x_size1*u_size1] = {0}; // 6x3
arm_matrix_instance_f32 B1 = {x_size1, u_size1, B1_data};
float Q1_data[x_size1*x_size1] = {0}; // 6x6
arm_matrix_instance_f32 Q1 = {x_size1, x_size1, Q1_data};
float P1_data[x_size1*x_size1] = {0}; // 6x6
arm_matrix_instance_f32 P1 = {x_size1, x_size1, P1_data};
float H1_data[z_size1*x_size1] = {0}; // 3x6
arm_matrix_instance_f32 H1 = {z_size1, x_size1, H1_data};
float R1_data[z_size1*z_size1] = {0}; // 3x3
arm_matrix_instance_f32 R1 = {z_size1, z_size1, R1_data};
float S1_data[z_size1*z_size1] = {0}; // 3x3
arm_matrix_instance_f32 S1 = {z_size1, z_size1, S1_data};
float K1_data[x_size1*z_size1];       // 6x3
arm_matrix_instance_f32 K1 = {x_size1, z_size1, K1_data};

float Mdeuler_data[9] = {0};
arm_matrix_instance_f32 Mdeuler = {3, 3, Mdeuler_data};


// create new Kalman Filter instance for velocity and position
kalman_data_t EKF2;

// state and output vectors
float x2[x_size2] = {0};
float z2[z_size2] = {0};
float h2[z_size2] = {0};
float v2[z_size2] = {0}; // innovation

float F2_data[x_size2*x_size2] = {0}; // 6x6
arm_matrix_instance_f32 F2 = {x_size2, x_size2, F2_data};
float B2_data[x_size2*u_size2] = {0}; // 6x3
arm_matrix_instance_f32 B2 = {x_size2, u_size2, B2_data};
float Q2_data[x_size2*x_size2] = {0}; // 6x6
arm_matrix_instance_f32 Q2 = {x_size2, x_size2, Q2_data};
float P2_data[x_size2*x_size2] = {0}; // 6x6
arm_matrix_instance_f32 P2 = {x_size2, x_size2, P2_data};
float H2_data[z_size2*x_size2] = {0}; // 3x6
arm_matrix_instance_f32 H2 = {z_size2, x_size2, H2_data};
float R2_data[z_size2*z_size2] = {0}; // 3x3
arm_matrix_instance_f32 R2 = {z_size2, z_size2, R2_data};
float S2_data[z_size2*z_size2] = {0}; // 6x6
arm_matrix_instance_f32 S2 = {z_size2, z_size2, S2_data};
float K2_data[x_size2*z_size2];       // 6x3
arm_matrix_instance_f32 K2 = {x_size2, z_size2, K2_data};


// Quaternion EKF variables
kalman_data_t EKF3;

// state and output vectors
float x3[x_size3] = {0};
float z3[z_size3] = {0};
float h3[z_size3] = {0};
float v3[z_size3] = {0}; // innovation

float F3_data[x_size3*x_size3] = {0}; // 7x7
arm_matrix_instance_f32 F3 = {x_size3, x_size3, F3_data};
float Q3_data[x_size3*x_size3] = {0}; // 7x7
arm_matrix_instance_f32 Q3 = {x_size3, x_size3, Q3_data};
float P3_data[x_size3*x_size3] = {0}; // 7x7
arm_matrix_instance_f32 P3 = {x_size3, x_size3, P3_data};
float H3_data[z_size3*x_size3] = {0}; // 6x7
arm_matrix_instance_f32 H3 = {z_size3, x_size3, H3_data};
float R3_data[z_size3*z_size3] = {0}; // 6x6
arm_matrix_instance_f32 R3 = {z_size3, z_size3, R3_data};
float S3_data[z_size3*z_size3] = {0}; // 6x6
arm_matrix_instance_f32 S3 = {z_size3, z_size3, S3_data};
float K3_data[x_size3*z_size3];       // 7x6
arm_matrix_instance_f32 K3 = {x_size3, z_size3, K3_data};

float M_rot_q_data[9];
arm_matrix_instance_f32 M_rot_q = {3, 3, M_rot_q_data};

float euler_from_q[3] = {0};

// NRF24L01+ packages
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
void ReadInternalADC(uint32_t* temperature, uint32_t* v_ref);
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
  HAL_Delay(200); // Wait for USB and other Peripherals to initialize
  GPS_Init(); //Initialize the GPS module
  /* USER CODE BEGIN StartDefaultTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1; //1000 Hz

  uid[0] = HAL_GetUIDw0();
  uid[1] = HAL_GetUIDw1();
  uid[2] = HAL_GetUIDw2();

  // define Kalman Filter dimensions for orientation
  EKFInit(&EKF1, EKF1_type, x_size1, z_size1, u_size1, dt, &F1, &H1, &K1, &P1, &Q1, &R1, &S1, &B1, x1, z1, h1, imu1_data.gyro, v1);

  // define Kalman Filter dimensions for velocity and position
  EKFInit(&EKF2, EKF2_type, x_size2, z_size2, u_size2, dt, &F2, &H2, &K2, &P2, &Q2, &R2, &S2, &B2, x2, z2, h2, a_BodyFrame, v2);

  // define Kalman Filter dimensions for Quaternion EKF
  EKFInit(&EKF3, EKF3_type, x_size3, z_size3, u_size3, dt, &F3, &H3, &K3, &P3, &Q3, &R3, &S3, NULL, x3, z3, h3, imu1_data.gyro, v3);

  // define output signal names
  signalPlotter_init();

  // starting point for KF
  while(IMU1_VerifyDataReady() & 0x03 != 0x03); // wait for IMU1 data
  IMU1_ReadSensorData(&imu1_data);
  arm_vec3_sub_f32(imu1_data.accel, IMU1_offset, imu1_data.accel);
  arm_vec3_element_product_f32(imu1_data.accel, IMU1_scale, imu1_data.accel);

  while(!(MAG_VerifyDataReady() & 0b00000001)); // wait for MAG data
  MAG_ReadSensorData(&mag_data);
  arm_vec3_sub_f32(mag_data.field, MAG_offset, mag_data.field);
  arm_vec3_element_product_f32(mag_data.field, MAG_scale, mag_data.field);

  EulerOrientationFix(imu1_data.accel, mag_data.field, z1);

  x1[0] = z1[0];
  x1[1] = z1[1];
  x1[2] = z1[2];

  radioSet(NRF_24_ACTIVE);
  radioSetMode(RADIO_MODE_TRANSCEIVER);

  // Quaternion EKF initialization
  arm_vecN_concatenate_f32(3, imu1_data.accel, 3, mag_data.field, z3); // put measurements into z vector
  EKFStateVInit(&EKF3);

  /* Infinite loop */
  for(;;) {
    TimeMeasureStart(); // Start measuring time

    BMP_GetRawData(&pressure_raw, &temperature_raw);

    HAL_DTS_GetTemperature(&hdts, &DTS_Temperature);

    ReadInternalADC(&ADC_Temperature, &ADC_V_Ref);

    nrf_timeout++;

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

    // Kompensierte Temperatur berechnen (°C * 100)
    temperature = bmp390_compensate_temperature(temperature_raw, &bmp_handle);  // float, z.B. °C
    pressure = bmp390_compensate_pressure(pressure_raw, &bmp_handle);  // in Pa
    // KALMAN FILTER, ORIENTATION
    // initialize A and B matrix
    DeulerMatrixFromEuler(phi, theta, &Mdeuler);
    arm_mat_insert_mult_f32(&Mdeuler, &F1, 0, 3, -dt);
    arm_mat_insert_mult_f32(&Mdeuler, &B1, 0, 0, dt);

    // predicting state vector x
    EKFPredictStateV(&EKF1);

    // normalize phi, theta, psi to be within +-180 deg
    normalizeAngleVector(x1, 0, 2, PI, -PI, 2*PI);

    phi = x1[0];
    theta = x1[1];
    psi = x1[2];

    euler_deg[0] = phi * 180. / PI;
    euler_deg[1] = theta * 180. / PI;
    euler_deg[2] = psi * 180. / PI;

    // predicting state vector's covariance matrix P
    EKFPredictCovariance(&EKF1);

    // transform measured body acceleration to world-frame acceleration
    RotationMatrixFromEuler(phi, theta, psi, &M_rot);
    arm_mat_trans_f32(&M_rot, &M_rot_inv);
    Vec3_BodyToWorld(imu1_data.accel, &M_rot, a_WorldFrame);
    a_WorldFrame[2] -= 9.8;

    // calculate acceleration w/o gravity in body frame
    arm_mat_vec_mult_f32(&M_rot, gravity_world_vec, gravity_body_vec);
    arm_vec3_sub_f32(imu1_data.accel, gravity_body_vec, a_BodyFrame);
    a_abs = arm_vec3_length_f32(a_BodyFrame);

    // KALMAN FILTER, POSITION
    if(gps_data.gpsFix == 3) {
      // update A and B with rotation matrix to keep accelerometer bias aligned to body axis
      arm_mat_insert_mult_f32(&M_rot_inv, &F2, 0, 6, -dt);
      arm_mat_insert_mult_f32(&M_rot_inv, &F2, 3, 6, -0.5*dt*dt);
      arm_mat_insert_mult_f32(&M_rot_inv, &B2, 0, 0, dt);
      arm_mat_insert_mult_f32(&M_rot_inv, &B2, 3, 0, 0.5*dt*dt);

      EKFPredictStateV(&EKF2);
      EKFPredictCovariance(&EKF2);

      v_WorldFrame[0] = x2[0];
      v_WorldFrame[1] = x2[1];
      v_WorldFrame[2] = x2[2];

      arm_mat_vec_mult_f32(&M_rot, v_WorldFrame, v_BodyFrame);
    }

    if(flight_status == 0) { // AWAIT GNSS FIX
      if(gps_data.gpsFix == 3) {
        flight_status = 1;
      }
    } 
    else if(flight_status == 1) { // ALIGN GUIDANCE
      for(int i = 0; i <= 2; i++) {
        WGS84_ref[i] = WGS84[i];
      }
      WGS84toECEF(WGS84_ref, ECEF_ref);
    }

    // KALMAN FILTER, QUATERNION
    // predicting step
    EKFPredictStateV(&EKF3);
    EKFGetStateTransitionJacobian(&EKF3);
    EKFPredictCovariance(&EKF3);
    EKFPredictMeasurement(&EKF3);

    // Conversion to Euler
    RotationMatrixFromQuaternion(x3, &M_rot_q, DCM_bi_WorldToBody);
    EulerFromRotationMatrix(&M_rot_q, euler_from_q);

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
    #ifdef SIGNAL_PLOTTER_OUT_1 // signal plotter outputs position ekf testing
    signalPlotter_sendData(1, (float)nrf_timeout);
    signalPlotter_sendData(2, a_WorldFrame[0]);
    signalPlotter_sendData(3, a_WorldFrame[1]);
    signalPlotter_sendData(4, a_WorldFrame[2]);
    signalPlotter_sendData(5, x2[6]);
    signalPlotter_sendData(6, x2[7]);
    signalPlotter_sendData(7, x2[8]);
    signalPlotter_sendData(8, (float)gps_data.sAcc/1000.f);
    signalPlotter_sendData(9, (float)gps_data.hAcc/1000.f);
    signalPlotter_sendData(10, (float)gps_data.vAcc/1000.f);
    signalPlotter_sendData(11, phi);
    signalPlotter_sendData(12, theta);
    signalPlotter_sendData(13, psi);
    signalPlotter_sendData(14, phi_fix);
    signalPlotter_sendData(15, theta_fix);
    signalPlotter_sendData(16, psi_fix);
    signalPlotter_sendData(17, x2[0]);
    signalPlotter_sendData(18, x2[1]);
    signalPlotter_sendData(19, x2[2]);
    signalPlotter_sendData(20, x2[3]);
    signalPlotter_sendData(21, x2[4]);
    signalPlotter_sendData(22, x2[5]);
    signalPlotter_sendData(23, z2[0]);
    signalPlotter_sendData(24, z2[1]);
    signalPlotter_sendData(25, z2[2]);
    signalPlotter_sendData(26, z2[3]);
    signalPlotter_sendData(27, z2[4]);
    signalPlotter_sendData(28, z2[5]);
    #endif

    #ifdef SIGNAL_PLOTTER_OUT_2 // signal plotter outputs raw sensor data
    signalPlotter_sendData(1, (float)nrf_timeout);
    signalPlotter_sendData(2, mag_data.field[0]);
    signalPlotter_sendData(3, mag_data.field[1]);
    signalPlotter_sendData(4, mag_data.field[2]);
    signalPlotter_sendData(5, imu1_data.accel[0]);
    signalPlotter_sendData(6, imu1_data.accel[1]);
    signalPlotter_sendData(7, imu1_data.accel[2]);
    signalPlotter_sendData(8, imu1_data.gyro[0]);
    signalPlotter_sendData(9, imu1_data.gyro[1]);
    signalPlotter_sendData(10, imu1_data.gyro[2]);
    signalPlotter_sendData(11, pressure);
    signalPlotter_sendData(12, temperature);
    signalPlotter_sendData(13, (float)gps_data.gpsFix);
    signalPlotter_sendData(14, (float)gps_data.numSV);
    signalPlotter_sendData(15, (float)gps_data.hAcc/1000.f);
    signalPlotter_sendData(16, (float)gps_data.vAcc/1000.f);
    signalPlotter_sendData(17, (float)gps_data.sAcc/1000.f);
    signalPlotter_sendData(18, (float)gps_data.lat*1e-7);
    signalPlotter_sendData(19, (float)gps_data.lon*1e-7);
    signalPlotter_sendData(20, (float)gps_data.height/1000.f);
    signalPlotter_sendData(21, (float)gps_data.velN/1000.f);
    signalPlotter_sendData(22, (float)gps_data.velE/1000.f);
    signalPlotter_sendData(23, (float)gps_data.velD/1000.f);
    #endif

    #ifdef SIGNAL_PLOTTER_OUT_3 // signal plotter outputs quaternion ekf testing
    signalPlotter_sendData(1, (float)nrf_timeout);
    signalPlotter_sendData(2, euler_from_q[0]);
    signalPlotter_sendData(3, euler_from_q[1]);
    signalPlotter_sendData(4, euler_from_q[2]);
    signalPlotter_sendData(5, h3[0]);
    signalPlotter_sendData(6, h3[1]);
    signalPlotter_sendData(7, h3[2]);
    signalPlotter_sendData(8, h3[3]);
    signalPlotter_sendData(9, h3[4]);
    signalPlotter_sendData(10, h3[5]);
    signalPlotter_sendData(11, imu1_data.accel[0]);
    signalPlotter_sendData(12, imu1_data.accel[1]);
    signalPlotter_sendData(13, imu1_data.accel[2]);
    signalPlotter_sendData(14, mag_data.field[0]);
    signalPlotter_sendData(15, mag_data.field[1]);
    signalPlotter_sendData(16, mag_data.field[2]);
    signalPlotter_sendData(17, phi);
    signalPlotter_sendData(18, theta);
    signalPlotter_sendData(19, psi);
    signalPlotter_sendData(20, imu1_data.gyro[0]);
    signalPlotter_sendData(21, imu1_data.gyro[1]);
    signalPlotter_sendData(22, imu1_data.gyro[2]);
    signalPlotter_sendData(23, x3[4]);
    signalPlotter_sendData(24, x3[5]);
    signalPlotter_sendData(25, x3[6]);
    signalPlotter_sendData(26, x1[3]);
    signalPlotter_sendData(27, x1[4]);
    signalPlotter_sendData(28, x1[5]);
    #endif

    signalPlotter_executeTransmission(HAL_GetTick());

    // attitude estimation using magnetometer and accelerometer
    EulerOrientationFix(imu1_data.accel, mag_data.field, z1);

    normalizeAnglePairVector(x1, z1, 0, 2, PI, -PI, 2*PI);

    phi_fix = z1[0];
    theta_fix = z1[1];
    psi_fix = z1[2];

    // Kalman Filter eulercorrection step
    EKFUpdateKalmanGain(&EKF1);
    EKFPredictMeasurement(&EKF1);
    EKFGetInnovation(&EKF1);
    EKFCorrectStateV(&EKF1);
    EKFCorrectCovariance(&EKF1);

    // normalize phi, theta, psi to be within +-180 deg
    normalizeAngleVector(x1, 0, 2, PI, -PI, 2*PI);

    phi = x1[0];
    theta = x1[1];
    psi = x1[2];

    // KALMAN FILTER, QUATERNION
    // correction step
    arm_vecN_concatenate_f32(3, imu1_data.accel, 3, mag_data.field, z3); // put measurements into z vector
    EKFGetInnovation(&EKF3);
    EKFGetObservationJacobian(&EKF3);
    EKFUpdateKalmanGain(&EKF3);
    EKFCorrectStateV(&EKF3);
    EKFCorrectCovariance(&EKF3);
    
    ShowStatus(RGB_PRIMARY, primary_status, 1, 100);

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
    SelfTest();         // Run self-test on startup

    GPS_ReadSensorData(&gps_data);
    if(primary_status > 0) {
      switch(gps_data.gpsFix) {
        case 0: primary_status = STATUS_GNSS_ALIGN; break;
        case 2: primary_status = STATUS_GNSS_2D; break;
        case 3: primary_status = STATUS_STANDBY; break;
      }
    }
    
    IMUPacket_t imu_packet = {
      .timestamp = HAL_GetTick(),
      .gyroX = imu1_data.gyro[0],
      .gyroY = imu1_data.gyro[1],
      .gyroZ = imu1_data.gyro[2],
      .accelX = imu1_data.accel[0],
      .accelY = imu1_data.accel[1],
      .accelZ = imu1_data.accel[2],
      .magX = mag_data.field[0],
      .magY = mag_data.field[1],
      .magZ = mag_data.field[2],
      .unused1 = 0,
      .unused2 = 0,
      .unused3 = 0
    };
    
    if(gps_data.gpsFix == 3) {
      UBLOXtoWGS84(gps_data.lat, gps_data.lon, gps_data.height, WGS84);
      WGS84toECEF(WGS84, ECEF);
      if(flight_status == 0) {
        for(int i = 0; i <= 2; i++) {
          WGS84_ref[i] = WGS84[i];
        }
        WGS84toECEF(WGS84_ref, ECEF_ref);
      }
      ECEFtoENU(WGS84_ref, ECEF_ref, ECEF, ENU);

      z2[0] = (float)gps_data.velE / 1000.f; // m/s
      z2[1] = (float)gps_data.velN / 1000.f;
      z2[2] = (float)gps_data.velD / (-1000.f);
      z2[3] = (float)ENU[0]; // m
      z2[4] = (float)ENU[1];
      z2[5] = (float)ENU[2];

      arm_mat_set_diag_f32(&R2, 0, 0, 3, (float)gps_data.sAcc * gps_data.sAcc / 1e6f * 0.001f * (1 + a_abs));
      arm_mat_set_diag_f32(&R2, 3, 3, 2, (float)gps_data.hAcc * gps_data.hAcc / 1e6f * 100.f);
      arm_mat_set_entry_f32(&R2, 5, 5, (float)gps_data.vAcc * gps_data.vAcc / 1e6f * 100.f);

      // Kalman Filter correction step
      EKFUpdateKalmanGain(&EKF2);
      EKFPredictMeasurement(&EKF2);
      EKFGetInnovation(&EKF2);
      EKFCorrectStateV(&EKF2);
      EKFCorrectCovariance(&EKF2);

      SERVO_MoveToAngle(1, 0);
      SERVO_MoveToAngle(2, 0);
    }

    // transmit data
    uint8_t tx_buf[NRF24L01P_PAYLOAD_LENGTH];
    tx_buf[1] = gps_data.sec; // Packet type
    radioSend(tx_buf);

    // RECOVERY TEST
    /*if(uwTick > 5500 && primary_status > 0 && primary_status != 5 && primary_status == 4) {
      primary_status = 5;
      SERVO_MoveToAngle(1, 0);
      SERVO_MoveToAngle(2, 0);
    }
    if(uwTick > 5000 && primary_status > 0 && primary_status != 5 && primary_status != 4) {
      primary_status = 4;
      SERVO_MoveToAngle(1, 90);
      SERVO_MoveToAngle(2, 90);
    }*/

    // KF2 correction steps
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
uint8_t rx_recieve_buf[NRF24L01P_PAYLOAD_LENGTH] = {0};
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
        
        if(nrf_mode) {
          nrf24l01p_tx_irq();
        } else {
          HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
          memset(rx_recieve_buf, 0, NRF24L01P_PAYLOAD_LENGTH);
          nrf24l01p_rx_receive(rx_recieve_buf);
        }
      }
    }
    osDelay(5);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
uint8_t selftest_tries = 0;
uint8_t SelfTest(void) {
  if((SelfTest_Bitfield == 0b11111) && (SelfTest_Bitfield != 0b10011111)){
    if(primary_status >= 0) primary_status = STATUS_GNSS_ALIGN;

    SelfTest_Bitfield |= (1<<7);  //All checks passed
  }
  else if (selftest_tries > 100){
    primary_status = STATUS_ERROR_STARTUP;
    return SelfTest_Bitfield;
  }
  else if(SelfTest_Bitfield != 0b10011111){
    selftest_tries += 1;
    SelfTest_Bitfield |= IMU1_SelfTest();
    SelfTest_Bitfield |= (IMU2_SelfTest()<<1);
    SelfTest_Bitfield |= (MAG_SelfTest()<<2);
    SelfTest_Bitfield |= (BMP_SelfTest()<<3);
    SelfTest_Bitfield |= (GPS_VER_CHECK()<<4); //Check if GPS is connected and working

    if(primary_status > 0) primary_status = STATUS_STARTUP;
  }

  return SelfTest_Bitfield;
}

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

/* USER CODE END Application */

