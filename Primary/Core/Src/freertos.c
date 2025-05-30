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

double WGS84[3];
double WGS84_ref[3] = {50.768692, 6.087405, 180.};

double ECEF[3];
double ECEF_ref[3];

double ENU[3];

int counter = 0;

uint8_t SelfTest_Bitfield = 0; //Bitfield for external Devices 0: IMU1, 1: IMU2, 2: MAG, 3: BARO, 4: GPS, 7:All checks passed

// euler angles
float phi, theta, psi;
float euler_deg[3];

// rotation matrix
float M_rot_data[9];
arm_matrix_instance_f32 M_rot = {3, 3, M_rot_data};

// euler angles from accelerations and magnetic field
float phi_fix, theta_fix, psi_fix;

// Variables for calibration
float gyr_sumup[3] = {0, 0, 0};
float gyr_offset[3] = {0, 0, 0};
float acc_sumup[3] = {0, 0, 0};
float mag_sumup[3] = {0, 0, 0};

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

float a_WorldFrame[3] = {0}; // Acceleration

float throttle_cmd = 0;

uint8_t offset_phi = 127;
uint8_t offset_theta = 127;

// time step
float dt = 0.001;


// create new Kalman Filter instance for orientation
Kalman_Instance Kalman1;

// state and output vectors
float x[x_size1] = {0};
float z[z_size1] = {0};

x6z3u3KalmanData KalmanData1;

float A1_data[x_size1*x_size1] = {0}; // 6x6
arm_matrix_instance_f32 A1 = {x_size1, x_size1, A1_data};
float B1_data[x_size1*u_size1] = {0}; // 6x3
arm_matrix_instance_f32 B1 = {x_size1, u_size1, B1_data};
float Q1_data[x_size1*x_size1] = {0}; // 6x6
arm_matrix_instance_f32 Q1 = {x_size1, x_size1, Q1_data};
float P1_data[x_size1*x_size1] = {0}; // 6x6
arm_matrix_instance_f32 P1 = {x_size1, x_size1, P1_data};
float C1_data[z_size1*x_size1] = {0}; // 3x6
arm_matrix_instance_f32 C1 = {z_size1, x_size1, C1_data};
float R1_data[z_size1*z_size1] = {0}; // 3x3
arm_matrix_instance_f32 R1 = {z_size1, z_size1, R1_data};
float K1_data[x_size1*z_size1];       // 6x3
arm_matrix_instance_f32 K1 = {x_size1, z_size1, K1_data};

float Mdeuler_data[9] = {0};
arm_matrix_instance_f32 Mdeuler = {3, 3, Mdeuler_data};


// create new Kalman Filter instance for velocity and position
Kalman_Instance Kalman2;

// state and output vectors
float x2[x_size2] = {0};
float z2[z_size2] = {0};

x9z6u3KalmanData KalmanData2;

float A2_data[x_size2*x_size2] = {0}; // 6x6
arm_matrix_instance_f32 A2 = {x_size2, x_size2, A2_data};
float B2_data[x_size2*u_size2] = {0}; // 6x3
arm_matrix_instance_f32 B2 = {x_size2, u_size2, B2_data};
float Q2_data[x_size2*x_size2] = {0}; // 6x6
arm_matrix_instance_f32 Q2 = {x_size2, x_size2, Q2_data};
float P2_data[x_size2*x_size2] = {0}; // 6x6
arm_matrix_instance_f32 P2 = {x_size2, x_size2, P2_data};
float C2_data[z_size2*x_size2] = {0}; // 3x6
arm_matrix_instance_f32 C2 = {z_size2, x_size2, C2_data};
float R2_data[z_size2*z_size2] = {0}; // 3x3
arm_matrix_instance_f32 R2 = {z_size2, z_size2, R2_data};
float K2_data[x_size2*z_size2];       // 6x3
arm_matrix_instance_f32 K2 = {x_size2, z_size2, K2_data};


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
  HAL_Delay(200); // Wait for USB and other Peripherals to initialize
  GPS_Init(); //Initialize the GPS module
  /* USER CODE BEGIN StartDefaultTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1; //1000 Hz

  // define Kalman Filter dimensions for orientation
  KalmanFilterInit(&Kalman1, &KalmanData1, x_size1, z_size1, u_size1);

  // define Kalman Filter dimensions for velocity and position
  KalmanFilterInit(&Kalman2, &KalmanData2, x_size2, z_size2, u_size2);

  signalPlotter_setSignalName(0, "delta Time");
  signalPlotter_setSignalName(1, "NRF timeout");
  signalPlotter_setSignalName(2, "MAG_X");
  signalPlotter_setSignalName(3, "MAG_Y");
  signalPlotter_setSignalName(4, "MAG_Z");
  signalPlotter_setSignalName(5, "ACC_X");
  signalPlotter_setSignalName(6, "ACC_Y");
  signalPlotter_setSignalName(7, "ACC_Z");
  signalPlotter_setSignalName(8, "GYR_X");
  signalPlotter_setSignalName(9, "GYR_Y");
  signalPlotter_setSignalName(10, "GYR_Z");
  signalPlotter_setSignalName(11, "phi");
  signalPlotter_setSignalName(12, "theta");
  signalPlotter_setSignalName(13, "psi");
  signalPlotter_setSignalName(14, "phi fix");
  signalPlotter_setSignalName(15, "theta fix");
  signalPlotter_setSignalName(16, "psi fix");
  signalPlotter_setSignalName(17, "vel x");
  signalPlotter_setSignalName(18, "vel y");
  signalPlotter_setSignalName(19, "vel z");
  signalPlotter_setSignalName(20, "pos x");
  signalPlotter_setSignalName(21, "pos y");
  signalPlotter_setSignalName(22, "pos z");
  signalPlotter_setSignalName(23, "vel x fix");
  signalPlotter_setSignalName(24, "vel y fix");
  signalPlotter_setSignalName(25, "vel z fix");
  signalPlotter_setSignalName(26, "pos x fix");
  signalPlotter_setSignalName(27, "pos y fix");
  signalPlotter_setSignalName(28, "pos z fix");
  
  arm_vec3_element_product_f32(K, Tv, Kd);
  Tn[0] = 1 / Tn[0];
  Tn[1] = 1 / Tn[1];
  Tn[2] = 1 / Tn[2];
  arm_vec3_element_product_f32(K, Tn, Ki);

  // configure orientation KF matrices
  arm_mat_fill_diag_f32(&A1, 0, 0, 1.);
  arm_mat_fill_diag_f32(&C1, 0, 0, 1.);
  arm_mat_fill_diag_f32(&R1, 0, 0, 1.);    // angle fixing method variance (noise)

  arm_mat_set_diag_f32(&P1, 0, 0, 3, 0.1);   // initial guess for angle variance
  arm_mat_set_diag_f32(&P1, 3, 3, 3, 0.001); // initial guess for offset variance

  arm_mat_set_diag_f32(&Q1, 0, 0, 3, 1e-8);  // variance of gyroscope data   (noise)
  arm_mat_set_diag_f32(&Q1, 3, 3, 3, 1e-12); // variance of gyroscope offset (drift)

  // configure position KF matrices
  arm_mat_fill_diag_f32(&A2, 0, 0, 1.);
  arm_mat_set_diag_f32(&A2, 3, 0, 3, dt);
  arm_mat_set_diag_f32(&A2, 0, 6, 3, -dt);
  arm_mat_set_diag_f32(&A2, 3, 6, 3, -0.5*dt*dt);
  arm_mat_fill_diag_f32(&B2, 0, 0, dt);
  arm_mat_fill_diag_f32(&B2, 3, 0, 0.5*dt*dt);
  arm_mat_fill_diag_f32(&C2, 0, 0, 1.);

  arm_mat_set_diag_f32(&P2, 0, 0, 3, 1.);  // initial guess for velocity variance
  arm_mat_set_diag_f32(&P2, 3, 3, 3, 0.);
  arm_mat_set_diag_f32(&P2, 6, 6, 3, 0.5); // initial guess for accelerometer offset variance

  arm_mat_set_diag_f32(&Q2, 0, 0, 3, 1e-3f*dt*dt); // variance of accelerometer data   (noise)
  arm_mat_set_diag_f32(&Q2, 3, 3, 3, 1e-3f*0.25*dt*dt*dt*dt);
  arm_mat_set_diag_f32(&Q2, 6, 6, 3, 1e-12f); // variance of accelerometer offset (drift)

  // covert reference point
  WGS84toECEF(WGS84_ref, ECEF_ref);

  // starting point for KF
  while(IMU1_VerifyDataReady() & 0x03 != 0x03); // wait for IMU1 data
  IMU1_ReadSensorData(&imu1_data);
  arm_vec3_sub_f32(imu1_data.accel, IMU1_offset, imu1_data.accel);
  arm_vec3_element_product_f32(imu1_data.accel, IMU1_scale, imu1_data.accel);

  while(!(MAG_VerifyDataReady() & 0b00000001)); // wait for MAG data
  MAG_ReadSensorData(&mag_data);
  arm_vec3_sub_f32(mag_data.field, MAG_offset, mag_data.field);
  arm_vec3_element_product_f32(mag_data.field, MAG_scale, mag_data.field);

  OrientationFix(imu1_data.accel, mag_data.field, z);

  x[0] = z[0];
  x[1] = z[1];
  x[2] = z[2];

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

    // KALMAN FILTER, ORIENTATION
    // initialize A and B matrix
    DeulerMatrixFromEuler(phi, theta, &Mdeuler);
    arm_mat_insert_mult_32(&Mdeuler, &A1, 0, 3, -dt);
    arm_mat_insert_mult_32(&Mdeuler, &B1, 0, 0, dt);

    // predicting state vector x
    KalmanFilterPredictSV(&Kalman1, &A1, x, &B1, imu1_data.gyro);

    // normalize phi, theta, psi to be within +-180 deg
    normalizeAngleVector(x, 0, 2, PI, -PI, 2*PI);

    phi = x[0];
    theta = x[1];
    psi = x[2];

    euler_deg[0] = phi * 180. / PI;
    euler_deg[1] = theta * 180. / PI;
    euler_deg[2] = psi * 180. / PI;

    // predicting state vector's covariance matrix P
    KalmanFilterPredictCM(&Kalman1, &A1, &P1, &Q1);

    // transform measured body acceleration to world-frame acceleration
    RotationMatrixFromEuler(phi, theta, psi, &M_rot);
    //Vec3_BodyToWorld(imu1_data.accel, &M_rot, a_WorldFrame);
    //a_WorldFrame[2] -= 9.8;

    // KALMAN FILTER, POSITION
    if(gps_data.gpsFix != 3) {
      //KalmanFilterPredictSV(&Kalman2, &A2, x2, &B2, a_WorldFrame);
      KalmanFilterPredictCM(&Kalman2, &A2, &P2, &Q2);
    }

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
        //normalizeAngle(&euler_set[2], 180, -180, 360);
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
    normalizeAnglePairVector(euler_deg, euler_set, 0, 2, 180, -180, 360);
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

    signalPlotter_executeTransmission(HAL_GetTick());

    // attitude estimation using magnetometer and accelerometer
    OrientationFix(imu1_data.accel, mag_data.field, z);

    normalizeAnglePairVector(x, z, 0, 2, PI, -PI, 2*PI);

    phi_fix = z[0];
    theta_fix = z[1];
    psi_fix = z[2];

    // Kalman Filter correction step
    KalmanFilterUpdateGain(&Kalman1, &P1, &C1, &R1, &K1);
    KalmanFilterCorrectSV(&Kalman1, &K1, z, &C1, x);
    KalmanFilterCorrectCM(&Kalman1, &K1, &C1, &P1);

    // normalize phi, theta, psi to be within +-180 deg
    normalizeAngleVector(x, 0, 2, PI, -PI, 2*PI);

    phi = x[0];
    theta = x[1];
    psi = x[2];

    tx_data.float1 = (float)x2[0];
    tx_data.float2 = (float)x2[1];
    tx_data.float3 = (float)x2[2];
    tx_data.float4 = (float)x2[3];
    tx_data.float5 = (float)x2[4];
    tx_data.float6 = (float)x2[5];
    tx_data.float7 = (float)ENU[0];
    tx_data.float8 = (float)ENU[1];

    /*tx_data.float1 = (float)gps_data.gpsFix;
    tx_data.float2 = (float)gps_data.numSV;
    tx_data.float3 = (float)gps_data.hAcc / 1000.f;
    tx_data.float4 = (float)gps_data.vAcc / 1000.f;
    tx_data.float5 = (float)gps_data.headAcc * 1e-5f;
    tx_data.float6 = (float)gps_data.headVeh * 1e-5f;
    tx_data.float7 = (float)ENU[0];
    tx_data.float8 = (float)ENU[1];*/

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

    if(gps_data.gpsFix == 3) {
      UBLOXtoWGS84(gps_data.lat, gps_data.lon, gps_data.height, WGS84);
      WGS84toECEF(WGS84, ECEF);
      ECEFtoENU(WGS84_ref, ECEF_ref, ECEF, ENU);

      z2[0] = (float)gps_data.velE / 1000.f;
      z2[1] = (float)gps_data.velN / 1000.f;
      z2[2] = (float)gps_data.velD / (-1000.f);
      z2[3] = (float)ENU[0];
      z2[4] = (float)ENU[1];
      z2[5] = (float)ENU[2];

      arm_mat_set_diag_f32(&R2, 0, 0, 3, (float)gps_data.sAcc * gps_data.sAcc / 1e6f);
      arm_mat_set_diag_f32(&R2, 3, 3, 2, (float)gps_data.hAcc * gps_data.hAcc / 1e6f);
      arm_mat_set_entry_f32(&R2, 5, 5, (float)gps_data.vAcc * gps_data.vAcc / 1e6f);

      KalmanFilterUpdateGain(&Kalman2, &P2, &C2, &R2, &K2);
      KalmanFilterCorrectSV(&Kalman2, &K2, z2, &C2, x2);
      KalmanFilterCorrectCM(&Kalman2, &K2, &C2, &P2);

      Set_LED(0, 0, 0, 255);
      Set_Brightness(45);
      WS2812_Send();
    } else {
      Set_LED(0, 255, 255, 0);
      Set_Brightness(45);
      WS2812_Send();
    }

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

