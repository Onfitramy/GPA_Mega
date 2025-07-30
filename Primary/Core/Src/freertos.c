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
#include "InterBoardCom.h"
#include "Packets.h"
#include "status.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"
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
float dt = 0.001;

// create new Kalman Filter instance for orientation
Kalman_Instance Kalman1;

// state and output vectors
float x[x_size1] = {0};
float z[z_size1] = {0};

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
  KalmanFilterInit(&Kalman1, x_size1, z_size1, u_size1);

  // define Kalman Filter dimensions for velocity and position
  KalmanFilterInit(&Kalman2, x_size2, z_size2, u_size2);

  signalPlotter_setSignalName(0, "delta Time");
  signalPlotter_setSignalName(1, "NRF timeout");
  signalPlotter_setSignalName(2, "a_x world");
  signalPlotter_setSignalName(3, "a_y world");
  signalPlotter_setSignalName(4, "a_z world");
  signalPlotter_setSignalName(5, "a_x offset");
  signalPlotter_setSignalName(6, "a_y offset");
  signalPlotter_setSignalName(7, "a_z offset");
  signalPlotter_setSignalName(8, "velocity var");
  signalPlotter_setSignalName(9, "horizontal var");
  signalPlotter_setSignalName(10, "vertical var");
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
  signalPlotter_setSignalName(29, "pressure");
  signalPlotter_setSignalName(30, "temperature");

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
  arm_mat_fill_diag_f32(&C2, 0, 0, 1.);

  arm_mat_set_diag_f32(&P2, 0, 0, 3, 1.);  // initial guess for velocity variance
  arm_mat_set_diag_f32(&P2, 3, 3, 3, 10.);
  arm_mat_set_diag_f32(&P2, 6, 6, 3, 0.5); // initial guess for accelerometer offset variance

  arm_mat_set_diag_f32(&Q2, 0, 0, 3, 1e-3f*dt*dt); // variance of accelerometer data   (noise)
  arm_mat_set_diag_f32(&Q2, 3, 3, 3, 1e-3f*0.25*dt*dt*dt*dt);
  arm_mat_set_diag_f32(&Q2, 6, 6, 3, 1e-12f); // variance of accelerometer offset (drift)

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

    BMP_GetRawData(&pressure_raw, &temperature_raw);

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
      arm_mat_insert_mult_32(&M_rot_inv, &A2, 0, 6, -dt);
      arm_mat_insert_mult_32(&M_rot_inv, &A2, 3, 6, -0.5*dt*dt);
      arm_mat_insert_mult_32(&M_rot_inv, &B2, 0, 0, dt);
      arm_mat_insert_mult_32(&M_rot_inv, &B2, 3, 0, 0.5*dt*dt);

      KalmanFilterPredictSV(&Kalman2, &A2, x2, &B2, a_BodyFrame);
      KalmanFilterPredictCM(&Kalman2, &A2, &P2, &Q2);

      v_WorldFrame[0] = x2[0];
      v_WorldFrame[1] = x2[1];
      v_WorldFrame[2] = x2[2];

      arm_mat_vec_mult_f32(&M_rot, v_WorldFrame, v_BodyFrame);
    }

    if(flight_status == 0) { // AWAIT GNSS FIX
      if(gps_data.gpsFix == 3) {
        flight_status = 1;
      }

      Set_LED(0, 255, 255, 0);
      Set_Brightness(45);
      WS2812_Send();
    } 
    else if(flight_status == 1) { // ALIGN GUIDANCE
      for(int i = 0; i <= 2; i++) {
        WGS84_ref[i] = WGS84[i];
      }
      WGS84toECEF(WGS84_ref, ECEF_ref);

      Set_LED(0, 0, 255, 0);
      Set_Brightness(45);
      WS2812_Send();
    }

    // send TVC command with deflection physically limited to 60° and twist limited to 30° to ensure xz stability

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
    signalPlotter_sendData(1, mag_data.field[0]);
    signalPlotter_sendData(2, mag_data.field[1]);
    signalPlotter_sendData(3, mag_data.field[2]);
    signalPlotter_sendData(4, imu1_data.accel[0]);
    signalPlotter_sendData(5, imu1_data.accel[1]);
    signalPlotter_sendData(6, imu1_data.accel[2]);
    signalPlotter_sendData(7, imu1_data.gyro[0]);
    signalPlotter_sendData(8, imu1_data.gyro[1]);
    signalPlotter_sendData(9, imu1_data.gyro[2]);
    signalPlotter_sendData(10, pressure);
    signalPlotter_sendData(11, temperature);
    signalPlotter_sendData(12, (float)gps_data.numSV);
    signalPlotter_sendData(13, (float)gps_data.hAcc/1000.f);
    signalPlotter_sendData(14, (float)gps_data.vAcc/1000.f);
    signalPlotter_sendData(15, (float)gps_data.sAcc/1000.f);
    signalPlotter_sendData(16, (float)gps_data.lat*1e-7);
    signalPlotter_sendData(17, (float)gps_data.lon*1e-7);
    signalPlotter_sendData(18, (float)gps_data.height/1000.f);
    signalPlotter_sendData(19, (float)gps_data.velN/1000.f);
    signalPlotter_sendData(20, (float)gps_data.velE/1000.f);
    signalPlotter_sendData(21, (float)gps_data.velD/1000.f);

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

    // transmit data
    uint8_t tx_buf[NRF24L01P_TX_PAYLOAD_LENGTH] = {0};  
    memcpy(tx_buf, &tx_data, sizeof(tx_buf));
    nrf24l01p_sendOnce(tx_buf);
    HAL_Delay(2);
    nrf24l01p_startListening();
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

    PacketData_u packet_data;
    packet_data.imu = imu_packet;
    InterBoardCom_SendDataPacket(InterBoardPACKET_ID_DataSaveFLASH ,PACKET_ID_IMU, &packet_data);

      arm_mat_set_diag_f32(&R2, 0, 0, 3, (float)gps_data.sAcc * gps_data.sAcc / 1e6f * 0.001f * (1 + a_abs));
      arm_mat_set_diag_f32(&R2, 3, 3, 2, (float)gps_data.hAcc * gps_data.hAcc / 1e6f * 100.f);
      arm_mat_set_entry_f32(&R2, 5, 5, (float)gps_data.vAcc * gps_data.vAcc / 1e6f * 100.f);

      KalmanFilterUpdateGain(&Kalman2, &P2, &C2, &R2, &K2);
      KalmanFilterCorrectSV(&Kalman2, &K2, z2, &C2, x2);
      KalmanFilterCorrectCM(&Kalman2, &K2, &C2, &P2);
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
        
        if(nrf_mode) {
          nrf24l01p_tx_irq();
        } else {
          HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
          uint8_t rx_buf[NRF24L01P_RX_PAYLOAD_LENGTH] = {0};
          nrf24l01p_rx_receive(rx_buf);
          if(rx_buf[4] != rx_buf[5]) { // discard trash data
            memcpy(&rx_data, rx_buf, sizeof(Data_Package_Receive));
          }        
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
    if(primary_status >= 0) primary_status = STATUS_GNSS_ALIGN;

    SelfTest_Bitfield |= (1<<7);  //All checks passed
  }
  else if(SelfTest_Bitfield != 0b10011111){
    SelfTest_Bitfield |= IMU1_SelfTest();
    SelfTest_Bitfield |= (IMU2_SelfTest()<<1);
    SelfTest_Bitfield |= (MAG_SelfTest()<<2);
    SelfTest_Bitfield |= (BMP_SelfTest()<<3);
    SelfTest_Bitfield |= (GPS_VER_CHECK()<<4); //Check if GPS is connected and working

    if(primary_status > 0) primary_status = STATUS_STARTUP;
  }

  return SelfTest_Bitfield;
}

/* USER CODE END Application */

