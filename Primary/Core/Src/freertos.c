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
#include "IMUS.h"
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
IMU_Data_t imu1_data;
IMU_Data_t imu2_data;
IMU_AverageData_t average_imu_data;
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
GPA_Mega gpa_mega;

int8_t primary_status = 0;
int8_t servo_status = 1;

uint32_t dt_1000Hz;

double WGS84[3];
double WGS84_ref[3];

double ECEF[3];
double ECEF_ref[3];

double ENU[3];

//uint8_t SelfTest_Bitfield = 0; //Bitfield for external Devices 0: IMU1, 1: IMU2, 2: MAG, 3: BARO, 4: GPS, 7:All checks passed
StatusPayload_t status_data = {0};
float F4_data_float;

// euler angles
float phi, theta, psi;
float euler_deg[3];

float Mdeuler_data[9] = {0};
arm_matrix_instance_f32 Mdeuler = {3, 3, Mdeuler_data};

// rotation matrix
float M_rot_data[9];
arm_matrix_instance_f32 M_rot_bi = {3, 3, M_rot_data};
float M_rot_inv_data[9];
arm_matrix_instance_f32 M_rot_ib = {3, 3, M_rot_inv_data};

// euler angles from accelerations and magnetic field
float phi_fix, theta_fix, psi_fix;


float a_WorldFrame[3] = {0}; // Acceleration
float a_BodyFrame[3] = {0};
float a_abs;
float gravity_world_vec[3] = {0, 0, 9.8};
float gravity_body_vec[3];

float height_baro;

uint8_t flight_status = 0; // 0 = awaiting gnss fix | 1 = align guidance | 2 = flight | 3 = abort

// time step
const float dt = 0.001;

float M_rot_q_data[9];
arm_matrix_instance_f32 M_rot_q = {3, 3, M_rot_q_data};

float euler_from_q[3] = {0};

// GNSS delay compensation
float corr_acc_buf[GNSS_VELOCITY_DELAY] = {0};
float corr_acc_sum = 0;
float corr_delta_v = 0;

float corr_vel_buf[GNSS_POSITION_DELAY] = {0};
float corr_vel_sum = 0;
float corr_delta_h = 0;

float gnss_height_corr;
float gnss_velZ_corr;


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
QueueHandle_t InterBoardCom_Queue;
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
  InterBoardCom_Queue = xQueueCreate(10, sizeof(InterBoardPacket_t));
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
  gpa_mega = GPA_MegaFromUID(uid);

  IMU_InitImu(&imu1_data, IMU1, gpa_mega);
  IMU_InitImu(&imu2_data, IMU2, gpa_mega);


  mag_data.calibration = CalibrationData[gpa_mega][2];

  // define Kalman Filter dimensions and pointers for orientation
  EKFInit(&EKF1, EKF1_type, x_size1, u_size1, dt, &F1, &P1, &Q1, &B1 , x1, imu1_data.gyro);
  EKFCorrectionInit(EKF1, &EKF1_corr1, corr1_type, 3, &H1_corr1, &K1_corr1, &R1_corr1, &S1_corr1, z1_corr1, h1_corr1, v1_corr1);

  // define Kalman Filter dimensions and pointers for velocity and position
  EKFInit(&EKF2, EKF2_type, x_size2, u_size2, dt, &F2, &P2, &Q2, NULL, x2, &a_WorldFrame[2]);
  EKFCorrectionInit(EKF2, &EKF2_corr1, corr1_type, 1, &H2_corr1, &K2_corr1, &R2_corr1, &S2_corr1, z2_corr1, h2_corr1, v2_corr1); // baro
  EKFCorrectionInit(EKF2, &EKF2_corr2, corr2_type, 2, &H2_corr2, &K2_corr2, &R2_corr2, &S2_corr2, z2_corr2, h2_corr2, v2_corr2); // GNSS

  // define Kalman Filter dimensions and pointers for Quaternion EKF
  EKFInit(&EKF3, EKF3_type, x_size3, u_size3, dt, &F3, &P3, &Q3, NULL, x3, imu1_data.gyro);
  EKFCorrectionInit(EKF3, &EKF3_corr1, corr1_type, 6, &H3_corr1, &K3_corr1, &R3_corr1, &S3_corr1, z3_corr1, h3_corr1, v3_corr1);

  // define output signal names
  signalPlotter_init();

  // starting point for KF
  while(IMU_VerifyDataReady(&imu1_data) & 0x03 != 0x03); // wait for IMU1 data
  HAL_StatusTypeDef status = IMU_ReadSensorData(&imu1_data);
  arm_vec3_sub_f32(imu1_data.accel, imu1_data.calibration.offset, imu1_data.accel);
  arm_vec3_element_product_f32(imu1_data.accel, imu1_data.calibration.scale, imu1_data.accel);

  while(!(MAG_VerifyDataReady() & 0b00000001)); // wait for MAG data
  status |= MAG_ReadSensorData(&mag_data);
  if (status != HAL_OK) primary_status = STATUS_ERROR_MEMS;
  arm_vec3_sub_f32(mag_data.field, mag_data.calibration.offset, mag_data.field);
  arm_vec3_element_product_f32(mag_data.field, mag_data.calibration.scale, mag_data.field);

  EulerOrientationFix(imu1_data.accel, mag_data.field, z1_corr1);

  x1[0] = z1_corr1[0];
  x1[1] = z1_corr1[1];
  x1[2] = z1_corr1[2];

  // init height EKF
  while(!BMP_GetRawData(&pressure_raw, &temperature_raw))
    HAL_Delay(1);

  // Calculate compensated BMP390 pressure & temperature
  temperature = bmp390_compensate_temperature(temperature_raw, &bmp_handle);
  pressure = bmp390_compensate_pressure(pressure_raw, &bmp_handle);

  BaroPressureToHeight(pressure, 101325, &height_baro);

  EKF2.x[0] = height_baro;
  EKF2.x[1] = 0;
  EKF2.x[2] = 101325;

  radioSet(NRF_24_ACTIVE);
  radioSetMode(RADIO_MODE_TRANSCEIVER);

  // Quaternion EKF initialization
  arm_vecN_concatenate_f32(3, imu1_data.accel, 3, mag_data.field, z3_corr1); // put measurements into z vector
  EKFStateVInit(&EKF3, &EKF3_corr1);

  // low pass filters
  status |= IMU_SetAccFilterMode(ACC_FILTER_MODE_LOW_PASS, &imu1_data);
  status |= IMU_SetAccFilterStage(ACC_FILTER_STAGE_SECOND, &imu1_data);
  status |= IMU_SetAccFilterBandwidth(ACC_FILTER_BANDWIDTH_ODR_OVER_800, &imu1_data);

  status |= IMU_SetGyroLowPassFilter(true, &imu1_data);
  status |= IMU_SetGyroFilterBandwidth(GYRO_FILTER_BANDWIDTH_8, &imu1_data);

  /* Infinite loop */
  for(;;) {
    TimeMeasureStart();
    nrf_timeout++;
    // READ SENSOR DATA

    if (status != HAL_OK) primary_status = STATUS_ERROR_MEMS;

    HAL_DTS_GetTemperature(&hdts, &DTS_Temperature);

    ReadInternalADC(&ADC_Temperature, &ADC_V_Ref);

    status |= IMU_Update(&imu1_data);
    status |= IMU_Update(&imu2_data);
    IMU_Average(&imu1_data, &imu2_data, &average_imu_data);

    if(MAG_VerifyDataReady() & 0b00000001) {
      status |= MAG_ReadSensorData(&mag_data);
      arm_vec3_sub_f32(mag_data.field, mag_data.calibration.offset, mag_data.field);
      arm_vec3_element_product_f32(mag_data.field, mag_data.calibration.scale, mag_data.field);
    }


    // KALMAN FILTER, ORIENTATION
    // initialize A and B matrix
    DeulerMatrixFromEuler(phi, theta, &Mdeuler);
    arm_mat_insert_mult_f32(&Mdeuler, &F1, 0, 3, -dt);
    arm_mat_insert_mult_f32(&Mdeuler, &B1, 0, 0, dt);

    // Prediction Step
    EKFPredictionStep(&EKF1);

    // normalize phi, theta, psi to be within +-180 deg
    normalizeAngleVector(x1, 0, 2, PI, -PI, 2*PI);

    phi = x1[0];
    theta = x1[1];
    psi = x1[2];

    euler_deg[0] = phi * 180. / PI;
    euler_deg[1] = theta * 180. / PI;
    euler_deg[2] = psi * 180. / PI;


    // transform measured body acceleration to world-frame acceleration
    RotationMatrixFromQuaternion(x3, &M_rot_bi, DCM_bi_WorldToBody);
    RotationMatrixFromQuaternion(x3, &M_rot_ib, DCM_ib_BodyToWorld);
    arm_mat_vec_mult_f32(&M_rot_ib, average_imu_data.accel, a_WorldFrame);
    a_WorldFrame[2] -= gravity_world_vec[2];

    // calculate acceleration w/o gravity in body frame
    arm_mat_vec_mult_f32(&M_rot_bi, gravity_world_vec, gravity_body_vec);
    arm_vec3_sub_f32(average_imu_data.accel, gravity_body_vec, a_BodyFrame);
    a_abs = arm_vec3_length_f32(a_BodyFrame);

    // GNSS DELAY COMPENSATION TESTING

    // discard oldest measurement
    corr_acc_sum -= corr_acc_buf[GNSS_VELOCITY_DELAY-1];

    // shift measurements down
    for(int i = GNSS_VELOCITY_DELAY - 1; i > 0; i--) {
      corr_acc_buf[i] = corr_acc_buf[i-1];
    }

    // insert new measurement
    corr_acc_buf[0] = a_BodyFrame[2];
    corr_acc_sum += corr_acc_buf[0];

    // calculate velocity difference between gnss delay and present
    corr_delta_v = corr_acc_sum * dt;


    // discard oldest measurement
    corr_vel_sum -= corr_vel_buf[GNSS_POSITION_DELAY-1];

    // shift measurements down
    for(int i = GNSS_POSITION_DELAY - 1; i > 0; i--) {
      corr_vel_buf[i] = corr_vel_buf[i-1];
    }

    // insert new measurement
    corr_vel_buf[0] = EKF2.x[1];
    corr_vel_sum += corr_vel_buf[0] + 0.5 * dt * corr_acc_buf[0];

    // calculate position difference between gnss delay and present
    corr_delta_h = corr_vel_sum * dt;

    // KALMAN FILTER, HEIGHT
    EKFPredictionStep(&EKF2);

    if(BMP_GetRawData(&pressure_raw, &temperature_raw)) {
      // Calculate compensated BMP390 pressure & temperature
      temperature = bmp390_compensate_temperature(temperature_raw, &bmp_handle);
      pressure = bmp390_compensate_pressure(pressure_raw, &bmp_handle);

      // correction step
      BaroPressureToHeight(pressure, 101325, &height_baro);
      EKF2_corr1.z[0] = pressure;
      arm_mat_set_entry_f32(EKF2_corr1.R, 0, 0, BARO_VAR);
      EKFCorrectionStep(&EKF2, &EKF2_corr1);
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
    // prediction step
    EKFPredictionStep(&EKF3);

    // Conversion to Euler
    RotationMatrixFromQuaternion(x3, &M_rot_q, DCM_bi_WorldToBody);
    EulerFromRotationMatrix(&M_rot_q, euler_from_q);

    dt_1000Hz = TimeMeasureStop();
    vTaskDelayUntil( &xLastWakeTime, xFrequency); // Delay for 1ms (1000Hz) Always at the end of the loop
  }

  /* USER CODE END StartDefaultTask */
}

void Start100HzTask(void *argument) {
  /* USER CODE BEGIN Start100HzTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10; //100 Hz
  /* Infinite loop */
  /*InterBoardPacket_t IMU_Packet = InterBoardCom_CreatePacket(InterBoardPACKET_ID_DataSaveFLASH);
  DataPacket_t imu1_data_packet;
  imu1_data_packet.Packet_ID = PACKET_ID_IMU1;
  InterBoardCom_FillData(&IMU_Packet, &imu1_data);*/
  for(;;) {

    InterBoardCom_SendTestPacket();

    #ifdef SIGNAL_PLOTTER_OUT_1 // signal plotter outputs position ekf testing
    signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
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
    signalPlotter_sendData(23, z2_corr1[0]);
    signalPlotter_sendData(24, z2_corr1[1]);
    signalPlotter_sendData(25, z2_corr1[2]);
    signalPlotter_sendData(26, z2_corr1[3]);
    signalPlotter_sendData(27, z2_corr1[4]);
    signalPlotter_sendData(28, z2_corr1[5]);
    #endif

    #ifdef SIGNAL_PLOTTER_OUT_2 // signal plotter outputs raw sensor data
    signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
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
    signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
    signalPlotter_sendData(1, (float)nrf_timeout);
    signalPlotter_sendData(2, euler_from_q[0]);
    signalPlotter_sendData(3, euler_from_q[1]);
    signalPlotter_sendData(4, euler_from_q[2]);
    signalPlotter_sendData(5, h3_corr1[0]);
    signalPlotter_sendData(6, h3_corr1[1]);
    signalPlotter_sendData(7, h3_corr1[2]);
    signalPlotter_sendData(8, h3_corr1[3]);
    signalPlotter_sendData(9, h3_corr1[4]);
    signalPlotter_sendData(10, h3_corr1[5]);
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

    #ifdef SIGNAL_PLOTTER_OUT_4 // signal plotter outputs height ekf testing
    signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
    signalPlotter_sendData(1, (float)nrf_timeout);
    signalPlotter_sendData(2, euler_from_q[0]);
    signalPlotter_sendData(3, euler_from_q[1]);
    signalPlotter_sendData(4, euler_from_q[2]);
    signalPlotter_sendData(5, imu1_data.accel[0]);
    signalPlotter_sendData(6, imu1_data.accel[1]);
    signalPlotter_sendData(7, imu1_data.accel[2]);
    signalPlotter_sendData(8, a_WorldFrame[0]);
    signalPlotter_sendData(9, a_WorldFrame[1]);
    signalPlotter_sendData(10, a_WorldFrame[2]);
    signalPlotter_sendData(11, pressure);
    signalPlotter_sendData(12, temperature);
    signalPlotter_sendData(13, height_baro);
    signalPlotter_sendData(14, (float)gps_data.gpsFix);
    signalPlotter_sendData(15, (float)gps_data.numSV);
    signalPlotter_sendData(16, (float)gps_data.hAcc/1000.f);
    signalPlotter_sendData(17, (float)gps_data.vAcc/1000.f);
    signalPlotter_sendData(18, (float)gps_data.sAcc/1000.f);
    signalPlotter_sendData(19, (float)gps_data.lat*1e-7);
    signalPlotter_sendData(20, (float)gps_data.lon*1e-7);
    signalPlotter_sendData(21, (float)gps_data.height/1000.f);
    signalPlotter_sendData(22, (float)gps_data.velN/1000.f);
    signalPlotter_sendData(23, (float)gps_data.velE/1000.f);
    signalPlotter_sendData(24, (float)-gps_data.velD/1000.f);
    signalPlotter_sendData(25, EKF2.x[0]);
    signalPlotter_sendData(26, EKF2.x[1]);
    signalPlotter_sendData(27, EKF2.x[2]);
    signalPlotter_sendData(28, gnss_height_corr);
    signalPlotter_sendData(29, gnss_velZ_corr);
    #endif

    #ifdef SIGNAL_PLOTTER_OUT_5 // signal plotter outputs testing data
    signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
    signalPlotter_sendData(1, (float)F4_data_float);
    #endif

    signalPlotter_executeTransmission(HAL_GetTick());

    // attitude estimation using magnetometer and accelerometer
    EulerOrientationFix(imu1_data.accel, mag_data.field, z1_corr1);

    normalizeAnglePairVector(x1, z1_corr1, 0, 2, PI, -PI, 2*PI);

    phi_fix = z1_corr1[0];
    theta_fix = z1_corr1[1];
    psi_fix = z1_corr1[2];

    // Kalman Filter euler correction step
    EKFCorrectionStep(&EKF1, &EKF1_corr1);

    // normalize phi, theta, psi to be within +-180 deg
    normalizeAngleVector(x1, 0, 2, PI, -PI, 2*PI);

    phi = x1[0];
    theta = x1[1];
    psi = x1[2];

    // KALMAN FILTER, QUATERNION
    // correction step
    arm_vecN_concatenate_f32(3, imu1_data.accel, 3, mag_data.field, z3_corr1); // put measurements into z vector
    EKFCorrectionStep(&EKF3, &EKF3_corr1);


    ShowStatus(RGB_PRIMARY, primary_status, 1, 100);

    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 100Hz
  }
  /* USER CODE END Start10HzTask */
}

void Start10HzTask(void *argument) {
  /* USER CODE BEGIN Start10HzTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; //10 Hz
  InterBoardCom_Init();
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

      // add correction velocity to compensate GNSS delay
      gnss_velZ_corr = gps_data.velD*(-1e-3) + corr_delta_v;

      // add correction height to compensate GNSS delay
      gnss_height_corr = gps_data.height*1e-3 + corr_delta_h;

      z2_corr2[0] = gnss_height_corr;
      z2_corr2[1] = gnss_velZ_corr;
      arm_mat_set_entry_f32(EKF2_corr2.R, 0, 0, (float)gps_data.vAcc*gps_data.vAcc*1e-6);
      arm_mat_set_entry_f32(EKF2_corr2.R, 1, 1, (float)gps_data.sAcc*gps_data.sAcc*1e-6);
      EKFCorrectionStep(&EKF2, &EKF2_corr2);

      //z2_corr1[0] = (float)gps_data.velE / 1000.f; // m/s
      //z2_corr1[1] = (float)gps_data.velN / 1000.f;
      //z2_corr1[2] = (float)gps_data.velD / (-1000.f);
      //z2_corr1[3] = (float)ENU[0]; // m
      //z2_corr1[4] = (float)ENU[1];
      //z2_corr1[5] = (float)ENU[2];

      //arm_mat_set_diag_f32(&R2_corr1, 0, 0, 3, (float)gps_data.sAcc * gps_data.sAcc / 1e6f * 0.001f * (1 + a_abs));
      //arm_mat_set_diag_f32(&R2_corr1, 3, 3, 2, (float)gps_data.hAcc * gps_data.hAcc / 1e6f * 100.f);
      //arm_mat_set_entry_f32(&R2_corr1, 5, 5, (float)gps_data.vAcc * gps_data.vAcc / 1e6f * 100.f);

      // Kalman Filter correction step
      //EKFCorrectionStep(&EKF2, 2);


      SERVO_MoveToAngle(1, 0);
      SERVO_MoveToAngle(2, 0);
    }

    // transmit data
    uint8_t tx_buf[NRF24L01P_PAYLOAD_LENGTH];
    tx_buf[1] = gps_data.sec; // Packet type
    radioSend(tx_buf);

    // RECOVERY TEST
    if(uwTick > 10000 && servo_status == 5) {
      servo_status = 6;
      SERVO_MoveToAngle(3, 90);
    }
    if(uwTick > 5300 && servo_status == 4) {
      servo_status = 5;
      SERVO_MoveToAngle(1, 0);
      SERVO_MoveToAngle(2, 0);
    }
    if(uwTick > 5000 && servo_status == 1) {
      servo_status = 4;
      SERVO_MoveToAngle(1, 35);
      SERVO_MoveToAngle(2, 35);
    }
    InterBoardCom_SendTestPacket();

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
uint8_t InterBoardPacket_receive_num = 0;

void StartInterruptHandlerTask(void *argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t receivedData;
  InterBoardPacket_t InterBoardCom_Packet;
  char GPS_Buffer[100]; // Buffer for GPS data
  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //Disabled due to always triggering when GPS is not connected
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(InterruptQueue, &receivedData, 10) == pdTRUE) {
      if(receivedData == 0x10) { // Handle GPS interrupt
        ublox_ReadOutput(GPS_Buffer); //Read the GPS output and decode it

      } else if (receivedData == 0x11) { //Handle NRF interrupt
        if(nrf_mode) {
          nrf24l01p_tx_irq();
        } else {
          //HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
          memset(rx_recieve_buf, 0, NRF24L01P_PAYLOAD_LENGTH);
          nrf24l01p_rx_receive(rx_recieve_buf);
        }

      }
    }
    if (xQueueReceive(InterBoardCom_Queue, &InterBoardCom_Packet, 10) == pdTRUE) {
      uint8_t Packet_ID = InterBoardCom_Packet.InterBoardPacket_ID;
      InterBoardPacket_receive_num += 1;
      memcpy(&F4_data_float, InterBoardCom_Packet.Data, sizeof(float));
      HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
      // Process received InterBoardCom_Packet
    }
    osDelay(1);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
uint8_t selftest_tries = 0;
uint8_t SelfTest(void) {
  if((status_data.sensor_status_flags & 0x1f) == 0x1f && ((status_data.sensor_status_flags >> 7) & 0x01) == 0) { //All sensors are working but selftest_pass flag not set
    if(primary_status >= 0) primary_status = STATUS_GNSS_ALIGN;

    status_data.sensor_status_flags |= (1<<7);  //All checks passed

  } else if (selftest_tries > 100){
    primary_status = STATUS_ERROR_STARTUP;
    return status_data.sensor_status_flags;
  }
  else if((status_data.sensor_status_flags & 0xff) != 0x9F) { //Run selftest if not all sensors are working or selftest not passed
    selftest_tries += 1;

    IMU_Data_t tmp_imu1_data;
    IMU_Data_t tmp_imu2_data;
    tmp_imu1_data.imu = IMU1;
    tmp_imu2_data.imu = IMU2;

    status_data.sensor_status_flags |= IMU_SelfTest(&tmp_imu1_data);
    status_data.sensor_status_flags |= (IMU_SelfTest(&tmp_imu2_data)<<1);
    status_data.sensor_status_flags |= (MAG_SelfTest()<<2);
    status_data.sensor_status_flags |= (BMP_SelfTest()<<3);
    //status_data.sensor_status_flags |= (GPS_VER_CHECK()<<4); //Check if GPS is connected and working
    status_data.sensor_status_flags |= (1<<4); //Excluding GPS check for now, as the first message cant be relaibly received by this function

    if(primary_status > 0) primary_status = STATUS_STARTUP;
  }

  return status_data.sensor_status_flags;
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

