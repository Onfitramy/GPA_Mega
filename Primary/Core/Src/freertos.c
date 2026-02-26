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
#include "calibration_data.h"
#include "InterBoardCom.h"
#include "packets.h"
#include "status.h"
#include "radio.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"
#include "statemachine.h"
#include "comSchedule.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static int32_t DTS_Temperature;

extern ADC_HandleTypeDef hadc3;
uint32_t ADC_Temperature, ADC_V_Ref;

GPA_Mega gpa_mega;

DataPacket_t powerData;

bool is_groundstation = true;

void SensorStatus_Reset(SensorStatus *sensor_status) {
  sensor_status->hal_status = HAL_OK;
  sensor_status->active = true;
} // why is this here?

StatusPayload_t status_data = {0};
float F4_data_float;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
StreamBufferHandle_t xStreamBuffer;
QueueHandle_t InterruptQueue;
QueueHandle_t InterBoardCom_Queue;
QueueHandle_t USB_Tx_Queue;
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

/*Tiny low priority USB thread*/
osThreadId_t USBTaskHandle;
const osThreadAttr_t USBTask_attributes = {
  .name = "USBTask",
  .stack_size = 128 * 12,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ReadInternalADC(uint32_t* temperature, uint32_t* v_ref);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void StartInterruptHandlerTask(void *argument);

void Start10HzTask(void *argument);

void Start100HzTask(void *argument);

void StartUSBTask(void *argument);

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
  USB_Tx_Queue = xQueueCreate(20, sizeof(InterBoardPacket_t));
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
  USBTaskHandle = osThreadNew(StartUSBTask, NULL, &USBTask_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1; //1000 Hz

  /* Infinite loop */
  for(;;) {
    //TimeMeasureStart();

    nrf_timeout++;

    // Run 1000 Hz Do Actions
    StateMachine_DoActions(&flight_sm, 1000);
  
    HAL_DTS_GetTemperature(&hdts, &DTS_Temperature);
    
    ReadInternalADC(&ADC_Temperature, &ADC_V_Ref); // 7us

    #ifdef HIL_TESTING
    if ((flight_sm.currentFlightState >= STATE_FLIGHT_ARMED) && (flight_sm.currentFlightState <= STATE_FLIGHT_LANDED)) {
      HILupdateStates(0.001);
    }
    #endif

    // after startup
    if (flight_sm.currentFlightState != STATE_FLIGHT_STARTUP && flight_sm.currentFlightState != STATE_GROUNDSTATION) {
      #ifndef HIL_TESTING
      SensorStatus_Reset(&imu1_status);
      SensorStatus_Reset(&imu2_status);
      SensorStatus_Reset(&mag_status);

      imu1_status.hal_status |= IMU_Update(&imu1_data); // 70us
      imu2_status.hal_status |= IMU_Update(&imu2_data); // 70us
      imu1_status.active = imu1_data.active;
      imu2_status.active = imu2_data.active;
      IMU_Average(&imu1_data, &imu2_data, &average_imu_data);

      if (MAG_VerifyDataReady() & 0b00000001) {
        mag_status.hal_status |= MAG_ReadSensorData(&mag_data);
        arm_vec3_sub_f32(mag_data.field, mag_data.calibration.offset, mag_data.field);
        arm_vec3_element_product_f32(mag_data.field, mag_data.calibration.scale, mag_data.field);
      } // 7us
      #else
      // calculate average_imu_data
      HILgetIMUData(&average_imu_data);
      // calculate mag_data.field
      HILgetMagnetometerData(&mag_data);
      #endif

      // transform measured body acceleration to world-frame acceleration
      arm_mat_vec_mult_f32(&M_rot_ib, average_imu_data.accel, a_WorldFrame_g);
      arm_vec3_sub_f32(a_WorldFrame_g, gravity_world_vec, a_WorldFrame_i);
      a_abs_g = arm_vec3_length_f32(a_WorldFrame_g);
      a_abs_i = arm_vec3_length_f32(a_WorldFrame_i);

      // calculate acceleration w/o gravity in body frame
      arm_mat_vec_mult_f32(&M_rot_bi, gravity_world_vec, gravity_body_vec);
      arm_vec3_sub_f32(average_imu_data.accel, gravity_body_vec, a_BodyFrame_i);

      /* --- GNSS DELAY COMPENSATION TESTING --- */
      #ifndef HIL_TESTING  // GPS Delay not implemented yet
      CompensateGNSSDelay(a_WorldFrame_i[2], EKF2.x[1], &corr_delta_v, &corr_delta_h, 0.001);
      #endif

      // KALMAN FILTER, HEIGHT
      EKFPredictionStep(&EKF2);

      #ifndef HIL_TESTING
      if (BMP_readData(&bmp_data.pressure, &bmp_data.height, &bmp_data.temperature)) {
        // execute this if new data is available
        // correction step
        EKF2_corr1.z[0] = bmp_data.pressure;
        EKFCorrectionStep(&EKF2, &EKF2_corr1);
      }
      #else
      // calculate bmp_data.pressure
      HILgetBarometerData(&bmp_data);

      EKF2_corr1.z[0] = bmp_data.pressure;
      EKFCorrectionStep(&EKF2, &EKF2_corr1);
      #endif

      // KALMAN FILTER, QUATERNION
      // prediction step
      EKFPredictionStep(&EKF3);

      RotationMatrixFromQuaternion(x3, &M_rot_bi, DCM_bi_WorldToBody);
      RotationMatrixFromQuaternion(x3, &M_rot_ib, DCM_ib_BodyToWorld);

      // Conversion to Euler
      EulerFromRotationMatrix(&M_rot_bi, euler);
      VAR_vec3_abs = QuaternionCovToSmallAngleCov(x3, &P3, &P3_angle);
      FlightPathAngleFromRotationMatrix(&M_rot_bi, &flightpath_angle);

      vel_abs = EKF2.x[1] / arm_mat_get_entry_f32(&M_rot_bi, 2, 2);
    }

    //dt_1000Hz = TimeMeasureStop();
    vTaskDelayUntil( &xLastWakeTime, xFrequency); // Delay for 1ms (1000Hz) Always at the end of the loop
  }

  /* USER CODE END StartDefaultTask */
}

void Start100HzTask(void *argument) {
  /* USER CODE BEGIN Start100HzTask */
  InitializeDataScheduler();

  DataPacket_t State_DataPacket = CreateDataPacket(PACKET_ID_STATE);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10; //100 Hz
  /* Infinite loop */
  for(;;) {
    // Run 100 Hz Do Actions
    StateMachine_DoActions(&flight_sm, 100);

    ProcessDataSchedule(xTaskGetTickCount());

    UpdateStatePacket(&State_DataPacket, HAL_GetTick(), flight_sm.currentFlightState, flight_sm.timestamp_ms);
    InterBoardCom_QueuePacket(&State_DataPacket);
    
    if (!is_groundstation) {
      //UpdateTemperaturePacket(&Temperature_DataPacket, HAL_GetTick(), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ptot_data.pressure);

      // Don't activate this and the SPARK communication at the same time, because they use the same SPI
      /*
      if (ptot_readData(&ptot_data)) {
        // execute this if new data is available
        // correction step
        EKF2_corr3.z[0] = ptot_data.pressure;
        EKFCorrectionStep(&EKF2, &EKF2_corr3);
      }
      */

      SPARK_ReadData();
      float stepper_est_position;
      StepperPositionFromAngle(stepper_zero_position, spark_data.Data.spark.magAngle, &stepper_est_position);
      ACSAngleFromStepperPosition(stepper_est_position, &acs_est_angle_deg);

      // Quaternion EKF correction step
      arm_vecN_concatenate_f32(3, average_imu_data.accel, 3, mag_data.field, z3_corr1); // put measurements into z vector
      EKFCorrectionStep(&EKF3, &EKF3_corr1);
    }

    InterBoardCom_ProcessTxBuffer();

    if (signalPlotterSend) signalPlotter_sendAll();

    ShowStatus(flight_sm.currentFlightState, 1, 100);

    vTaskDelayUntil( &xLastWakeTime, xFrequency); // 100Hz
  }
  /* USER CODE END Start10HzTask */
}

void Start10HzTask(void *argument) {
  /* USER CODE BEGIN Start10HzTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; //10 Hz
  GPS_Init(); //Initialize the GPS module

  /* Infinite loop */
  for(;;) {
    // Run 10 Hz Do Actions
    StateMachine_DoActions(&flight_sm, 10);

    #ifndef HIL_TESTING
    GPS_ReadSensorData(&gps_data);
    #else
    // calculate gps_data
    HILgetGPSData(&gps_data);
    #endif
    
    //GPS_RequestSensorData(); // Request GPS data

    if (!is_groundstation) { //Secondary board sends data to groundstation

      if ((flight_sm.currentFlightState >= STATE_FLIGHT_GNC_ALIGN) && (flight_sm.currentFlightState <= STATE_FLIGHT_ARMED)) {
        // not needed for now...
        //UBLOXtoWGS84(gps_data.lat, gps_data.lon, gps_data.height, WGS84);
        //WGS84toECEF(WGS84, ECEF);
        //ECEFtoENU(WGS84_ref, ECEF_ref, ECEF, ENU);

        #ifndef HIL_TESTING // GPS Delay not implemented yet
        // add correction velocity to compensate GNSS delay
        gnss_velZ_corr = gps_data.velD*(-1e-3) + corr_delta_v;

        // add correction height to compensate GNSS delay
        gnss_height_corr = gps_data.height*1e-3 + corr_delta_h;

        z2_corr2[0] = gnss_height_corr;
        z2_corr2[1] = gnss_velZ_corr;
        #else
        z2_corr2[0] = gps_data.height*(1e-3);
        z2_corr2[1] = gps_data.velD*(-1e-3);
        #endif

        arm_mat_set_entry_f32(EKF2_corr2.R, 0, 0, (float)gps_data.vAcc*gps_data.vAcc*1e-6);
        arm_mat_set_entry_f32(EKF2_corr2.R, 1, 1, (float)gps_data.sAcc*gps_data.sAcc*1e-6);

        // Height EKF GNSS correction step
        EKFCorrectionStep(&EKF2, &EKF2_corr2);
      }
    }


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
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //Aktivate Interrupt for GPS and NRF
  uint8_t receivedData;
  InterBoardPacket_t InterBoardCom_Packet;
  char GPS_Buffer[100]; // Buffer for GPS data
  InterBoardCom_Init();

  // Create queue set that can hold items from both queues
  QueueSetHandle_t xQueueSet = xQueueCreateSet(20); // Total items from both queues

  // Add both queues to the set
  xQueueAddToSet(InterruptQueue, xQueueSet);
  xQueueAddToSet(InterBoardCom_Queue, xQueueSet);

  /* Infinite loop */
  for(;;)
  {
    // Wait on the queue set with timeout (blocks efficiently)
    QueueSetMemberHandle_t xActivatedMember = xQueueSelectFromSet(xQueueSet, portMAX_DELAY);

    if (xActivatedMember == InterruptQueue) {
      if (xQueueReceive(InterruptQueue, &receivedData, 0) == pdTRUE) {
        if(receivedData == 0x10) { // Handle GPS interrupt
          //GPS_ReadNavPVT(&gps_data);
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
    }
    else if (xActivatedMember == InterBoardCom_Queue) {
      if (xQueueReceive(InterBoardCom_Queue, &InterBoardCom_Packet, 0) == pdTRUE) {
        uint8_t Packet_ID = InterBoardCom_Packet.InterBoardPacket_ID;
        InterBoardPacket_receive_num += 1;
        DataPacket_t receivedPacket;
        memcpy(&receivedPacket, InterBoardCom_Packet.Data, sizeof(DataPacket_t));

        InterBoardCom_ProcessTxBuffer(); // Check if more packets to send and send them
        HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
        // Process received InterBoardCom_Packet
        InterBoardCom_ParsePacket(&InterBoardCom_Packet);
      }
    }
  }
}

void StartUSBTask(void *argument) {
  /* USER CODE BEGIN StartUSBTask */

  /* Infinite loop */
  for(;;)
  {
    DataPacket_t receivedPacket;
    if (xQueueReceive(USB_Tx_Queue, &receivedPacket, 0) == pdTRUE) {
      if (USB_OutputDataPacket(&receivedPacket) == USBD_OK) {
        //USB transmission successful, do nothing
      } else {
        xQueueSendToFront(USB_Tx_Queue, &receivedPacket, 0); // Re-queue the packet for the next attempt
      }
    }
  }
  /* USER CODE END StartUSBTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

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

