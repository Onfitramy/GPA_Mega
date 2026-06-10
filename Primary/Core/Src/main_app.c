#include "main_app.h"

/*Internal Libraries*/
#include "_libraries.h"

#include "statemachine.h"

#include "cli_app.h"

#include "dts.h"

/*External Libraries*/
#include "armMathAddon.h"
#include <stdio.h>

/* FreeRTOS Variables, shared with all tasks */
StreamBufferHandle_t xStreamBuffer;
QueueHandle_t InterruptQueue;
QueueHandle_t InterBoardCom_Queue;
QueueHandle_t USB_Tx_Queue;

/*Public Variables*/
uint32_t uid[3]; // Unique ID of the board
extern bool is_groundstation;

static int32_t DTS_Temperature;

extern ADC_HandleTypeDef hadc3;
uint32_t ADC_Temperature, ADC_V_Ref;

GPA_Mega gpa_mega;

DataPacket_t powerData;

bool is_groundstation = false;

extern volatile uint8_t ib_queue_ready_flag;

StatusPayload_t status_data = {0};
float F4_data_float;

/**
  ******************************************************************************
  * Function Description : 
  * Code in here is executed after peripherals are initialized but before the FreeRTOS scheduler is started. It is used to initialize any variables, data structures, or 
  * state machines that need to be set up before the tasks start running. It can also be used to perform any necessary calibration or self-tests on sensors or other hardware components.
  ******************************************************************************
  */
void Startup() {
    // find out board no.
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
    gpa_mega = GPA_MegaFromUID(uid);

    // if board is a ground station, set flag
    if (gpa_mega == GPA_MEGA_1) {
        is_groundstation = true;
        cli_target_mode = CLI_TARGET_MODE_EXTERNAL; // Groundstation always uses external target mode
        signalPlotterSend = false; // Disable signal plotter by default on groundstation

        StateMachine_Init(&flight_sm, STATE_GROUNDSTATION);
    } else {
        is_groundstation = false;
        StateMachine_Init(&flight_sm, STATE_FLIGHT_STARTUP);
    }

    // define output signal names
    signalPlotter_init();

    rho0_const = p0_const / (R_const * T0_const);
    a0_const = sqrt(kappa*R_const*T0_const);

    GPS_Init(); //Initialize the GPS module

    InitializeDataScheduler();

    #ifdef HIL_TESTING
    HILInit();
    #endif
}


/**
  ******************************************************************************
  * Function Description : 
  * This Task runs at 1000Hz and is responsible for reading high freq. sensors (IMU, Baro, ADC), 
  * doing the EKF prediction steps and running the State Machine actions.
  ******************************************************************************
  */
void Task1000Hz_Step(void *argument) {
    DataPacket_t State_DataPacket = CreateDataPacket(PACKET_ID_STATE);
    nrf_timeout++;

    // Run 1000 Hz Do Actions
    StateMachine_DoActions(&flight_sm, 1000);

    HAL_DTS_GetTemperature(&hdts, &DTS_Temperature);
    
    ReadInternalADC(&ADC_Temperature, &ADC_V_Ref); // 7us

    #ifdef HIL_TESTING
    if ((flight_sm.currentFlightState >= STATE_FLIGHT_BURN) && (flight_sm.currentFlightState <= STATE_FLIGHT_LANDED)) {
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

        ProcessDataSchedule(xTaskGetTickCount());

        if(is_groundstation) {
            UpdateStatePacket(&State_DataPacket, HAL_GetTick(), flight_sm.currentFlightState, flight_sm.timestamp_ms);
            InterBoardCom_SendDataPacket(INTERBOARD_OP_SAVE_SEND | INTERBOARD_TARGET_NONE, &State_DataPacket); //Needed to keep Primary and Secondary syncronized
        }

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
        EulerFromRotationMatrix(&M_rot_bi, body_euler);
        VAR_vec3_abs = QuaternionCovToSmallAngleCov(x3, &P3, &P3_angle);
        FlightPathAngleFromRotationMatrix(&M_rot_bi, &flightpath_angle);

        vel_abs = EKF2.x[1] / arm_mat_get_entry_f32(&M_rot_bi, 2, 2);
    }
}


/**
  ******************************************************************************
  * Function Description : 
  * This Task runs at 100Hz and is responsible for doing the EKF correction steps 
  * and running the low freq. State Machine actions. It also receives the SPARK data, Sends and processes the InterBoardCom data 
  * and sends data to the signal plotter.
  ******************************************************************************
  */
void Task100Hz_Step(void *argument) {
    // Run 100 Hz Do Actions
    StateMachine_DoActions(&flight_sm, 100);
    
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

    // Quaternion EKF magnetometer correction step
    // project magnetometer readings onto horizontal plane
    float mag_enu[3];
    //float mag_b_tilde[3];
    arm_mat_vec_mult_f32(&M_rot_ib, mag_data.field, mag_enu);
    mag_enu[2] = 0;
    arm_mat_vec_mult_f32(&M_rot_bi, mag_enu, EKF3_corr1.z);
    EKFCorrectionStep(&EKF3, &EKF3_corr1);

    // Quaternion EKF accelerometer correction step
    arm_vec3_copy_f32(average_imu_data.accel, EKF3_corr2.z);
    EKFCorrectionStep(&EKF3, &EKF3_corr2);
    }

    InterBoardCom_ProcessTxBuffer();

    if (signalPlotterSend) signalPlotter_sendAll();

    ShowStatus(flight_sm.currentFlightState, 1, 100);
}

/**
  ******************************************************************************
  * Function Description : 
  * This Task runs at 10Hz and handles low freq. State Machine actions, reading GPS data, doing the GPS EKF correction
  ******************************************************************************
  */
void Task10Hz_Step(void *argument) {       
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
}


uint8_t rx_recieve_buf[NRF24L01P_PAYLOAD_LENGTH] = {0};
uint8_t InterBoardPacket_receive_num = 0;

/**
  ******************************************************************************
  * Function Description : 
  * This Task runs at the highest priority and is responsible for handling all interrupts from the GPS, NRF and SPI communication. 
  * It receives the interrupt notifications via a FreeRTOS queue and processes them accordingly.
  * Right now it handles the InterboardCom SPI communication.
  ******************************************************************************
  */
void InterruptTask(void *argument) {
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //Aktivate Interrupt for GPS and NRF
    InterBoardPacket_t InterBoardCom_Packet;
    InterBoardCom_Init();

    /* Infinite loop */
    for(;;)
    {
        while (xQueueReceive(InterBoardCom_Queue, &InterBoardCom_Packet, 0) == pdTRUE) {
        InterBoardPacket_receive_num += 1;
        InterBoardCom_ProcessTxBuffer(); // Check if more packets to send and send them
        HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
        // Process received InterBoardCom_Packet
        InterBoardCom_ParsePacket(&InterBoardCom_Packet);
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for ISR notification
    }
}


/**
  ******************************************************************************
  * Function Description : 
  * This Task runs at the lowest priority and is responsible for sending data to the USB interface. It receives data packets to be sent via a FreeRTOS queue 
  * and attempts to transmit them over USB. If the transmission fails, it re-queues the packet for another attempt.
  ******************************************************************************
  */
void USBTask(void *argument) {
    /* Infinite loop */
    for(;;)
    {
        DataPacket_t receivedPacket;
        if (xQueueReceive(USB_Tx_Queue, &receivedPacket, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (USB_OutputDataPacket(&receivedPacket) == USBD_OK) {
            //USB transmission successful, do nothing
        } else {
            xQueueSendToFront(USB_Tx_Queue, &receivedPacket, 0); // Re-queue the packet for the next attempt
        }
        }
    }
}