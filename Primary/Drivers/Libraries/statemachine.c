#include "statemachine.h"

static flight_state_t AbortHandler(flight_event_t event);
static flight_state_t StartupHandler(flight_event_t event);
static flight_state_t InitHandler(flight_event_t event);
static flight_state_t AlignGNCHandler(flight_event_t event);
static flight_state_t CheckoutsHandler(flight_event_t event);
static flight_state_t ArmedHandler(flight_event_t event);
static flight_state_t BurnHandler(flight_event_t event);
static flight_state_t CoastHandler(flight_event_t event);
static flight_state_t UnbrakedDescendHandler(flight_event_t event);
static flight_state_t DrogueDescendHandler(flight_event_t event);
static flight_state_t MainDescendHandler(flight_event_t event);
static flight_state_t LandedHandler(flight_event_t event);

/* --- State handler lookup table --- */
typedef flight_state_t (*StateHandler_t)(flight_event_t event);
static StateHandler_t stateTable[STATE_FLIGHT_MAX] = {
    AbortHandler,
    StartupHandler,
    InitHandler,
    AlignGNCHandler,
    CheckoutsHandler,
    ArmedHandler,
    BurnHandler,
    CoastHandler,
    UnbrakedDescendHandler,
    DrogueDescendHandler,
    MainDescendHandler,
    LandedHandler
};

uint32_t stateMinEntryDelayTable[STATE_FLIGHT_MAX] = {
    0,      // Abort
    0,      // Startup
    0,      // Init
    50,     // AlignGNC
    5000,   // Checkouts
    60000,  // Armed
    60000,  // Burn
    3000,   // Coast
    10000,  // UnbrakedDescend
    500,    // DrogueDescend
    60000,  // MainDescend
    500     // Landed
};

/* --- Entry actions --- */
static void AbortEntry(StateMachine_t *sm) {
    // TODO:
    // disable recovery, ACS and cams
}
static void StartupEntry(StateMachine_t *sm) {
    /* --- Initialize sensors --- */
    IMU_Data_t tmp_imu1_data;
    IMU_Data_t tmp_imu2_data;
    tmp_imu1_data.imu = IMU1;
    tmp_imu2_data.imu = IMU2;

    IMU_VerifyDataReady(&tmp_imu1_data);
    imu1_status.hal_status |= IMU_InitImu(&imu1_data, IMU1, gpa_mega);
    IMU_ConfigXL(IMU_ODR_1660_Hz, IMU_FS_XL_8, 0, &imu1_data);
    IMU_ConfigG(IMU_ODR_1660_Hz, IMU_FS_G_2000, &imu1_data);

    IMU_VerifyDataReady(&tmp_imu2_data);
    imu2_status.hal_status |= IMU_InitImu(&imu2_data, IMU2, gpa_mega);
    IMU_ConfigXL(IMU_ODR_6660_Hz, IMU_FS_XL_16, 0, &imu2_data);
    IMU_ConfigG(IMU_ODR_6660_Hz, IMU_FS_G_4000, &imu2_data);

    // low pass filters
    // imu2_status |= IMU_SetAccFilterMode(ACC_FILTER_MODE_LOW_PASS, &imu2_data);
    // imu2_status |= IMU_SetAccFilterStage(ACC_FILTER_STAGE_SECOND, &imu2_data);
    // imu2_status |= IMU_SetAccFilterBandwidth(ACC_FILTER_BANDWIDTH_ODR_OVER_800, &imu2_data);

    // imu2_status |= IMU_SetGyroLowPassFilter(true, &imu2_data);
    // imu2_status |= IMU_SetGyroFilterBandwidth(GYRO_FILTER_BANDWIDTH_8, &imu2_data);

    MAG_VerifyDataReady();
    mag_data.calibration = CalibrationData[gpa_mega][2];
    MAG_ConfigSensor(LIS3MDL_OM_ULTRA, LIS3MDL_ODR_80_Hz, LIS3MDL_FS_4, LIS3MDL_FAST_ODR_ON, LIS3MDL_TEMP_ON);

    BMP_SelfTest();
    BMP_enable();
    BMP_Read_Calibration_Params(&bmp_handle);

    // set NRF24L01 frequency and data rate
    nrf24l01p_init(2462, _1Mbps);
    radioSet(NRF_24_ACTIVE);
    radioSetMode(RADIO_MODE_TRANSCEIVER);

    // initialize Servo PWM timers to neutral position
    SERVO_Init(1, 1500, 2500, 135);
    SERVO_Init(2, 1500, 2500, 135);
    SERVO_MoveToAngle(1, 0);
    SERVO_MoveToAngle(2, 0);

    /* --- Initialize (Extended) Kalman Filters --- */
    // define Kalman Filter dimensions and pointers for velocity and position
    EKFInit(&EKF2, EKF2_type, x_size2, u_size2, 0.001, &F2, &P2, &Q2, NULL, x2, &a_WorldFrame[2]);
    EKFCorrectionInit(EKF2, &EKF2_corr1, corr1_type, 1, &H2_corr1, &K2_corr1, &R2_corr1, &S2_corr1, z2_corr1, h2_corr1, v2_corr1); // baro
    EKFCorrectionInit(EKF2, &EKF2_corr2, corr2_type, 2, &H2_corr2, &K2_corr2, &R2_corr2, &S2_corr2, z2_corr2, h2_corr2, v2_corr2); // GNSS

    // define Kalman Filter dimensions and pointers for Quaternion EKF
    EKFInit(&EKF3, EKF3_type, x_size3, u_size3, 0.001, &F3, &P3, &Q3, NULL, x3, average_imu_data.gyro);
    EKFCorrectionInit(EKF3, &EKF3_corr1, corr1_type, 6, &H3_corr1, &K3_corr1, &R3_corr1, &S3_corr1, z3_corr1, h3_corr1, v3_corr1);
}
static void InitEntry(StateMachine_t *sm) {
    /* --- Read sensor data for EKF initialization --- */
    imu1_status.hal_status |= IMU_Update(&imu1_data);
    imu2_status.hal_status |= IMU_Update(&imu2_data);
    imu1_status.active = imu1_data.active;
    imu2_status.active = imu2_data.active;
    
    IMU_Average(&imu1_data, &imu2_data, &average_imu_data);

    mag_status.hal_status |= MAG_ReadSensorData(&mag_data);
    arm_vec3_sub_f32(mag_data.field, mag_data.calibration.offset, mag_data.field);
    arm_vec3_element_product_f32(mag_data.field, mag_data.calibration.scale, mag_data.field);

    BMP_GetRawData(&pressure_raw, &temperature_raw);

    // Calculate compensated BMP390 pressure & temperature
    temperature = bmp390_compensate_temperature(temperature_raw, &bmp_handle);
    pressure = bmp390_compensate_pressure(pressure_raw, &bmp_handle);

    BaroPressureToHeight(pressure, 101325, &height_baro);

    // initialize height EKF state vector
    EKF2.x[0] = height_baro;
    EKF2.x[1] = 0;
    EKF2.x[2] = 101325;

    // Quaternion EKF initialization
    arm_vecN_concatenate_f32(3, average_imu_data.accel, 3, mag_data.field, z3_corr1); // put measurements into z vector
    EKFStateVInit(&EKF3, &EKF3_corr1);
    RotationMatrixFromQuaternion(x3, &M_rot_bi, DCM_bi_WorldToBody);
    RotationMatrixFromQuaternion(x3, &M_rot_ib, DCM_ib_BodyToWorld);

    // TODO:
    // check communications
}
static void AlignGNCEntry(StateMachine_t *sm) {
    // TODO:
    // begin xBee comms
}
static void CheckoutsEntry(StateMachine_t *sm) {
    // TODO:
    // notify ground station that it now needs to command the checkouts
    // enable recovery, ACS and cams
}
static void ArmedEntry(StateMachine_t *sm) {
    // TODO:
    // move acs to neutral position
    // lock ACS
    // start video recording
    // start data logging
}
static void BurnEntry(StateMachine_t *sm) {}
static void CoastEntry(StateMachine_t *sm) {
    // TODO:
    // unlock ACS
    // extend ACS to pre-defined position
    // enable MPC
}
static void UnbrakedDescendEntry(StateMachine_t *sm) {
    // TODO:
    // deplody drogue
}
static void DrogueDescendEntry(StateMachine_t *sm) {}
static void MainDescendEntry(StateMachine_t *sm) {}
static void LandedEntry(StateMachine_t *sm) {
    // TODO:
    // stop video recording
    // disable recovery, ACS and cams
    // switch off LEDs
    // stop data logging
    // slow xBee and NRF data rate down to save power
    // enable buzzer
}

/* --- Do actions --- */
static void AbortDo(StateMachine_t *sm) {}
static void StartupDo(StateMachine_t *sm) {
    // All sensors are working but selftest_pass flag not set
    if ((status_data.sensor_status_flags & 0x1F) == 0x1F) {
        status_data.sensor_status_flags |= (1<<7);  //All checks passed
        StateMachine_Dispatch(sm, EVENT_FLIGHT_STARTUP_COMPLETE);
    }
    // All sensors but IMU1 working
    else if (((selftest_tries > MAX_SELFTEST_TRIES) && (status_data.sensor_status_flags & 0x1E) == 0x1E)) {
        // TODO: Notify user of defect IMU1 and disable IMU1 permanently
        status_data.sensor_status_flags |= (1<<7);  //All checks passed
        StateMachine_Dispatch(sm, EVENT_FLIGHT_STARTUP_COMPLETE);
    }
    // All sensors but IMU2 working
    else if (((selftest_tries > MAX_SELFTEST_TRIES) && (status_data.sensor_status_flags & 0x1D) == 0x1D)) {
        // TODO: Notify user of defect IMU2 and disable IMU2 permanently
        status_data.sensor_status_flags |= (1<<7);  //All checks passed
        StateMachine_Dispatch(sm, EVENT_FLIGHT_STARTUP_COMPLETE);
    }
    // Run selftest if not all sensors are working or selftest not passed
    else if ((status_data.sensor_status_flags & 0xFF) != 0x9F) {
        selftest_tries += 1;

        IMU_Data_t tmp_imu1_data;
        IMU_Data_t tmp_imu2_data;
        tmp_imu1_data.imu = IMU1;
        tmp_imu2_data.imu = IMU2;

        status_data.sensor_status_flags |= IMU_SelfTest(&tmp_imu1_data);
        status_data.sensor_status_flags |= (IMU_SelfTest(&tmp_imu2_data)<<1);
        status_data.sensor_status_flags |= (MAG_SelfTest()<<2);
        status_data.sensor_status_flags |= (BMP_SelfTest()<<3);

        // Excluding GPS check for now, as the first message cant be relaibly received by this function
        //status_data.sensor_status_flags |= (GPS_VER_CHECK()<<4); //Check if GPS is connected and working
        status_data.sensor_status_flags |= (1<<4);
    }
}
static void InitDo(StateMachine_t *sm) {
    if ((gps_data.gpsFix == 3)) {
        StateMachine_Dispatch(sm, EVENT_FLIGHT_GNSS_FIX);
    }
}
static void AlignGNCDo(StateMachine_t *sm) {}
static void CheckoutsDo(StateMachine_t *sm) {}
static void ArmedDo(StateMachine_t *sm) {}
static void BurnDo(StateMachine_t *sm) {}
static void CoastDo(StateMachine_t *sm) {}
static void UnbrakedDescendDo(StateMachine_t *sm) {}
static void DrogueDescendDo(StateMachine_t *sm) {}
static void MainDescendDo(StateMachine_t *sm) {}
static void LandedDo(StateMachine_t *sm) {}

/* --- Exit actions --- */
static void AbortExit(StateMachine_t *sm) {}
static void StartupExit(StateMachine_t *sm) {}
static void InitExit(StateMachine_t *sm) {}
static void AlignGNCExit(StateMachine_t *sm) {
    // initialize GNSS reference point
    for (int i = 0; i <= 2; i++) {
        WGS84_ref[i] = WGS84[i];
      }
    WGS84toECEF(WGS84_ref, ECEF_ref);
}
static void CheckoutsExit(StateMachine_t *sm) {}
static void ArmedExit(StateMachine_t *sm) {}
static void BurnExit(StateMachine_t *sm) {}
static void CoastExit(StateMachine_t *sm) {}
static void UnbrakedDescendExit(StateMachine_t *sm) {}
static void DrogueDescendExit(StateMachine_t *sm) {}
static void MainDescendExit(StateMachine_t *sm) {}
static void LandedExit(StateMachine_t *sm) {}

/* --- Action handler functions --- */
static void StateEntryHandler(StateMachine_t *sm, flight_state_t state) {
    // set entry timestamp
    sm->timestamp_us = uwTick;

    // handle entry for each state
    switch (state) {
        case STATE_FLIGHT_ABORT:            AbortEntry(sm); break;
        case STATE_FLIGHT_STARTUP:          StartupEntry(sm); break;
        case STATE_FLIGHT_INIT:             InitEntry(sm); break;
        case STATE_FLIGHT_GNC_ALIGN:        AlignGNCEntry(sm); break;
        case STATE_FLIGHT_CHECKOUTS:        CheckoutsEntry(sm); break;
        case STATE_FLIGHT_ARMED:            ArmedEntry(sm); break;
        case STATE_FLIGHT_BURN:             BurnEntry(sm); break;
        case STATE_FLIGHT_COAST:            CoastEntry(sm); break;
        case STATE_FLIGHT_DESCEND_UNBRAKED: UnbrakedDescendEntry(sm); break;
        case STATE_FLIGHT_DESCEND_DROGUE:   DrogueDescendEntry(sm); break;
        case STATE_FLIGHT_DESCEND_MAIN:     MainDescendEntry(sm); break;
        case STATE_FLIGHT_LANDED:           LandedEntry(sm); break;
        default: break;
    }
}

static void StateExitHandler(StateMachine_t *sm, flight_state_t state) {
    switch (state) {
        case STATE_FLIGHT_ABORT:            AbortExit(sm); break;
        case STATE_FLIGHT_STARTUP:          StartupExit(sm); break;
        case STATE_FLIGHT_INIT:             InitExit(sm); break;
        case STATE_FLIGHT_GNC_ALIGN:        AlignGNCExit(sm); break;
        case STATE_FLIGHT_CHECKOUTS:        CheckoutsExit(sm); break;
        case STATE_FLIGHT_ARMED:            ArmedExit(sm); break;
        case STATE_FLIGHT_BURN:             BurnExit(sm); break;
        case STATE_FLIGHT_COAST:            CoastExit(sm); break;
        case STATE_FLIGHT_DESCEND_UNBRAKED: UnbrakedDescendExit(sm); break;
        case STATE_FLIGHT_DESCEND_DROGUE:   DrogueDescendExit(sm); break;
        case STATE_FLIGHT_DESCEND_MAIN:     MainDescendExit(sm); break;
        case STATE_FLIGHT_LANDED:           LandedExit(sm); break;
        default: break;
    }
}

static void StateDoHandler(StateMachine_t *sm, flight_state_t state) {
    switch (state) {
        case STATE_FLIGHT_ABORT:            AbortDo(sm); break;
        case STATE_FLIGHT_STARTUP:          StartupDo(sm); break;
        case STATE_FLIGHT_INIT:             InitDo(sm); break;
        case STATE_FLIGHT_GNC_ALIGN:        AlignGNCDo(sm); break;
        case STATE_FLIGHT_CHECKOUTS:        CheckoutsDo(sm); break;
        case STATE_FLIGHT_ARMED:            ArmedDo(sm); break;
        case STATE_FLIGHT_BURN:             BurnDo(sm); break;
        case STATE_FLIGHT_COAST:            CoastDo(sm); break;
        case STATE_FLIGHT_DESCEND_UNBRAKED: UnbrakedDescendDo(sm); break;
        case STATE_FLIGHT_DESCEND_DROGUE:   DrogueDescendDo(sm); break;
        case STATE_FLIGHT_DESCEND_MAIN:     MainDescendDo(sm); break;
        case STATE_FLIGHT_LANDED:           LandedDo(sm); break;
        default: break;
    }
}

/* --- User functions --- */
void StateMachine_Init(StateMachine_t *sm, flight_state_t initialState) {
    sm->currentFlightState = initialState;

    // call entry action for initial state
    StateEntryHandler(sm, sm->currentFlightState);
}

void StateMachine_Dispatch(StateMachine_t *sm, flight_event_t event) {
    if (sm->currentFlightState >= STATE_FLIGHT_MAX) return;

    // TODO: store event on Flash & SD
    
    // store old flight state
    flight_state_t oldState = sm->currentFlightState;

    // handle event and retrieve new flight state
    flight_state_t newState = stateTable[oldState](event);

    // prevent exit and entry actions if no state change
    if (newState == oldState || newState >= STATE_FLIGHT_MAX) return;

    // don't update state if minimum entry time delay hasn't elapsed yet
    if (stateMinEntryDelayTable[newState] > (uwTick - sm->timestamp_us)) return;
    
    // exit old state
    StateExitHandler(sm, oldState);

    // update flight state
    sm->currentFlightState = newState;

    // enter new state
    StateEntryHandler(sm, newState);
}

void StateMachine_DoActions(StateMachine_t *sm) {
    // perform standard state actions
    StateDoHandler(sm, sm->currentFlightState);
}

/* --- Event handlers --- */
static flight_state_t AbortHandler(flight_event_t event) {
    switch (event) {
        default: return STATE_FLIGHT_ABORT;
    }
}

static flight_state_t StartupHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_STARTUP_COMPLETE:     return STATE_FLIGHT_INIT;
        default: return STATE_FLIGHT_STARTUP;
    }
}

static flight_state_t InitHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_GNSS_FIX:             return STATE_FLIGHT_GNC_ALIGN;
        default: return STATE_FLIGHT_INIT;
    }
}

static flight_state_t AlignGNCHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_FILTER_CONVERGED:     return STATE_FLIGHT_CHECKOUTS;
        default: return STATE_FLIGHT_GNC_ALIGN;
    }
}

static flight_state_t CheckoutsHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_CHECKOUTS_COMPLETE:   return STATE_FLIGHT_ARMED;
        default: return STATE_FLIGHT_CHECKOUTS;
    }
}

static flight_state_t ArmedHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_LAUNCH_DETECTED:      return STATE_FLIGHT_BURN;
        default: return STATE_FLIGHT_ARMED;
    }
}

static flight_state_t BurnHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_BURNOUT_DETECTED:     return STATE_FLIGHT_COAST;
        default: return STATE_FLIGHT_BURN;
    }
}

static flight_state_t CoastHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_APOGEE_DETECTED:      return STATE_FLIGHT_DESCEND_UNBRAKED;
        default: return STATE_FLIGHT_COAST;
    }
}

static flight_state_t UnbrakedDescendHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_DROGUE_DEPLOY:        return STATE_FLIGHT_DESCEND_DROGUE;
        default: return STATE_FLIGHT_DESCEND_UNBRAKED;
    }
}

static flight_state_t DrogueDescendHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_MAIN_DEPLOY:          return STATE_FLIGHT_DESCEND_MAIN;
        default: return STATE_FLIGHT_DESCEND_DROGUE;
    }
}

static flight_state_t MainDescendHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_TOUCHDOWN:            return STATE_FLIGHT_LANDED;
        default:                                return STATE_FLIGHT_DESCEND_MAIN;
    }
}

static flight_state_t LandedHandler(flight_event_t event) {
    switch (event) {
        default: return STATE_FLIGHT_LANDED;
    }
}