#include "statemachine.h"
#include "SERVO.h"
#include "comSchedule.h"
#include "navigation.h"
#include "packets.h"

StateMachine_t flight_sm;

uint8_t selftest_tries = 0;

// time index for event dispatch logic
uint32_t time_index = 0;

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
        case EVENT_TEST_MODE_ENTER:             return STATE_TEST_INIT;
        default: return STATE_FLIGHT_INIT;
    }
}

static flight_state_t AlignGNCHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_ABORT:                return STATE_FLIGHT_ABORT;
        case EVENT_FLIGHT_FILTER_CONVERGED:     return STATE_FLIGHT_CHECKOUTS;
        case EVENT_TEST_MODE_ENTER:             return STATE_TEST_INIT;
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
        case EVENT_FLIGHT_BURNOUT_DETECTED:     return STATE_FLIGHT_COAST;
        default: return STATE_FLIGHT_BURN;
    }
}

static flight_state_t CoastHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_DROGUE_COMMANDED:     return STATE_FLIGHT_AWAIT_DROGUE;
        default: return STATE_FLIGHT_COAST;
    }
}

static flight_state_t AwaitDrogueHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_DROGUE_CONFIRMED:     return STATE_FLIGHT_DESCEND_DROGUE;
        case EVENT_FLIGHT_TOUCHDOWN:            return STATE_FLIGHT_LANDED;
        default: return STATE_FLIGHT_AWAIT_DROGUE;
    }
}

static flight_state_t DrogueDescendHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_MAIN_COMMANDED:       return STATE_FLIGHT_AWAIT_MAIN;
        default: return STATE_FLIGHT_DESCEND_DROGUE;
    }
}

static flight_state_t AwaitMainHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_TOUCHDOWN:            return STATE_FLIGHT_LANDED;
        case EVENT_FLIGHT_MAIN_CONFIRMED:       return STATE_FLIGHT_DESCEND_MAIN;
        default:                                return STATE_FLIGHT_AWAIT_MAIN;
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
        case EVENT_FLIGHT_LANDED_SAFE:          return STATE_FLIGHT_LANDED_SAFE;
        default:                                return STATE_FLIGHT_LANDED;
    }
}

static flight_state_t LandedSafeHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_LANDED_POWER_DOWN:    return STATE_FLIGHT_LANDED_POWER_DOWN;
        default:                                return STATE_FLIGHT_LANDED_SAFE;
    }
}

static flight_state_t LandedPowerDownHandler(flight_event_t event) {
    switch (event) {
        default:                                return STATE_FLIGHT_LANDED_POWER_DOWN;
    }
}

static flight_state_t TestInitHandler(flight_event_t event) {
    switch (event) {
        case EVENT_TEST_CALIBRATE:              return STATE_TEST_CALIB;
        case EVENT_TEST_MODE_EXIT:              return STATE_FLIGHT_INIT;
        default: return STATE_TEST_INIT;
    }
}

static flight_state_t TestCalibHandler(flight_event_t event) {
    switch (event) {
        case EVENT_TEST_MODE_EXIT:              return STATE_FLIGHT_INIT;
        default: return STATE_TEST_CALIB;
    }
}

static flight_state_t GroundstationHandler(flight_event_t event) {
    switch (event) {
        default: return STATE_GROUNDSTATION;
    }
}

/* --- Entry actions --- */
static void AbortEntry(StateMachine_t *sm) {
    // disable PU outputs
    PU_setACS(DISABLE);
    PU_setREC(DISABLE);
    PU_setCAM(DISABLE);

    // play beep code
    Buzzer_PlaySong(BUZZER_PATTERN_SONG_FIX_YOU);
    // Buzzer_PlayBeepCode(BUZZER_PATTERN_CODE_PRIMARY_ABORT);
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

    // initialize Servo PWM timers to neutral position
    SERVO_Init(SERVO_Data, &drogue_servo, DROGUE_SERVO, &htim13, 1500, 2500, 135);
    SERVO_Init(SERVO_Data, &main_servo, MAIN_SERVO, &htim16, 1500, 2500, 135);
    SERVO_MoveToAngle(&drogue_servo, DROGUE_NEUTRAL_ANGLE);
    SERVO_MoveToAngle(&main_servo, MAIN_NEUTRAL_ANGLE);

    /* --- Initialize (Extended) Kalman Filters --- */
    // define Kalman Filter dimensions and pointers for velocity and position
    EKFInit(&EKF2, EKF2_type, x_size2, u_size2, 0.001, &F2, &P2, &Q2, NULL, x2, &a_WorldFrame_i[2]);
    EKFCorrectionInit(EKF2, &EKF2_corr1, corr1_type, 1, &H2_corr1, &K2_corr1, 
        &R2_corr1, &S2_corr1, &S2_inv_corr1, z2_corr1, h2_corr1, v2_corr1, EKF_NIS_THRESH_BARO); // baro
    EKFCorrectionInit(EKF2, &EKF2_corr2, corr2_type, 2, &H2_corr2, &K2_corr2, 
        &R2_corr2, &S2_corr2, &S2_inv_corr2, z2_corr2, h2_corr2, v2_corr2, EKF_NIS_THRESH_GNSS); // GNSS

    // define Kalman Filter dimensions and pointers for Quaternion EKF
    EKFInit(&EKF3, EKF3_type, x_size3, u_size3, 0.001, &F3, &P3, &Q3, NULL, x3, average_imu_data.gyro);
    EKFCorrectionInit(EKF3, &EKF3_corr1, corr1_type, 3, &H3_corr1, 
        &K3_corr1, &R3_corr1, &S3_corr1, &S3_inv_corr1, z3_corr1, h3_corr1, v3_corr1, EKF_NIS_THRESH_MAG); // mag
    EKFCorrectionInit(EKF3, &EKF3_corr2, corr2_type, 3, &H3_corr2, 
        &K3_corr2, &R3_corr2, &S3_corr2, &S3_inv_corr2, z3_corr2, h3_corr2, v3_corr2, EKF_NIS_THRESH_ACC); // accel
    
    // initialize MPC algorithm
    MPCInit(&a_mpc, PREDICTION_HORIZON, NUM_INEQUALITY_CONSTRAINTS, MPC_DELTA_T, u_star, x_star);
}
static void InitEntry(StateMachine_t *sm) {
    /* --- Read sensor data for EKF initialization --- */
    #ifndef HIL_TESTING
    imu1_status.hal_status |= IMU_Update(&imu1_data);
    imu2_status.hal_status |= IMU_Update(&imu2_data);
    imu1_status.active = imu1_data.active;
    imu2_status.active = imu2_data.active;
    
    IMU_Average(&imu1_data, &imu2_data, &average_imu_data);

    mag_status.hal_status |= MAG_ReadSensorData(&mag_data);
    arm_vec3_sub_f32(mag_data.field, mag_data.calibration.offset, mag_data.field);
    arm_vec3_element_product_f32(mag_data.field, mag_data.calibration.scale, mag_data.field);

    BMP_readData(&bmp_data.pressure, &bmp_data.height, &bmp_data.temperature);
    #else
    HILgetIMUData(&average_imu_data);
    HILgetMagnetometerData(&mag_data);
    HILgetBarometerData(&bmp_data);
    #endif
    
    // initialize height EKF state vector
    EKF2.x[0] = bmp_data.height;
    EKF2.x[1] = 0.0f;
    EKF2.x[2] = 101325.0f;

    // Quaternion EKF initialization
    float *q = x3;

    // calculate initial quaternion from accelerometer and magnetometer readings
    RotationMatrixOrientationFix(average_imu_data.accel, mag_data.field, &M_rot_bi, DCM_bi_WorldToBody);
    QuaternionFromRotationMatrix(&M_rot_bi, q); // does this function want body to world????
    arm_quaternion_conjugate_f32(q);         // quick fix
    arm_mat_trans_f32(&M_rot_bi, &M_rot_ib);

    // Activate height EKF prediction step and barometer correction step
    EKF2.prediction_state = active;
    EKF2_corr1.correction_state = active;   // baro
    EKF2_corr2.correction_state = inactive; // GNSS

    // Activate quaternion EKF prediction and correction step
    EKF3.prediction_state = active;
    EKF3_corr1.correction_state = active;   // mag
    EKF3_corr2.correction_state = active;   // accel

    SetComSchedule(1); // Main comm schedule
    // calculate stepper position for closed ACS position
    StepperPositionFromACSAngle(0.f, &stepper_zero_position);
    stepper_zero_position += stepper_neutral_angle / 360.f * ROD_SLOPE;
    SPARK_ExitMode();
}
static void AlignGNCEntry(StateMachine_t *sm) {
    // activate GNSS correction step
    EKF2_corr2.correction_state = active;

    // disable NIS thresholds

    PU_setCAM(ENABLE);
    // TODO:
}
static void CheckoutsEntry(StateMachine_t *sm) {
    PU_setACS(ENABLE);
    Camera_Power(ENABLE);
    #ifndef HIL_TESTING
    //HAL_Delay(5);
    #endif
    // TODO: Re-enable
    // Storage_FlashSave(true);
    SetSaveSchedule(1); // start saving pre-flight data to flash
    // TODO:
    // notify ground station that it now needs to command the checkouts
}
static void ArmedEntry(StateMachine_t *sm) {
    PU_setREC(ENABLE);
    // The servo position can be off from the actual position if the arm is rotated manually.
    // This is fixed by moving the servo to a different position and returning to the neutral angle.
    SERVO_MoveSequence(&main_servo, MAIN_NEUTRAL_ANGLE - 5.0f, MAIN_NEUTRAL_ANGLE, 1000, 1);
    SERVO_MoveSequence(&drogue_servo, DROGUE_NEUTRAL_ANGLE - 15.0f, DROGUE_NEUTRAL_ANGLE, 1000, 1);
    SPARK_ZeroStepper();
    Camera_Recording(1);
    SetSaveSchedule(2);
    // TODO:
    // lock ACS
    // start data logging
}
static void BurnEntry(StateMachine_t *sm) {
    EKF3_corr1.correction_state = inactive; // mag & accel
    SPARK_TargetPositionMode(16);
}
static void CoastEntry(StateMachine_t *sm) {
    // initial xstar for MPC optimizer
    float vel_vec_input[3] = { 0 };
    vel_vec_input[0] = gps_data.velE*(1e-3);
    vel_vec_input[1] = gps_data.velN*(1e-3);
    vel_vec_input[2] = EKF2.x[1];
    predictFutureStateGamma(EKF2.x[0], vel_vec_input, AREF, DRYMASS, 0, a_mpc.dt*a_mpc.N, 0.1, &a_mpc.xstar[0], &a_mpc.xstar[1]);

}
static void AwaitDrogueEntry(StateMachine_t *sm) {
    // deploy drogue parachute
    SERVO_MoveSequence(&drogue_servo, DROGUE_DEPLOY_ANGLE, DROGUE_NEUTRAL_ANGLE, DROGUE_MOVE_DELAY_MS, 0);
    drogue_deploy_attempts++;
#ifdef HIL_TESTING
    tim14_target_ms = HIL_DROGUE_DEPLOY_TIME;
    HAL_TIM_Base_Start_IT(&htim14);
#endif

}
static void DrogueDescendEntry(StateMachine_t *sm) {
    // play beep code
    Buzzer_PlaySong(BUZZER_PATTERN_SONG_COW);
}
static void AwaitMainEntry(StateMachine_t *sm) {
    // deplay main parachute
    SERVO_MoveSequence(&main_servo, MAIN_DEPLOY_ANGLE, MAIN_NEUTRAL_ANGLE, MAIN_MOVE_DELAY_MS, 0);
    main_deploy_attempts++;
#ifdef HIL_TESTING
    tim14_target_ms = HIL_MAIN_DEPLOY_TIME;
    HAL_TIM_Base_Start_IT(&htim14);
#endif
}
static void MainDescendEntry(StateMachine_t *sm) {}
static void LandedEntry(StateMachine_t *sm) {
    PU_setACS(DISABLE);
    PU_setREC(DISABLE);
    Camera_Recording(DISABLE);

    // TODO:
    // stop data logging
}
static void LandedSafeEntry(StateMachine_t *sm) {
    Camera_Power(DISABLE);
    SetComSchedule(5); // Landed flight state, only GNSS and state
    SetSaveSchedule(0);
    Storage_FlashSave(DISABLE);

    // play beep code
    Buzzer_PlaySong(BUZZER_PATTERN_SONG_ASE_SONG);
}
static void LandedPowerDownEntry(StateMachine_t *sm) {
    PU_setCAM(DISABLE);

    // play beep code repeatidly
    Buzzer_PlayBeepCode(BUZZER_PATTERN_CODE_PRIMARY_RECOVERY);
}
static void TestInitEntry(StateMachine_t *sm) {
    SERVO_MoveSequence(&main_servo, MAIN_DEPLOY_ANGLE, MAIN_NEUTRAL_ANGLE, 9000, 1);
}
static void TestCalibEntry(StateMachine_t *sm) {}
static void GroundstationEntry(StateMachine_t *sm) {}

/* --- Do actions --- */
static void AbortDo(StateMachine_t *sm, uint16_t freq) {}
static void StartupDo(StateMachine_t *sm, uint16_t freq) {
    // Selftest is called with 10 Hz
    if (freq != 10) return;

    // All sensors are working but selftest_pass flag not set
    if ((status_data.sensor_status_flags & 0x1F) == 0x1F) {
        imu1_data.passed_self_test = true;
        imu2_data.passed_self_test = true;
        status_data.sensor_status_flags |= (1<<7);  //All checks passed
        StateMachine_Dispatch(sm, EVENT_FLIGHT_STARTUP_COMPLETE);
    }
    // All sensors but IMU1 working
    else if (((selftest_tries > MAX_SELFTEST_TRIES) && (status_data.sensor_status_flags & 0x1E) == 0x1E)) {
        // TODO: Notify user of defect IMU1 and disable IMU1 permanently
        imu1_data.passed_self_test = false;
        imu2_data.passed_self_test = true;
        status_data.sensor_status_flags |= (1<<7);  //All checks passed
        StateMachine_Dispatch(sm, EVENT_FLIGHT_STARTUP_COMPLETE);
    }
    // All sensors but IMU2 working
    else if (((selftest_tries > MAX_SELFTEST_TRIES) && (status_data.sensor_status_flags & 0x1D) == 0x1D)) {
        // TODO: Notify user of defect IMU2 and disable IMU2 permanently
        imu1_data.passed_self_test = true;
        imu2_data.passed_self_test = false;
        status_data.sensor_status_flags |= (1<<7);  //All checks passed
        StateMachine_Dispatch(sm, EVENT_FLIGHT_STARTUP_COMPLETE);
    }
    // Run selftest if not all sensors are working or selftest not passed
    else if ((status_data.sensor_status_flags & 0xFF) != 0x9F) {
        selftest_tries += 1;

        status_data.sensor_status_flags |= IMU_SelfTest(&imu1_data);
        status_data.sensor_status_flags |= (IMU_SelfTest(&imu2_data) << 1);
        status_data.sensor_status_flags |= (MAG_SelfTest() << 2);
        status_data.sensor_status_flags |= (BMP_SelfTest() << 3);

        // Excluding GPS check for now, as the first message cant be relaibly received by this function
        //status_data.sensor_status_flags |= (GPS_VER_CHECK()<<4); //Check if GPS is connected and working

        // GPA Mega won't pass self test if no PtotCB is connected
        //status_data.sensor_status_flags |= (ptot_SelfTest(&ptot_data) << 5);

        // TODO: XBEE, PU, Comms, SPARK SelfTest
        status_data.sensor_status_flags |= (1 << 4);
    }
}
static void InitDo(StateMachine_t *sm, uint16_t freq) {
    // Check GPS with 10 Hz
    if (freq != 10) return;

    if ((gps_data.gpsFix == 3)) {
        StateMachine_Dispatch(sm, EVENT_FLIGHT_GNSS_FIX);
    }
}
static void AlignGNCDo(StateMachine_t *sm, uint16_t freq) {
    if (freq != 10) return;
    if (EKFisAligned(&EKF2) && EKFisAligned(&EKF3)) {
        StateMachine_Dispatch(sm, EVENT_FLIGHT_FILTER_CONVERGED);
    }
}
static void CheckoutsDo(StateMachine_t *sm, uint16_t freq) {
    // checkouts complete is confirmed by ground station
}
static void ArmedDo(StateMachine_t *sm, uint16_t freq) {
    // Check for liftoff with 1000 Hz
    if (freq != 1000) return;

    if (a_BodyFrame_i[1] < LIFTOFF_ACC_THRESHOLD) {
        time_index = HAL_GetTick();
    } else if ((HAL_GetTick() - time_index) > LIFTOFF_MIN_DURATION) {
        StateMachine_Dispatch(sm, EVENT_FLIGHT_LAUNCH_DETECTED);
    }
}
static void BurnDo(StateMachine_t *sm, uint16_t freq) {
    // Check for burnout with 1000 Hz
    if (freq != 1000) return;

    if (a_BodyFrame_i[1] > BURNOUT_ACC_THRESHOLD) {
        time_index = HAL_GetTick();
    } else if ((HAL_GetTick() - time_index) > BURNOUT_MIN_DURATION) {
        StateMachine_Dispatch(sm, EVENT_FLIGHT_BURNOUT_DETECTED);
    }
}
static void CoastDo(StateMachine_t *sm, uint16_t freq) {
    // check for apogee with 1000 Hz
    if (freq == 1000) {
        if (EKF2.x[1] > APOGEE_VEL_THRESHOLD) {
            time_index = HAL_GetTick();
        } else if ((HAL_GetTick() - time_index) > APOGEE_MIN_DURATION) {
            StateMachine_Dispatch(sm, EVENT_FLIGHT_DROGUE_COMMANDED);
        }
    }

    if (freq != 10) return;

    // MPC
    TimeMeasureStart();

    // define input velocity vector
    float vel_vec_input[3] = { 0 };
    vel_vec_input[0] = gps_data.velE*(1e-3);
    vel_vec_input[1] = gps_data.velN*(1e-3);
    vel_vec_input[2] = EKF2.x[1];

    // solve optimization problem
    acs_target_angle_deg = runMPC(&a_mpc, EKF2.x[0], vel_vec_input);
    dt_mpc = TimeMeasureStop();

    // send airbrake deflection angle command to SPARK
    StepperPositionFromACSAngle(acs_target_angle_deg, &stepper_target_position);
    StepperAngleFromPosition(stepper_target_position, stepper_zero_position, &stepper_target_angle_deg);
    SPARK_SetAngle(stepper_target_angle_deg);
}
static void AwaitDrogueDo(StateMachine_t *sm, uint16_t freq) {
    // check for drogue with 1000 Hz
    if (freq == 1000) {
        // option 2
        if ((EKF2.x[1] > -DROGUE_MIN_VELOCITY) || (EKF2.x[1] < -DROGUE_MAX_VELOCITY) || (a_abs_g < DROGUE_MIN_ACCELERATION)) {
            time_index = HAL_GetTick();
        } else if ((HAL_GetTick() - time_index) > DROGUE_MIN_DURATION) {
            StateMachine_Dispatch(sm, EVENT_FLIGHT_DROGUE_CONFIRMED);
        }
    }

    // drogue deploy handler
    if (freq == 100) {
        uint32_t elapsed_time = HAL_GetTick() - sm->timestamp_ms;

        // attempt further deployment attempts
        if ((elapsed_time >= drogue_deploy_attempts * DROGUE_ATTEMPT_PERIOD) && (drogue_deploy_attempts < DROGUE_MAX_NUM_ATTEMPTS)) {
            SERVO_MoveSequence(&drogue_servo, DROGUE_DEPLOY_ANGLE, DROGUE_NEUTRAL_ANGLE, DROGUE_MOVE_DELAY_MS, 0);
            drogue_deploy_attempts++;
        }
    }

    // fully deploy airbrakes
    if (freq != 10) return;
    acs_target_angle_deg = ACS_ANGLE_MAX_DEG;
    StepperPositionFromACSAngle(acs_target_angle_deg, &stepper_target_position);
    StepperAngleFromPosition(stepper_target_position, stepper_zero_position, &stepper_target_angle_deg);
    SPARK_SetAngle(stepper_target_angle_deg);
}
static void DrogueDescendDo(StateMachine_t *sm, uint16_t freq) {
    // check for main deployment height with 1000 Hz
    if (freq == 1000) {
        // option 2
        if (EKF2.x[0] > MAIN_MIN_DEPLOY_HEIGHT) {
            time_index = HAL_GetTick();
        } else if ((HAL_GetTick() - time_index) > MAIN_MIN_DURATION) {
            StateMachine_Dispatch(sm, EVENT_FLIGHT_MAIN_COMMANDED);
        }
    }

    // fully retract airbrakes
    if (freq != 10) return;
    acs_target_angle_deg = ACS_ANGLE_MIN_DEG;
    StepperPositionFromACSAngle(acs_target_angle_deg, &stepper_target_position);
    StepperAngleFromPosition(stepper_target_position, stepper_zero_position, &stepper_target_angle_deg);
    SPARK_SetAngle(stepper_target_angle_deg);
}
static void AwaitMainDo(StateMachine_t *sm, uint16_t freq) {
    if (freq == 100) {
        uint32_t elapsed_time = HAL_GetTick() - sm->timestamp_ms;

        // attempt further deployment attempts
        if ((elapsed_time >= main_deploy_attempts * MAIN_ATTEMPT_PERIOD) && (main_deploy_attempts < MAIN_MAX_NUM_ATTEMPTS)) {
            SERVO_MoveSequence(&main_servo, MAIN_DEPLOY_ANGLE, MAIN_NEUTRAL_ANGLE, MAIN_MOVE_DELAY_MS, 0);
            main_deploy_attempts++;
        }
    }

    // check for main with 1000 Hz
    if (freq == 1000) {
        // option 2
        if ((EKF2.x[1] > -MAIN_MIN_VELOCITY) || (EKF2.x[1] < -MAIN_MAX_VELOCITY) || (a_abs_g < MAIN_MIN_ACCELERATION)) {
            time_index = HAL_GetTick();
        } else if ((HAL_GetTick() - time_index) > MAIN_MIN_DURATION) {
            StateMachine_Dispatch(sm, EVENT_FLIGHT_MAIN_CONFIRMED);
        }
    }
}
static void MainDescendDo(StateMachine_t *sm, uint16_t freq) {
    // check for landing with 1000 Hz
    if (freq == 1000) {
        // option 2
        if ((EKF2.x[1] < -LANDED_MAX_VELOCITY) || (a_abs_g < LANDED_MIN_ACCELERATION) || (a_abs_g) > LANDED_MAX_ACCELERATION) {
            time_index = HAL_GetTick();
        } else if ((HAL_GetTick() - time_index) > LANDED_MIN_DURATION) {
            StateMachine_Dispatch(sm, EVENT_FLIGHT_TOUCHDOWN);
        }
    }
}
static void LandedDo(StateMachine_t *sm, uint16_t freq) {}
static void LandedSafeDo(StateMachine_t *sm, uint16_t freq) {
    // check battery voltage with 10 Hz
    if (freq != 10) return;

    if (powerData.Data.power.PU_bat_bus_volt < 7400) // 7.4 V
        StateMachine_Dispatch(sm, EVENT_FLIGHT_LANDED_POWER_DOWN);
}
static void LandedPowerDownDo(StateMachine_t *sm, uint16_t freq) {}
static void TestInitDo(StateMachine_t *sm, uint16_t freq) {}
static void TestCalibDo(StateMachine_t *sm, uint16_t freq) {}
static void GroundstationDo(StateMachine_t *sm, uint16_t freq) {}

/* --- Exit actions --- */
static void AbortExit(StateMachine_t *sm) {}
static void StartupExit(StateMachine_t *sm) {}
static void InitExit(StateMachine_t *sm) {}
static void AlignGNCExit(StateMachine_t *sm) {
    // set reference height
    h_ref = EKF2.x[0];
    EKF2.x[0] = 0.0f;
}
static void CheckoutsExit(StateMachine_t *sm) {}
static void ArmedExit(StateMachine_t *sm) {}
static void BurnExit(StateMachine_t *sm) {}
static void CoastExit(StateMachine_t *sm) {}
static void AwaitDrogueExit(StateMachine_t *sm) {}
static void DrogueDescendExit(StateMachine_t *sm) {}
static void AwaitMainExit(StateMachine_t *sm) {}
static void MainDescendExit(StateMachine_t *sm) {}
static void LandedExit(StateMachine_t *sm) {}
static void LandedSafeExit(StateMachine_t *sm) {}
static void LandedPowerDownExit(StateMachine_t *sm) {}
static void TestInitExit(StateMachine_t *sm) {}
static void TestCalibExit(StateMachine_t *sm) {}
static void GroundstationExit(StateMachine_t *sm) {}

/* --- Lookup tables for state functions --- */
static StateHandler_t stateHandlerTable[STATE_MAX] = {
    AbortHandler,
    StartupHandler,
    InitHandler,
    AlignGNCHandler,
    CheckoutsHandler,
    ArmedHandler,
    BurnHandler,
    CoastHandler,
    AwaitDrogueHandler,
    DrogueDescendHandler,
    AwaitMainHandler,
    MainDescendHandler,
    LandedHandler,
    LandedSafeHandler,
    LandedPowerDownHandler,
    TestInitHandler,
    TestCalibHandler,
    GroundstationHandler
};

static StateEntry_t stateEntryTable[STATE_MAX] = {
    AbortEntry,
    StartupEntry,
    InitEntry,
    AlignGNCEntry,
    CheckoutsEntry,
    ArmedEntry,
    BurnEntry,
    CoastEntry,
    AwaitDrogueEntry,
    DrogueDescendEntry,
    AwaitMainEntry,
    MainDescendEntry,
    LandedEntry,
    LandedSafeEntry,
    LandedPowerDownEntry,
    TestInitEntry,
    TestCalibEntry,
    GroundstationEntry
};

static StateDo_t stateDoTable[STATE_MAX] = {
    AbortDo,
    StartupDo,
    InitDo,
    AlignGNCDo,
    CheckoutsDo,
    ArmedDo,
    BurnDo,
    CoastDo,
    AwaitDrogueDo,
    DrogueDescendDo,
    AwaitMainDo,
    MainDescendDo,
    LandedDo,
    LandedSafeDo,
    LandedPowerDownDo,
    TestInitDo,
    TestCalibDo,
    GroundstationDo
};

static StateExit_t stateExitTable[STATE_MAX] = {
    AbortExit,
    StartupExit,
    InitExit,
    AlignGNCExit,
    CheckoutsExit,
    ArmedExit,
    BurnExit,
    CoastExit,
    AwaitDrogueExit,
    DrogueDescendExit,
    AwaitMainExit,
    MainDescendExit,
    LandedExit,
    LandedSafeExit,
    LandedPowerDownExit,
    TestInitExit,
    TestCalibExit,
    GroundstationExit
};

/* --- Lookup tables for timing constraints --- */
// this table specifies for each event the minimum time in ms between a state transition and the event
uint32_t minEventDelayTable[EVENT_MAX] = {
    // flight events
    [EVENT_FLIGHT_ABORT]              = 0,
    [EVENT_FLIGHT_STARTUP_COMPLETE]   = 0,
    [EVENT_FLIGHT_GNSS_FIX]           = 0,
    [EVENT_FLIGHT_FILTER_CONVERGED]   = MIN_DELAY_UNTIL_FILTER_CONVERGED,
    [EVENT_FLIGHT_CHECKOUTS_COMPLETE] = MIN_DELAY_UNTIL_CHECKOUTS_COMPLETE,
    [EVENT_FLIGHT_LAUNCH_DETECTED]    = MIN_DELAY_UNTIL_LAUNCH_DETECTED,
    [EVENT_FLIGHT_BURNOUT_DETECTED]   = MIN_DELAY_UNTIL_BURNOUT_DETECTED,
    [EVENT_FLIGHT_DROGUE_COMMANDED]   = MIN_DELAY_UNTIL_DROGUE_COMMANDED,
    [EVENT_FLIGHT_DROGUE_CONFIRMED]   = MIN_DELAY_UNTIL_DROGUE_CONFIRMED,
    [EVENT_FLIGHT_MAIN_COMMANDED]     = MIN_DELAY_UNTIL_MAIN_COMMANDED,
    [EVENT_FLIGHT_MAIN_CONFIRMED]     = MIN_DELAY_UNTIL_MAIN_CONFIRMED,
    [EVENT_FLIGHT_TOUCHDOWN]          = MIN_DELAY_UNTIL_TOUCHDOWN,
    [EVENT_FLIGHT_LANDED_SAFE]        = 0,
    [EVENT_FLIGHT_LANDED_POWER_DOWN]  = MIN_DELAY_UNTIL_POWER_DOWN,

    // test events
    [EVENT_TEST_MODE_ENTER] = 0,
    [EVENT_TEST_CALIBRATE]  = 0,
    [EVENT_TEST_MODE_EXIT]  = 0,
};

// this table specifies for each state the maximum time until an event is dispatched
uint32_t maxStateDurationTable[STATE_MAX] = {
    // flight states
    [STATE_FLIGHT_ABORT]   = MAX_DELAY_UNUSED,   // Abort Exit; LEAVE AT ZERO
    [STATE_FLIGHT_STARTUP] = MAX_DELAY_UNUSED,   // Startup complete
    [STATE_FLIGHT_INIT]    = MAX_DELAY_UNUSED,   // GNSS Fix
#ifdef HIL_TESTING
    [STATE_FLIGHT_GNC_ALIGN] = 2000,             // Filter converged
#else
    [STATE_FLIGHT_GNC_ALIGN] = MAX_DELAY_UNUSED, // Filter converged
#endif
    [STATE_FLIGHT_CHECKOUTS]         = MAX_DELAY_UNUSED,   // GCS command
    [STATE_FLIGHT_ARMED]             = MAX_DELAY_UNUSED,   // Liftoff; LEAVE AT ZERO
    [STATE_FLIGHT_BURN]              = MAX_DELAY_UNTIL_BURNOUT_DETECTED,
    [STATE_FLIGHT_COAST]             = MAX_DELAY_UNTIL_DROGUE_COMMANDED,
    [STATE_FLIGHT_AWAIT_DROGUE]      = MAX_DELAY_UNTIL_TOUCHDOWN_BAD,
    [STATE_FLIGHT_DESCEND_DROGUE]    = MAX_DELAY_UNTIL_MAIN_COMMANDED,
    [STATE_FLIGHT_AWAIT_MAIN]        = MAX_DELAY_UNTIL_TOUCHDOWN_DROGUE,
    [STATE_FLIGHT_DESCEND_MAIN]      = MAX_DELAY_UNTIL_TOUCHDOWN_MAIN,
    [STATE_FLIGHT_LANDED]            = MAX_DELAY_UNTIL_LANDED_SAFE,
    [STATE_FLIGHT_LANDED_SAFE]       = MAX_DELAY_UNUSED,
    [STATE_FLIGHT_LANDED_POWER_DOWN] = MAX_DELAY_UNUSED,

    // test states
    [STATE_TEST_INIT] = MAX_DELAY_UNUSED,
    [STATE_TEST_CALIB] = MAX_DELAY_UNUSED,

    // groundstation state
    [STATE_GROUNDSTATION] = MAX_DELAY_UNUSED,
};

// this table specifies for each state which event is forced after the maximum state duration
flight_event_t autoEventTriggerTable[STATE_MAX] = {
    // flight states
    [STATE_FLIGHT_ABORT]             = EVENT_MAX,
    [STATE_FLIGHT_STARTUP]           = EVENT_MAX,
    [STATE_FLIGHT_INIT]              = EVENT_MAX,
    [STATE_FLIGHT_GNC_ALIGN]         = EVENT_FLIGHT_FILTER_CONVERGED,
    [STATE_FLIGHT_CHECKOUTS]         = EVENT_FLIGHT_CHECKOUTS_COMPLETE,
    [STATE_FLIGHT_ARMED]             = EVENT_MAX,
    [STATE_FLIGHT_BURN]              = EVENT_FLIGHT_BURNOUT_DETECTED,
    [STATE_FLIGHT_COAST]             = EVENT_FLIGHT_DROGUE_COMMANDED,
    [STATE_FLIGHT_AWAIT_DROGUE]      = EVENT_FLIGHT_TOUCHDOWN,
    [STATE_FLIGHT_DESCEND_DROGUE]    = EVENT_FLIGHT_MAIN_COMMANDED,
    [STATE_FLIGHT_AWAIT_MAIN]        = EVENT_FLIGHT_TOUCHDOWN,
    [STATE_FLIGHT_DESCEND_MAIN]      = EVENT_FLIGHT_TOUCHDOWN,
    [STATE_FLIGHT_LANDED]            = EVENT_FLIGHT_LANDED_SAFE,
    [STATE_FLIGHT_LANDED_SAFE]       = EVENT_MAX,
    [STATE_FLIGHT_LANDED_POWER_DOWN] = EVENT_MAX,

    // test states
    [STATE_TEST_INIT]  = EVENT_MAX,
    [STATE_TEST_CALIB] = EVENT_MAX,

    // groundstation state
    [STATE_GROUNDSTATION] = EVENT_MAX,
};


/* --- Action handler functions --- */
static void StateEntryHandler(StateMachine_t *sm, flight_state_t state) {
    // set entry timestamp
    sm->timestamp_ms = HAL_GetTick();

    // handle entry for each state
    stateEntryTable[state](sm);
}

static void StateExitHandler(StateMachine_t *sm, flight_state_t state) {
    stateExitTable[state](sm);
}

static void StateDoHandler(StateMachine_t *sm, flight_state_t state, uint16_t freq) {
    stateDoTable[state](sm, freq);
}

/* --- User functions --- */
void StateMachine_Init(StateMachine_t *sm, flight_state_t initialState, TIM_HandleTypeDef *htim) {
    // set the timer HAL pointer
    sm->htim = htim;

    // initial state
    sm->currentFlightState = initialState;

    // call entry action for initial state
    StateEntryHandler(sm, sm->currentFlightState);
}

void StateMachine_Dispatch(StateMachine_t *sm, flight_event_t event) {
    if (sm->currentFlightState >= STATE_MAX)
        return;
    if (event >= EVENT_MAX)
        return;

    // TODO: store event on Flash & SD
    
    // store old flight state
    flight_state_t oldState = sm->currentFlightState;

    // handle event and retrieve new flight state
    flight_state_t newState = stateHandlerTable[oldState](event);

    // prevent exit and entry actions if no state change
    if (newState == oldState || newState >= STATE_MAX)
        return;

    // don't update state if minimum entry time delay for event hasn't elapsed yet
    if (minEventDelayTable[event] > (HAL_GetTick() - sm->timestamp_ms))
        return;
    
    // exit old state
    StateExitHandler(sm, oldState);

    // update flight state
    sm->currentFlightState = newState;

    // stop timer
    HAL_TIM_Base_Stop_IT(sm->htim);

    // start timer to enforce maximum delay until event
    if ((sm->tim_trigger_time_ms = maxStateDurationTable[newState])) {
        sm->tim_counter = 0;
        sm->tim_trigger_event = autoEventTriggerTable[newState];
        HAL_TIM_Base_Start_IT(sm->htim);
    }

    // enter new state
    StateEntryHandler(sm, newState);
    time_index = HAL_GetTick();
}

void StateMachine_ForceState(StateMachine_t *sm, flight_state_t newState) {
    if (sm->currentFlightState >= STATE_MAX) return;

    // TODO: store command on Flash & SD
    
    // store old flight state
    flight_state_t oldState = sm->currentFlightState;

    // prevent exit and entry actions if no state change
    if (newState == oldState || newState >= STATE_MAX)
        return;
    
    // exit old state
    StateExitHandler(sm, oldState);

    // update flight state
    sm->currentFlightState = newState;

    // stop timer
    HAL_TIM_Base_Stop_IT(sm->htim);

    // start timer to enforce maximum delay until event
    if ((sm->tim_trigger_time_ms = maxStateDurationTable[newState])) {
        sm->tim_counter = 0;
        sm->tim_trigger_event = autoEventTriggerTable[newState];
        HAL_TIM_Base_Start_IT(sm->htim);
    }

    // enter new state
    StateEntryHandler(sm, newState);
    time_index = HAL_GetTick();
}

void StateMachine_TimeHandler(StateMachine_t *sm, TIM_HandleTypeDef *htim) {
    // check whether our timer got called
    if (sm->htim->Instance != htim->Instance)
        return;
    
    // increment counter
    sm->tim_counter++;

    // have we reached the target time?
    if (sm->tim_counter < sm->tim_trigger_time_ms)
        return;

    // halt & reset the timer
    HAL_TIM_Base_Stop_IT(sm->htim);
    sm->tim_counter = 0;

    // dispatch the triggered event
    StateMachine_Dispatch(sm, sm->tim_trigger_event);
}

void StateMachine_DoActions(StateMachine_t *sm, uint16_t freq) {
    // perform standard state actions
    StateDoHandler(sm, sm->currentFlightState, freq);
}