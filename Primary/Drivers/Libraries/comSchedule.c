#include "comSchedule.h"
#include "InterBoardCom.h"

uint8_t messages_num = 0;
message_info_t message_schedule[MAX_NUM_MESSAGES];

uint8_t communication_mode = COMM_MODE_REMOTE_TRANSMIT; // Default communication mode(flight mode)

DataPacket_t Status_DataPacket;
DataPacket_t Power_DataPacket;
DataPacket_t GPS_DataPacket;
DataPacket_t IMU_DataPacket;
DataPacket_t Temperature_DataPacket;
DataPacket_t Position_DataPacket;
DataPacket_t Attitude_DataPacket;
DataPacket_t Kalman_DataPacket;
DataPacket_t Spark_DataPacket;
DataPacket_t MPC_Info_DataPacket;
DataPacket_t State_DataPacket;

extern bool is_groundstation; // Flag to indicate if this board is the groundstation

// GPS data
extern UBX_NAV_PVT gps_data;
// IMU data
extern IMU_AverageData_t average_imu_data;
extern LIS3MDL_Data_t mag_data;
// Attitude data
extern float euler[3];
// Kalman data
extern arm_matrix_instance_f32 P2;
extern ekf_data_t EKF2;
// Spark data
extern DataPacket_t spark_data;
// MPC data
extern uint32_t dt_1000Hz;
extern float acs_est_angle_deg;
extern float acs_target_angle_deg;
// State machine data
extern StateMachine_t flight_sm;

void UpdatePacket(DataPacket_t *packet);

void InitializeDataScheduler() {

    if(is_groundstation) { // If this is the groundstation, we want to forward all data received via Radio to the PC via USB, so we use the forwarding mode
        communication_mode = COMM_MODE_FORWARDING;
    } else {
        communication_mode = COMM_MODE_LOCAL;
    }

    //Initialize the packets with their IDs
    Status_DataPacket = CreateDataPacket(PACKET_ID_STATUS);
    Power_DataPacket = CreateDataPacket(PACKET_ID_POWER);
    GPS_DataPacket = CreateDataPacket(PACKET_ID_GPS);
    IMU_DataPacket = CreateDataPacket(PACKET_ID_IMU);
    Temperature_DataPacket = CreateDataPacket(PACKET_ID_TEMPERATURE);
    Position_DataPacket = CreateDataPacket(PACKET_ID_POSITION);
    Attitude_DataPacket = CreateDataPacket(PACKET_ID_ATTITUDE);
    Kalman_DataPacket = CreateDataPacket(PACKET_ID_KALMANMATRIX);
    Spark_DataPacket = CreateDataPacket(PACKET_ID_SPARK);
    MPC_Info_DataPacket = CreateDataPacket(PACKET_ID_MPC_INFO);
    State_DataPacket = CreateDataPacket(PACKET_ID_STATE);

    // Initialize message info for each packet, starting at 1Hz
    message_info_t status_msg_info = {1000, 0, 0, 0, &Status_DataPacket};
    message_info_t power_msg_info = {1000, 0, 0, 0, &Power_DataPacket};
    message_info_t gps_msg_info = {1000, 0, 0, 0, &GPS_DataPacket};
    message_info_t imu_msg_info = {1000, 0, 0, 0, &IMU_DataPacket};
    message_info_t temperature_msg_info = {1000, 0, 0, 0, &Temperature_DataPacket};
    message_info_t position_msg_info = {1000, 0, 0, 0, &Position_DataPacket};
    message_info_t attitude_msg_info = {1000, 0, 0, 0, &Attitude_DataPacket};
    message_info_t kalman_msg_info = {1000, 0, 0, 0, &Kalman_DataPacket};
    message_info_t spark_msg_info = {1000, 0, 0, 0, &Spark_DataPacket};
    message_info_t mpc_info_msg_info = {1000, 0, 0, 0, &MPC_Info_DataPacket};
    message_info_t state_msg_info = {1000, 0, 0, 0, &State_DataPacket};

    // Add message info to the list
    message_schedule[0] = status_msg_info;
    message_schedule[1] = power_msg_info;
    message_schedule[2] = gps_msg_info;
    message_schedule[3] = imu_msg_info;
    message_schedule[4] = temperature_msg_info;
    message_schedule[5] = position_msg_info;
    message_schedule[6] = attitude_msg_info;
    message_schedule[7] = kalman_msg_info;
    message_schedule[8] = spark_msg_info;
    message_schedule[9] = mpc_info_msg_info;
    message_schedule[10] = state_msg_info;

    messages_num = 11; // Update the number of messages in the schedule

    // Initialize last sent timestamps and last saved timestamps to 0
    for (int i = 0; i < messages_num; i++) {
        message_schedule[i].last_sent_tick = 0;
        message_schedule[i].last_saved_tick = 0;
    }
}
//                                  stat, pow, gps, imu, temp, pos, att, kalman, spark, mpc, state
uint32_t comm_schedule1_frequencies[] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100}; // Frequencies for schedule 1 (10Hz for all packets, pre-launch)
uint32_t comm_schedule2_frequencies[] = {100, 1000, 1000, 50, 500, 100, 100, 500, 1000, 1000, 1000}; // Frequencies for schedule 2 (Burn)
uint32_t comm_schedule3_frequencies[] = {100, 200, 1000, 200, 100, 100, 100, 1000, 200, 100, 1000}; //Frequencies for schedule 3 (Coast)
uint32_t comm_schedule4_frequencies[] = {100, 1000, 100, 1000, 1000, 500, 1000, 1000, 1000, 1000, 1000}; // Frequencies for schedule 4 (Descent)
uint32_t comm_schedule5_frequencies[] = {1000, 0, 500, 0, 0, 0, 0, 0, 0, 0, 0}; // Frequencies for schedule 5 (Landed
uint32_t comm_schedule6_frequencies[] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100}; // Frequencies for schedule 6 (HIL Testing)
//Used to set the comunication Schedule to one of the predefined schedules
void SetComSchedule(uint8_t schedule_id) {
    switch(schedule_id) {
        case 0: // Default schedule
            break;
        case 1: // Pre-Launch
            UpdateComSchedule(comm_schedule1_frequencies);
            return;
        case 2: // Burn
            UpdateComSchedule(comm_schedule2_frequencies);
            return;
        case 3: // Coast
            UpdateComSchedule(comm_schedule3_frequencies);
            return;
        case 4: // Descent
            UpdateComSchedule(comm_schedule4_frequencies);
            return;
        case 5: // Landed
            UpdateComSchedule(comm_schedule5_frequencies);
            return;
        case 6: // HIL Testing
            UpdateComSchedule(comm_schedule6_frequencies);
            return;
        default:
            break;
    }
}

// This function changes the frequencys of the messages by passing in an array of frequencys, the order of the frequencys should be the same as the order of the messages in the message_schedule array
void UpdateComSchedule(uint32_t* new_frequencies) {
    for (int i = 0; i < messages_num; i++) {
        message_schedule[i].send_period = new_frequencies[i];
    }
}

void UpdateSendSchedule(uint32_t* new_frequencies) {
    for (int i = 0; i < messages_num; i++) {
        message_schedule[i].save_period = new_frequencies[i];
    }
}

// This function should be called in the 100Hz loop, it checks if any messages are due to be sent and sends them if necessary
void ProcessDataSchedule(uint32_t current_tick) {
    for (int i = 0; i < messages_num; i++) {
        uint8_t is_send, is_save = 0;
        if ((current_tick - message_schedule[i].last_sent_tick >= message_schedule[i].send_period) && (message_schedule[i].send_period != 0)) {
            is_send = 1;
        }
        if ((current_tick - message_schedule[i].last_saved_tick >= message_schedule[i].save_period) && (message_schedule[i].save_period != 0)) {
            is_save = 1;
        }

        if (is_send || is_save){
            UpdatePacket(message_schedule[i].packet);
        }
        
        if (is_send && !is_save) {
            // After updating the packet, send it via the appropriate communication interface
            if (communication_mode == COMM_MODE_FORWARDING || communication_mode == COMM_MODE_LOCAL) {
                USB_QueueDataPacket(message_schedule[i].packet); // Forward the packet to the PC via USB
            } else if (communication_mode == COMM_MODE_REMOTE_TRANSMIT) {
                InterBoardCom_SendDataPacket(INTERBOARD_OP_SAVE_SEND | INTERBOARD_TARGET_RADIO, message_schedule[i].packet); // Send the packet to the groundstation via Radio
            }
            message_schedule[i].last_sent_tick = current_tick;
        } else if (!is_send && is_save) {
            // After updating the packet, save it to flash or SD card
            InterBoardCom_SendDataPacket(INTERBOARD_OP_SAVE_SEND | INTERBOARD_TARGET_FLASH, message_schedule[i].packet);
            message_schedule[i].last_saved_tick = current_tick;
        } else if (is_send && is_save) {
            // If the packet is due for both sending and saving, we can combine the operations to save time and resources
            InterBoardCom_SendDataPacket(INTERBOARD_OP_SAVE_SEND | INTERBOARD_TARGET_RADIO | INTERBOARD_TARGET_FLASH, message_schedule[i].packet); // Send the packet to the groundstation via Radio and indicate that it should also be saved to flash or SD card
            message_schedule[i].last_sent_tick = current_tick;
            message_schedule[i].last_saved_tick = current_tick;
        }
    }   
}

// This function Updates the packet data based on its type and then send it via the appropriate communication interface (e.g., USB, Radio)
void UpdatePacket(DataPacket_t *packet) {
    switch(packet->Packet_ID) {
        case PACKET_ID_STATUS:
            UpdateStatusPacket(packet, HAL_GetTick(), /*status_flags*/0, /* sensor_flags */ 0, /* error_flags */ 0, /* flight_state */ 0);
            break;
        case PACKET_ID_POWER:
            UpdatePowerPacket(packet, HAL_GetTick(), /* PU_bat_volt */ 0, /* PU_out_pow */ 0, /* PU_out_curr */ 0, /* M2_bus_5V */ 0, /* M2_bus_GPA_bat_volt */ 0);
            break;
        case PACKET_ID_GPS:
            UpdateGPSDataPacket(packet, HAL_GetTick(), &gps_data);
            break;
        case PACKET_ID_IMU:
            UpdateIMUDataPacket(packet, HAL_GetTick(), &average_imu_data, &mag_data);
            break;
        case PACKET_ID_TEMPERATURE:
            UpdateTemperaturePacket(packet, HAL_GetTick(), /* M1_DTS */ 0, /* M1_ADC */ 0, /* M1_BMP */ 0, /* M1_IMU1 */ 0, /* M1_IMU2 */ 0, /* M1_MAG */ 0, /* M2_3V3 */ 0, /* M2_XBee */ 0, /* PU_bat */ 0, /* pressure_static */ 0, /* pressure_total */ 0);
            break;
        case PACKET_ID_POSITION:
            UpdatePositionPacket(packet, HAL_GetTick(), /* posX */ 0.0f, /* posY */ 0.0f, /* posZ */ 0.0f, /* velX */ 0.0f, /* velY */ 0.0f, /* velZ */ 0.0f);
            break;
        case PACKET_ID_ATTITUDE:
            UpdateAttitudePacket(packet, HAL_GetTick(), euler[0], euler[1], euler[2]);
            break;
        case PACKET_ID_KALMANMATRIX:
            UpdateKalmanMatrixPacket(packet, HAL_GetTick(), arm_mat_get_entry_f32(&P2, 0, 0), arm_mat_get_entry_f32(&P2, 1, 1), arm_mat_get_entry_f32(&P2, 2, 2), EKF2.x[0], EKF2.x[1], EKF2.x[2]);
            break;
        case PACKET_ID_MPC_INFO:
            UpdateMPCInfoPacket(packet, HAL_GetTick(), dt_1000Hz, acs_est_angle_deg, acs_target_angle_deg);
            break;
        case PACKET_ID_SPARK:
            packet = &spark_data; // Assuming spark_data is already updated with the latest data, we can just point to it here
            break;
        case PACKET_ID_STATE:
            UpdateStatePacket(packet, HAL_GetTick(), flight_sm.currentFlightState, flight_sm.timestamp_ms);
            break;
        default:
            break; // Unsupported packet type
    }
}