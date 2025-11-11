#include "packets.h"

#include <string.h>

#include "InterBoardCom.h"
#include "signalPlotter.h"
#include <string.h>
#include "spark.h"

int16_t float_to_int16_scaled(float value, float scale_factor);
int32_t float_to_int32_scaled(float value, float scale_factor);

//Create a Packet with the given ID and initialize its fields
DataPacket_t CreateDataPacket(PacketType_t Packet_ID) {
    DataPacket_t packet;
    packet.Packet_ID = Packet_ID;
    packet.timestamp = 0; // Initialize timestamp to 0, will be updated later
    memset(&packet.Data, 0, sizeof(packet.Data)); // Zero out the payload
    packet.crc = 0; // Initialize CRC to 0, will be calculated later
    return packet;
}

void calcCRC(DataPacket_t *packet) {
    // Simple XOR-based CRC calculation
    uint8_t *data = (uint8_t *)packet;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(DataPacket_t) - 1; i++) {
        crc ^= data[i];
    }
    packet->crc = crc;
}

uint8_t getCRC(DataPacket_t *packet) {
    // Simple XOR-based CRC calculation
    uint8_t *data = (uint8_t *)packet;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(DataPacket_t) - 1; i++) {
        crc ^= data[i];
    }
    return crc;
}

void PlotDataPacket(DataPacket_t *packet) {
    // Send the packet data to the signal plotter for visualization
    #ifdef SIGNAL_PLOTTER_OUT_GROUND
        switch(packet->Packet_ID) {
            case PACKET_ID_ATTITUDE:
                signalPlotter_sendData(0, packet->timestamp);
                signalPlotter_sendData(1, packet->Data.attitude.phi);
                signalPlotter_sendData(2, packet->Data.attitude.theta);
                signalPlotter_sendData(3, packet->Data.attitude.psi);
                break;
            
            case PACKET_ID_IMU:
                signalPlotter_sendData(0, packet->timestamp);
                signalPlotter_sendData(4, packet->Data.imu.accelX / 1000.0f);
                signalPlotter_sendData(5, packet->Data.imu.accelY / 1000.0f);
                signalPlotter_sendData(6, packet->Data.imu.accelZ / 1000.0f);
        
            default:
                break;
        }
    #endif
}

void UpdateStatusPacket(DataPacket_t *status_packet, uint32_t timestamp, int32_t status_flags, int32_t sensor_flags, int32_t error_flags, uint32_t flight_state) {
    // Update the status payload with the latest status information
    status_packet->timestamp = timestamp;
    status_packet->Data.status.status_flags = status_flags;
    status_packet->Data.status.sensor_status_flags = sensor_flags;
    status_packet->Data.status.error_flags = error_flags;
    status_packet->Data.status.State = flight_state;

    calcCRC(status_packet);
}

void UpdatePowerPacket(DataPacket_t *power_packet, uint32_t timestamp, float PU_bat_volt, float PU_out_pow, float PU_out_curr, float M2_bus_5V, float M2_bus_GPA_bat_volt) {
    // Update the power payload with the latest power information
    power_packet->timestamp = timestamp;
    power_packet->Data.power.PU_bat_bus_volt = float_to_int16_scaled(PU_bat_volt, 1e-3f);
    power_packet->Data.power.PU_pow = float_to_int16_scaled(PU_out_pow, 1e-3f);
    power_packet->Data.power.PU_curr = float_to_int16_scaled(PU_out_curr, 1e-3f);
    power_packet->Data.power.M2_bus_5V = float_to_int16_scaled(M2_bus_5V, 1e-3f);
    power_packet->Data.power.M2_bus_GPA_bat_volt = float_to_int16_scaled(M2_bus_GPA_bat_volt, 1e-3f);
    calcCRC(power_packet);
}

void UpdateIMUDataPacket(DataPacket_t *imu_packet, uint32_t timestamp, IMU_Data_t *imu_data, LIS3MDL_Data_t *mag_data) {
    imu_packet->timestamp = timestamp;

    // Update the IMU packet with the latest IMU data
    imu_packet->Data.imu.gyroX = float_to_int16_scaled(imu_data->gyro[0], 1e-6f);
    imu_packet->Data.imu.gyroY = float_to_int16_scaled(imu_data->gyro[1], 1e-6f);
    imu_packet->Data.imu.gyroZ = float_to_int16_scaled(imu_data->gyro[2], 1e-6f);
    imu_packet->Data.imu.accelX = float_to_int32_scaled(imu_data->accel[0], 1e-6f);
    imu_packet->Data.imu.accelY = float_to_int32_scaled(imu_data->accel[1], 1e-6f);
    imu_packet->Data.imu.accelZ = float_to_int32_scaled(imu_data->accel[2], 1e-6f);
    // Assuming mag data is available in mag_data structure
    imu_packet->Data.imu.magX = float_to_int16_scaled(mag_data->field[0], 1e-4f);
    imu_packet->Data.imu.magY = float_to_int16_scaled(mag_data->field[1], 1e-4f);
    imu_packet->Data.imu.magZ = float_to_int16_scaled(mag_data->field[2], 1e-4f);

    calcCRC(imu_packet);
}

void UpdateGPSDataPacket(DataPacket_t *gps_packet, uint32_t timestamp, UBX_NAV_PVT *gps_data) {
    gps_packet->timestamp = timestamp;

    // Update the GPS packet with the latest GPS data
    gps_packet->Data.gps.latitude = gps_data->lat;   // Latitude in 1e-7 degrees
    gps_packet->Data.gps.longitude = gps_data->lon;  // Longitude in 1e-7 degrees
    gps_packet->Data.gps.altitude = gps_data->height; // Altitude in mm
    gps_packet->Data.gps.speed = (int16_t)(gps_data->gSpeed / 100); // Speed over ground in cm/s
    gps_packet->Data.gps.course =  (int16_t)gps_data->headMot; // Course over ground °*1e-5

    calcCRC(gps_packet);
}

void UpdateTemperaturePacket(DataPacket_t *temp_packet, uint32_t timestamp, int32_t M1_DTS, int32_t M1_ADC, float M1_BMP, float M1_IMU1, float M1_IMU2, float M1_MAG, float M2_3V3, uint16_t M2_XBee, float PU_bat, float pressure_static, float pressure_total) {
    temp_packet->timestamp = timestamp;

    // Update the Temperature packet with the latest temperature information
    if (M1_DTS != NULL) temp_packet->Data.temperature.M1_DTS = (int16_t)M1_DTS;
    if (M1_ADC != NULL) temp_packet->Data.temperature.M1_ADC = (int16_t)M1_ADC;
    if (M1_BMP != INVALID_FLOAT) temp_packet->Data.temperature.M1_BMP = float_to_int16_scaled(M1_BMP, 0.01f);
    if (M1_IMU1 != INVALID_FLOAT) temp_packet->Data.temperature.M1_IMU1 = float_to_int16_scaled(M1_IMU1, 0.01f);
    if (M1_IMU2 != INVALID_FLOAT) temp_packet->Data.temperature.M1_IMU2 = float_to_int16_scaled(M1_IMU2, 0.01f);
    if (M1_MAG != INVALID_FLOAT) temp_packet->Data.temperature.M1_MAG = float_to_int16_scaled(M1_MAG, 0.01f);
    if (M2_3V3 != INVALID_FLOAT) temp_packet->Data.temperature.M2_3V3 = float_to_int16_scaled(M2_3V3, 0.01f);
    if (M2_XBee != NULL) temp_packet->Data.temperature.M2_XBee = (int16_t)M2_XBee;
    if (PU_bat != INVALID_FLOAT) temp_packet->Data.temperature.PU_bat = float_to_int16_scaled(PU_bat, 0.01f);
    if (pressure_static != INVALID_FLOAT) temp_packet->Data.temperature.pressure_static = pressure_static;
    if (pressure_total != INVALID_FLOAT) temp_packet->Data.temperature.pressure_total = pressure_total;

    calcCRC(temp_packet);
}

void UpdateKalmanMatrixPacket(DataPacket_t *kalman_packet, uint32_t timestamp, float P11, float P22, float P33, float EKF2_Heigth, float EKF2_vel, float EKF2_refPres) {
    kalman_packet->timestamp = timestamp;

    // Update the Kalman Matrix packet with the latest Kalman matrix information
    kalman_packet->Data.kalman.P11 = P11;
    kalman_packet->Data.kalman.P22 = P22;
    kalman_packet->Data.kalman.P33 = P33;
    kalman_packet->Data.kalman.EKF2_Heigth = EKF2_Heigth;
    kalman_packet->Data.kalman.EKF2_vel = EKF2_vel;
    kalman_packet->Data.kalman.EKF2_refPres = EKF2_refPres;

    calcCRC(kalman_packet);
}

void UpdatePositionPacket(DataPacket_t *position_packet, uint32_t timestamp, float posX, float posY, float posZ, float velX, float velY, float velZ) {
    position_packet->timestamp = timestamp;

    // Update the Position packet with the latest position information
    position_packet->Data.position.posX = float_to_int32_scaled(posX, 0.01f);
    position_packet->Data.position.posY = float_to_int32_scaled(posY, 0.01f);
    position_packet->Data.position.posZ = float_to_int32_scaled(posZ, 0.01f);
    position_packet->Data.position.velX = float_to_int32_scaled(velX, 0.01f);
    position_packet->Data.position.velY = float_to_int32_scaled(velY, 0.01f);
    position_packet->Data.position.velZ = float_to_int32_scaled(velZ, 0.01f);

    calcCRC(position_packet);
}

void UpdateAttitudePacket(DataPacket_t *attitude_packet, uint32_t timestamp, float phi, float theta, float psi) {
    attitude_packet->timestamp = timestamp;

    // Update the Attitude packet with the latest attitude information
    attitude_packet->Data.attitude.phi = phi;
    attitude_packet->Data.attitude.theta = theta;
    attitude_packet->Data.attitude.psi = psi;

    calcCRC(attitude_packet);
}

void CreateCommandPacket(DataPacket_t *command_packet, uint32_t timestamp, CommandTarget_t command_target, uint8_t command_id, uint8_t *params, size_t params_length) {
    command_packet->Packet_ID = PACKET_ID_COMMAND;
    command_packet->timestamp = timestamp;

    // Update the Command packet with the command information
    command_packet->Data.command.command_target = command_target;
    command_packet->Data.command.command_id = command_id;
    if (params != NULL && params_length > 0) {
        size_t copy_length = (params_length > sizeof(command_packet->Data.command.params)) ? sizeof(command_packet->Data.command.params) : params_length;
        memcpy(command_packet->Data.command.params, params, copy_length);
    }
    // Set all unused params to 0
    if (params_length < sizeof(command_packet->Data.command.params)) {
        memset(command_packet->Data.command.params + params_length, 0, sizeof(command_packet->Data.command.params) - params_length);
    }

    calcCRC(command_packet);
}

// Convert float to int16 with conversion factor
int16_t float_to_int16_scaled(float value, float scale_factor) {
    float scaled_value = value / scale_factor;
    
    // Add 0.5 for proper rounding (positive values)
    // Add -0.5 for proper rounding (negative values)
    if (scaled_value >= 0) {
        scaled_value += 0.5f;
    } else {
        scaled_value -= 0.5f;
    }
    
    // Clamp to int16 range to prevent overflow
    if (scaled_value > 32767.0f) {
        return 32767;
    } else if (scaled_value < -32768.0f) {
        return -32768;
    }
    
    return (int16_t)scaled_value;
}

// Convert int16 back to float
float int16_to_float_scaled(int16_t value, float scale_factor) {
    return (float)value * scale_factor;
}

// Convert float to int32 with conversion factor
int32_t float_to_int32_scaled(float value, float scale_factor) {
    float scaled_value = value / scale_factor;
    
    // Add 0.5 for proper rounding (positive values)
    // Add -0.5 for proper rounding (negative values)
    if (scaled_value >= 0) {
        scaled_value += 0.5f;
    } else {
        scaled_value -= 0.5f;
    }
    
    // Clamp to int16 range to prevent overflow
    if (scaled_value > 2147483647.0f) {
        return 2147483647;
    } else if (scaled_value < -2147483648.0f) {
        return -2147483648;
    }

    return (int32_t)scaled_value;
}

// Convert int32 back to float
float int32_to_float_scaled(int32_t value, float scale_factor) {
    return (float)value * scale_factor;
}

/* Secondary commands */
void PU_setCAM(bool on) {
    // Create and send command packet to Power Unit to toggle Camera Power
    DataPacket_t packet;
    uint8_t params[1] = {on};
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_PU_POWER_CAM, params, sizeof(params));
    sendcmdToTarget(&packet);
}

void PU_setREC(bool on) {
    // Create and send command packet to Power Unit to toggle Recovery Power
    DataPacket_t packet;
    uint8_t params[1] = {on};
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_PU_POWER_RECOVERY, params, sizeof(params));
    sendcmdToTarget(&packet);
}

void PU_setACS(bool on) {
    // Create and send command packet to Power Unit to toggle ACS Power
    DataPacket_t packet;
    uint8_t params[1] = {on};
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_PU_POWER_ACS, params, sizeof(params));
    sendcmdToTarget(&packet);
}

void Buzzer_PlayNote(char *note, uint32_t duration_ms) {
    uint8_t parameters[6];
    uint8_t length = strnlen(note, sizeof(note));
    
    parameters[0] = (uint8_t)(duration_ms >> 8);
    parameters[1] = (uint8_t)duration_ms;
    parameters[2] = length;
    parameters[3] = note[0];
    parameters[4] = note[1];
    if (length == 3) {
        parameters[5] = note[2];
    }

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_BUZZER_PLAYNOTE, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);
}

void Buzzer_PlaySong(uint8_t song_num) {
    uint8_t parameters[1];

    parameters[0] = song_num;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_BUZZER_PLAYSONG, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);
}

void Buzzer_PlaySongRepeat(uint8_t song_num, uint16_t period_ms) {
    uint8_t parameters[3];
    
    parameters[0] = (uint8_t)(period_ms >> 8);
    parameters[1] = (uint8_t)period_ms;
    parameters[2] = song_num;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_BUZZER_PLAYSONGREPEAT, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);
}

void Camera_Power(bool enable) {
    uint8_t parameters[1];
    parameters[0] = enable;
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_CAMERA, COMMAND_ID_CAMERA_POWER, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);
}

void Camera_Recording(bool enable) {
    uint8_t parameters[1];
    parameters[0] = enable;
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_CAMERA, COMMAND_ID_CAMERA_RECORD, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);
}

void Camera_SkipDate() {
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_CAMERA, COMMAND_ID_CAMERA_SKIPDATE, NULL, 0);
    sendcmdToTarget(&packet);
}

void Camera_Wifi(bool enable) {
    uint8_t parameters[1];
    parameters[0] = enable;
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_CAMERA, COMMAND_ID_CAMERA_WIFI, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);
}