#include "packets.h"
#include "InterBoardCom.h"

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

void UpdateStatusPayload(StatusPayload_t *status_payload) {
    // Update the status payload with the latest status information
    // Implement the logic to fill the status_payload structure
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
    gps_packet->Data.gps.course = (int16_t)(gps_data->headMot / 100); // Course over ground in 0.01 degrees

    calcCRC(gps_packet);
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