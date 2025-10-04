#include "packets.h"
#include "InterBoardCom.h"
#include <string.h>

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

/*void UpdateIMUDataPacket(DataPacket_t *imu_packet, uint32_t timestamp, IMU_Data_t *imu_data, LIS3MDL_Data_t *mag_data) {
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
}*/

/*void UpdateGPSDataPacket(DataPacket_t *gps_packet, uint32_t timestamp, UBX_NAV_PVT *gps_data) {
    gps_packet->timestamp = timestamp;

    // Update the GPS packet with the latest GPS data
    gps_packet->Data.gps.latitude = gps_data->lat;   // Latitude in 1e-7 degrees
    gps_packet->Data.gps.longitude = gps_data->lon;  // Longitude in 1e-7 degrees
    gps_packet->Data.gps.altitude = gps_data->height; // Altitude in mm
    gps_packet->Data.gps.speed = (int16_t)(gps_data->gSpeed / 100); // Speed over ground in cm/s
    gps_packet->Data.gps.course =  (int16_t)gps_data->headMot; // Course over ground °*1e-5

    calcCRC(gps_packet);
}*/

void UpdateTemperaturePacket(DataPacket_t *temp_packet, uint32_t timestamp, int32_t M1_DTS, int32_t M1_ADC, float M1_BMP, float M1_IMU1, float M1_IMU2, float M1_MAG, float M2_3V3, uint16_t M2_XBee, float PU_bat, float pressure) {
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
    if (pressure != INVALID_FLOAT) temp_packet->Data.temperature.pressure = pressure;

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

/**
 * @brief Initializes the circular buffer
 * @param cb Pointer to the circular buffer structure
 */
void DataCircBuffer_Init(DataCircularBuffer_t* cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

/**
 * @brief Pushes a packet into the circular buffer
 * @param cb Pointer to the circular buffer structure
 * @param packet Pointer to the packet to be added
 * @return 1 if successful, 0 if buffer is full
 */
uint8_t DataCircBuffer_Push(DataCircularBuffer_t* cb, DataPacket_t* packet) {
    if (cb->count >= INTERBOARD_BUFFER_SIZE) {
        return 0; // Buffer is full
    }
    
    // Disable interrupts to ensure atomic operation
    __disable_irq();
    
    // Copy packet to buffer
    memcpy(&cb->buffer[cb->head], packet, sizeof(DataPacket_t));
    
    // Update head pointer
    cb->head = (cb->head + 1) % INTERBOARD_BUFFER_SIZE;
    cb->count++;
    
    __enable_irq();
    
    return 1; // Success
}

/**
 * @brief Pops a packet from the circular buffer
 * @param cb Pointer to the circular buffer structure
 * @param packet Pointer to store the popped packet
 * @return 1 if successful, 0 if buffer is empty
 */
uint8_t DataCircBuffer_Pop(DataCircularBuffer_t* cb, DataPacket_t* packet) {
    if (cb->count == 0) {
        return 0; // Buffer is empty
    }
    
    // Disable interrupts to ensure atomic operation
    __disable_irq();
    
    // Copy packet from buffer
    memcpy(packet, &cb->buffer[cb->tail], sizeof(DataPacket_t));

    // Update tail pointer
    cb->tail = (cb->tail + 1) % INTERBOARD_BUFFER_SIZE;
    cb->count--;
    
    __enable_irq();
    
    return 1; // Success
}

/**
 * @brief Checks if the circular buffer is empty
 * @param cb Pointer to the circular buffer structure
 * @return 1 if empty, 0 if not empty
 */
uint8_t DataCircBuffer_IsEmpty(DataCircularBuffer_t* cb) {
    return (cb->count == 0);
}

/**
 * @brief Checks if the circular buffer is full
 * @param cb Pointer to the circular buffer structure
 * @return 1 if full, 0 if not full
 */
uint8_t DataCircBuffer_IsFull(DataCircularBuffer_t* cb) {
    return (cb->count >= INTERBOARD_BUFFER_SIZE);
}

/**
 * @brief Gets the current count of items in the buffer
 * @param cb Pointer to the circular buffer structure
 * @return Number of items in buffer
 */
uint16_t DataCircBuffer_Count(DataCircularBuffer_t* cb) {
    return cb->count;
}

/**
 * @brief Clears the circular buffer
 * @param cb Pointer to the circular buffer structure
 */
void DataCircBuffer_Clear(DataCircularBuffer_t* cb) {
    __disable_irq();
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
    __enable_irq();
}