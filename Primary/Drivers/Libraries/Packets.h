#ifndef Packets_H_
#define Packets_H_

#include "stm32h7xx_hal.h"
#include "IMUS.h"
#include "LIS3MDL.h"
#include "SAM-M8Q.h"
/*This file includes all public Packets for the differten devices and sending modes*/
/*They are used for radio transmittion, flash/SD saving and interBoardCommunication*/

#define INVALID_FLOAT -999.0f 

typedef enum __attribute__((packed)){
    PACKET_ID_STATUS = 0x01, // VR data packet
    PACKET_ID_POWER = 0x02, // Power data packet
    PACKET_ID_GPS = 0x03, // GPS data packet
    PACKET_ID_IMU = 0x04, // IMU data packet
    PACKET_ID_TEMPERATURE = 0x05, // Temperature data packet
    PACKET_ID_POSITION = 0x06, // Position data packet
    PACKET_ID_ATTITUDE = 0x07, // Attitude data packet
    PACKET_ID_KALMANMATRIX = 0x08, // Kalman Matrix data packet

    PACKET_ID_COMMAND = 0x10, // Command packet
    
    // Force to 8-bit size
    PACKET_ID_FORCE_8BIT = 0xFF
} PacketType_t;


typedef enum __attribute__((packed)){
    COMMAND_TARGET_NONE = 0x00,
    COMMAND_TARGET_SPECIAL = 0x01,
    COMMAND_TARGET_CAMERA = 0x02,
    COMMAND_TARGET_STATE = 0x03,
    COMMAND_TARGET_POWERUNIT = 0x04,
    COMMAND_TARGET_TESTING = 0x05,
    COMMAND_TARGET_STORAGE = 0x06,
    COMMAND_TARGET_GROUNDSTATION = 0x07,
    COMMAND_TARGET_LOGGING = 0x08,
    COMMAND_TARGET_ACK = 0x10,
} CommandTarget_t;

/* Packet and Payload structure definitions */
/* Each Payload has to be exactly 26 bytes */
#pragma pack(push, 1)
typedef struct {
    int32_t status_flags, sensor_status_flags, error_flags;
    int16_t State;
    int32_t  Unused1, Unused2, Unused3;
} StatusPayload_t;

typedef struct {
    uint32_t M1_5V_bus, M1_BAT_bus_volt, unused1; // 12 bytes
    uint16_t M2_bus_5V, M2_bus_GPA_bat_volt, unused2; // 18 bytes
    uint16_t PU_pow, PU_curr, PU_bat_bus_volt; //24
    uint16_t Unused3;  // 26 bytes
} PowerPayload_t;

typedef struct {
    int32_t latitude, longitude, altitude;
    int16_t speed, course, unused1, unused2, unused3, unused4, unused5;
} GPSPayload_t;

typedef struct {
    int16_t gyroX, gyroY, gyroZ; // 6 bytes
    int32_t accelX, accelY, accelZ; // 18 bytes
    int16_t magX, magY, magZ; // 24 bytes
    uint16_t unused1; // 26 bytes
} IMUPayload_t;

typedef struct {
    int16_t M1_DTS, M1_ADC, M1_BMP, M1_IMU1, M1_IMU2, M1_MAG; // 12 bytes
    int16_t M2_3V3, M2_XBee; // 16 bytes
    uint16_t PU_bat; // 18 bytes
    float pressure; // 22 bytes
    uint16_t unused1, unused2; // 26 bytes
} TemperaturePayload_t;

typedef struct {
    int32_t posX, posY, posZ;
    int32_t velX, velY, velZ;
    uint16_t unused1;
} PositionPayload_t;

typedef struct {
    float phi, theta, psi;
    uint32_t unused1, unused2, unused3;
    uint16_t unused4;
} AttitudePayload_t;

typedef struct {
    float P11, P22, P33;
    float EKF2_Heigth, EKF2_vel, EKF2_refPres;
    uint16_t unused1;
}KalmanMatrixPayload_t;

typedef struct {
    float test1, test2, test3, test4, test5, test6; // 24 bytes
    uint16_t unused1; // 26 bytes
} TestPayload_t;

typedef struct {
    CommandTarget_t command_target;
    uint8_t command_id;
    uint8_t params[24];
} CommandPayload_t;

typedef union {
    StatusPayload_t status;
    PowerPayload_t power;
    GPSPayload_t gps;
    IMUPayload_t imu;
    TemperaturePayload_t temperature;
    PositionPayload_t position;
    AttitudePayload_t attitude;
    KalmanMatrixPayload_t kalman;
    TestPayload_t test;
    CommandPayload_t command;
    uint8_t raw[26];
} PayloadData_u;

typedef struct {
    uint8_t Packet_ID; // Packet ID
    uint32_t timestamp;
    PayloadData_u Data;
    uint8_t crc;
} DataPacket_t;
#pragma pack(pop)


DataPacket_t CreateDataPacket(PacketType_t Packet_ID);
void UpdateStatusPacket(DataPacket_t *status_packet, uint32_t timestamp, int32_t status_flags, int32_t sensor_flags, int32_t error_flags, uint32_t flight_state);
void UpdateIMUDataPacket(DataPacket_t *imu_packet, uint32_t timestamp, IMU_Data_t *imu_data, LIS3MDL_Data_t *mag_data);
void UpdateGPSDataPacket(DataPacket_t *gps_packet, uint32_t timestamp, UBX_NAV_PVT *gps_data);
void UpdateTemperaturePacket(DataPacket_t *temp_packet, uint32_t timestamp, int32_t M1_DTS, int32_t M1_ADC, float M1_BMP, float M1_IMU1, float M1_IMU2, float M1_MAG, float M2_3V3, uint16_t M2_XBee, float PU_bat, float pressure);
void UpdatePowerPacket(DataPacket_t *power_packet, uint32_t timestamp, float PU_bat_volt, float PU_out_pow, float PU_out_curr, float M2_bus_5V, float M2_bus_GPA_bat_volt);
void UpdateKalmanMatrixPacket(DataPacket_t *kalman_packet, uint32_t timestamp, float P11, float P22, float P33, float EKF2_Heigth, float EKF2_vel, float EKF2_refPres);
void UpdatePositionPacket(DataPacket_t *position_packet, uint32_t timestamp, float posX, float posY, float posZ, float velX, float velY, float velZ);
void UpdateAttitudePacket(DataPacket_t *attitude_packet, uint32_t timestamp, float phi, float theta, float psi);
void PlotDataPacket(DataPacket_t *packet);
void CreateCommandPacket(DataPacket_t *command_packet, uint32_t timestamp, CommandTarget_t command_target, uint8_t command_id, uint8_t *params, size_t params_length);
#endif /* Packets_H_ */