#ifndef Packets_H_
#define Packets_H_

#include "stm32f4xx_hal.h"
/*This file includes all public Packets for the differten devices and sending modes*/
/*They are used for radio transmittion, flash/SD saving and interBoardCommunication*/
/*---------------------*/

#define INVALID_FLOAT -999.0f 

typedef enum {
    PACKET_ID_STATUS = 0x01, // VR data packet
    PACKET_ID_POWER = 0x02, // Power data packet
    PACKET_ID_GPS = 0x03, // GPS data packet
    PACKET_ID_IMU = 0x04, // IMU data packet
    PACKET_ID_TEMPERATURE = 0x05, // Temperature data packet
    PACKET_ID_POSITION = 0x06, // Position data packet
    PACKET_ID_ATTITUDE = 0x07, // Attitude data packet
    PACKET_ID_KALMANMATRIX = 0x08, // Kalman Matrix data packet
} PacketType_t;

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


#endif /* Packets_H_ */