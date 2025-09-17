#ifndef Packets_H_
#define Packets_H_

#include "stm32h7xx_hal.h"
/*This file includes all public Packets for the differten devices and sending modes*/
/*They are used for radio transmittion, flash/SD saving and interBoardCommunication*/

typedef enum {
    PACKET_ID_Status = 0x01, // VR data packet
    PACKET_ID_BATTERY = 0x02, // Battery data packet
    PACKET_ID_GPS = 0x03, // GPS data packet
    PACKET_ID_IMU = 0x04, // IMU data packet
    PACKET_ID_TEMPERATURE = 0x05, // Temperature data packet
    PACKET_ID_PRESSURE = 0x06, // Pressure data packet
    PACKET_ID_HUMIDITY = 0x07, // Humidity data packet
    PACKET_ID_POSITION = 0x08, // Position data packet
    PACEKT_ID_ATTITUDE = 0x09, // Attitude data packet
} PacketType_t;

/* Packet and Payload structure definitions */
/* Each Payload has to be exactly 26 bytes */
#pragma pack(push, 1)
typedef struct {
    int32_t status_flags, sensor_status_flags, error_flags;
    int32_t  Unused1, Unused2, Unused3;
    int16_t Unused4;
} StatusPayload_t;

typedef struct {
    uint32_t voltage5V0bus, voltageBATbus, unused1;
    uint32_t unused2, unused3, unused4;
    int16_t Unused4;
} BatteryPayload_t;

typedef struct {
    int32_t latitude, longitude, altitude;
    int16_t speed, course, unused1, unused2, unused3, unused4, unused5;
} GPSPayload_t;

typedef struct {
    int16_t gyroX, gyroY, gyroZ;
    int16_t accelX, accelY, accelZ;
    int16_t magX, magY, magZ;
    uint16_t unused1, unused2, unused3, unused4;
} IMUPayload_t;

typedef struct {
    int32_t posX, posY, posZ;
    int16_t velX, velY, velZ;
    int16_t accX, accY, accZ;
    uint16_t unused1;
} PositionPayload_t;

typedef union {
    StatusPayload_t status;
    BatteryPayload_t battery;
    GPSPayload_t gps;
    IMUPayload_t imu;
    PositionPayload_t position;
    uint8_t raw[26];
} PayloadData_u;

typedef struct {
    uint8_t Packet_ID; // Packet ID
    uint32_t timestamp;
    PayloadData_u Data;
    uint8_t crc;
} DataPacket_t;
#pragma pack(pop)




#endif /* Packets_H_ */