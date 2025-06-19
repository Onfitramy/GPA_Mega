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

#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp;
    int32_t Status, Warnings, Errors;
    int16_t  Unused1, Unused2, Unused3;
    int16_t Unused4, Unused5, Unused6;
} StatusPacket_t;

typedef struct {
    uint32_t timestamp;
    uint32_t voltage5V0bus, voltageBATbus, unused1;
    uint32_t unused2, unused3, unused4;
} BatteryPacket_t;

typedef struct {
    uint32_t timestamp;
    int32_t latitude, longitude, altitude;
    int16_t speed, course, unused1;
    int16_t unused2, unused3, unused4;
} GPSPacket_t;

typedef struct {
    uint32_t timestamp;
    int16_t gyroX, gyroY, gyroZ;
    int16_t accelX, accelY, accelZ;
    int16_t magX, magY, magZ;
    uint16_t unused1, unused2, unused3;
} IMUPacket_t;

typedef struct {
    uint32_t timestamp;
    int32_t posX, posY, posZ;
    int16_t velX, velY, velZ;
    int16_t accX, accY, accZ;
} PositionPacket_t;

typedef union {
    StatusPacket_t status;
    BatteryPacket_t battery;
    GPSPacket_t gps;
    IMUPacket_t imu;
    PositionPacket_t position;
    uint8_t raw[28];
} PacketData_u;

typedef struct {
    uint8_t Header[2]; // 0x41, 0x45 (AP)
    uint8_t Packet_ID; // Packet ID
    PacketData_u Data;
    uint8_t crc;
} DataPacket_t;
#pragma pack(pop)




#endif /* Packets_H_ */