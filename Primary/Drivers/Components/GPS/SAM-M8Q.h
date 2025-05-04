#ifndef SAM_M8Q_H_
#define SAM_M8Q_H_

#include "stm32h7xx_hal.h"

#include "main.h" //Für LED_Debuggen

extern I2C_HandleTypeDef hi2c2;

#define GPS_I2C        hi2c2 
#define GPS_I2C_ADDR  (0x42 << 1) 

#define GPS_MESSAGE_CLASS_CFG 0x06

#define GPS_MESSAGE_ID_0 0x00

#define GPS_PORT_ID 0x00

typedef struct
{
    int32_t messageClass;       

    int32_t messageId;

    char* messageBody;

    uint8_t status;

} UBX_MessageType; //On failure to extract message all fields 0/NULL

#pragma pack(push, 1)
typedef struct {
    uint32_t iTOW;     // GPS time of week in milliseconds
    int32_t lon;       // Longitude in degrees * 1e-7
    int32_t lat;       // Latitude in degrees * 1e-7
    int32_t height;    // Height above ellipsoid in millimeters
    int32_t hMSL;      // Height above mean sea level in millimeters
    uint32_t hAcc;     // Horizontal accuracy estimate in millimeters
    uint32_t vAcc;     // Vertical accuracy estimate in millimeters
} ubx_nav_posllh_t;
#pragma pack(pop)

uint8_t GPS_VER_CHECK(void);

void GPS_Init(void);

UBX_MessageType ublox_ReadOutput(char* UBX_MessageReturn);

uint8_t GPS_ReadSensorData(ubx_nav_posllh_t *posllh);

#endif /* SAM_M8Q_H_ */