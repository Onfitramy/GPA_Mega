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

uint8_t GPS_VER_CHECK(void);

void GPS_Init(void);

UBX_MessageType ublox_ReadOutput(char* UBX_MessageReturn);

#endif /* SAM_M8Q_H_ */