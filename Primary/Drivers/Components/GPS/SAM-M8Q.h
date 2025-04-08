#ifndef SAM_M8Q_H_
#define SAM_M8Q_H_

#include "stm32h7xx_hal.h"

#include "main.h" //Für LED_Debuggen

extern I2C_HandleTypeDef hi2c2;

#define GPS_I2C        hi2c2 
#define GPS_I2C_ADDR  (0x42 << 1)  

uint8_t GPS_VER_CHECK(void);

#endif /* SAM_M8Q_H_ */