#ifndef LSM6DSR_H_
#define LSM6DSR_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"

extern SPI_HandleTypeDef hspi4;

#define IMU1_SPI        hspi4
#define IMU1_CS_PORT    GPIOE
#define IMU1_CS_PIN     GPIO_PIN_4

#define WHO_AM_I_REG 0x0F

bool LSM6DSR_SelfTest(void);

#endif /* LSM6DSR_H_ */