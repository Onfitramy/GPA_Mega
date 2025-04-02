#ifndef ISM330DHCX_H_
#define ISM330DHCX_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"

extern SPI_HandleTypeDef hspi3;

#define IMU2_SPI        hspi3
#define IMU2_CS_PORT    GPIOD
#define IMU2_CS_PIN     GPIO_PIN_0

#define WHO_AM_I_REG 0x0F

bool ISM330DHCX_SelfTest(void);

#endif /* ISM330DHCX_H_ */