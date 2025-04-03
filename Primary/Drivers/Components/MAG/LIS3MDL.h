#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"

#include "main.h" //ONLY FOR TESTING WITH THE LED

extern SPI_HandleTypeDef hspi4;

#define MAG_SPI        hspi4
#define MAG_CS_PORT    GPIOC
#define MAG_CS_PIN     GPIO_PIN_13

#define WHO_AM_I_REG 0x0F

bool LIS3MDL_SelfTest(void);

#endif /* LIS3MDL_H_ */