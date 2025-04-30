#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"

extern SPI_HandleTypeDef hspi4;

typedef struct {
    int16_t field[3];  // X, Y, Z
    float temp;        // temperature
} LIS3MDL_Data_t;

uint8_t MAG_SelfTest(void);
uint8_t MAG_VerifyDataReady(void);
HAL_StatusTypeDef MAG_Init(void);
HAL_StatusTypeDef MAG_ReadSensorData(LIS3MDL_Data_t *data);
uint8_t MAG_Offset(int16_t set_x, int16_t set_y, int16_t set_z);

#define MAG_SPI        hspi4
#define MAG_CS_PORT    GPIOC
#define MAG_CS_PIN     GPIO_PIN_13

#define LIS3MDL_WHO_AM_I_VAL 0x3D

#define LIS3MDL_SPI_READ     0x80  // RWB=1 for read
#define LIS3MDL_SPI_AUTOINC  0x40  // MSB=1 for auto-increment

#define LIS3MDL_OFFSET_X_REG_L_M    0x05 // Type: RW, Default: 0b00000000
#define LIS3MDL_OFFSET_X_REG_H_M    0x06 // Type: RW, Default: 0b00000000
#define LIS3MDL_OFFSET_Y_REG_L_M    0x07 // Type: RW, Default: 0b00000000
#define LIS3MDL_OFFSET_Y_REG_H_M    0x08 // Type: RW, Default: 0b00000000
#define LIS3MDL_OFFSET_Z_REG_L_M    0x09 // Type: RW, Default: 0b00000000
#define LIS3MDL_OFFSET_Z_REG_H_M    0x0A // Type: RW, Default: 0b00000000
#define LIS3MDL_WHO_AM_I            0x0F // Type: R,  Default: 0b00111101
#define LIS3MDL_CTRL_REG1           0x20 // Type: RW, Default: 0b00010000
#define LIS3MDL_CTRL_REG2           0x21 // Type: RW, Default: 0b00000000
#define LIS3MDL_CTRL_REG3           0x22 // Type: RW, Default: 0b00000011
#define LIS3MDL_CTRL_REG4           0x23 // Type: RW, Default: 0b00000000
#define LIS3MDL_CTRL_REG5           0x24 // Type: RW, Default: 0b00000000
#define LIS3MDL_STATUS_REG          0x27 // Type: R,  Default: output
#define LIS3MDL_OUT_X_L             0x28 // Type: R,  Default: output
#define LIS3MDL_OUT_X_H             0x29 // Type: R,  Default: output
#define LIS3MDL_OUT_Y_L             0x2A // Type: R,  Default: output
#define LIS3MDL_OUT_Y_H             0x2B // Type: R,  Default: output
#define LIS3MDL_OUT_Z_L             0x2C // Type: R,  Default: output
#define LIS3MDL_OUT_Z_H             0x2D // Type: R,  Default: output
#define LIS3MDL_TEMP_OUT_L          0x2E // Type: R,  Default: output
#define LIS3MDL_TEMP_OUT_H          0x2F // Type: R,  Default: output
#define LIS3MDL_INT_CFG             0x30 // Type: RW, Default: 0b11101000
#define LIS3MDL_INT_SRC             0x31 // Type: R,  Default: 0b00000000
#define LIS3MDL_INT_THS_L           0x32 // Type: RW, Default: 0b00000000
#define LIS3MDL_INT_THS_H           0x33 // Type: RW, Default: 0b00000000

#endif /* LIS3MDL_H_ */