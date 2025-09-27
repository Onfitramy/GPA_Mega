#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include "calibration_data.h"
#include "stm32h7xx_hal.h"
#include "stdbool.h"

extern SPI_HandleTypeDef hspi4;

extern int16_t MAG_FS_LSB;

typedef struct {
    float field[3];  // X, Y, Z
    float temp;        // temperature
    CalibrationData_t calibration;
} LIS3MDL_Data_t;

uint8_t MAG_SelfTest(void);
uint8_t MAG_VerifyDataReady(void);
HAL_StatusTypeDef MAG_Init(void);
HAL_StatusTypeDef MAG_ConfigSensor(uint8_t OperatingMode, uint8_t DataRate, uint8_t FullScale, bool fast_ODR, bool temp_en);
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

#define LIS3MDL_OM_LOW       0b00
#define LIS3MDL_OM_MEDIUM    0b01
#define LIS3MDL_OM_HIGH      0b10
#define LIS3MDL_OM_ULTRA     0b11

#define LIS3MDL_ODR_0_625_Hz    0b000
#define LIS3MDL_ODR_1_25_Hz     0b001
#define LIS3MDL_ODR_2_5_Hz      0b010
#define LIS3MDL_ODR_5_Hz        0b011
#define LIS3MDL_ODR_10_Hz       0b100
#define LIS3MDL_ODR_20_Hz       0b101
#define LIS3MDL_ODR_40_Hz       0b110
#define LIS3MDL_ODR_80_Hz       0b111

#define LIS3MDL_FS_4    0b00
#define LIS3MDL_FS_8    0b01
#define LIS3MDL_FS_12   0b10
#define LIS3MDL_FS_16   0b11

#define LIS3MDL_FAST_ODR_ON 1
#define LIS3MDL_FAST_ODR_OFF 0

#define LIS3MDL_TEMP_ON 1
#define LIS3MDL_TEMP_OFF 0

#endif /* LIS3MDL_H_ */