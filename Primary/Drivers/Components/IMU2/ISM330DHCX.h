#ifndef ISM330DHCX_H_
#define ISM330DHCX_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"

extern SPI_HandleTypeDef hspi3;

extern int16_t IMU2XL_FS_LSB;
extern int16_t IMU2G_FS_LSB;

typedef struct {
    int16_t accel[3];  // X, Y, Z
    int16_t gyro[3];   // X, Y, Z
    float temp;        // temperature
} ISM330DHCX_Data_t;

uint8_t IMU2_SelfTest(void);
uint8_t IMU2_VerifyDataReady(void);
HAL_StatusTypeDef IMU2_Init(void);
HAL_StatusTypeDef IMU2_ConfigXL(uint8_t ODR, uint8_t FS, bool LPF2);
HAL_StatusTypeDef IMU2_ConfigG(uint8_t ODR, uint8_t FS);

HAL_StatusTypeDef IMU2_ReadSensorData(ISM330DHCX_Data_t *data);

#define IMU2_SPI        hspi3
#define IMU2_CS_PORT    GPIOD
#define IMU2_CS_PIN     GPIO_PIN_0

#define ISM330DHCX_WHO_AM_I_VAL 0x6B

#define ISM330DHCX_SPI_READ     0x80  // RWB=1 for read

#define ISM330DHCX_FUNC_CFG_ACCESS          0x01 // Type: RW, Default: 0b00000000
#define ISM330DHCX_PIN_CTRL                 0x02 // Type: RW, Default: 0b00111111
#define ISM330DHCX_FIFO_CTRL1               0x07 // Type: RW, Default: 0b00000000
#define ISM330DHCX_FIFO_CTRL2               0x08 // Type: RW, Default: 0b00000000
#define ISM330DHCX_FIFO_CTRL3               0x09 // Type: RW, Default: 0b00000000
#define ISM330DHCX_FIFO_CTRL4               0x0A // Type: RW, Default: 0b00000000
#define ISM330DHCX_COUNTER_BDR_REG1         0x0B // Type: RW, Default: 0b00000000
#define ISM330DHCX_COUNTER_BDR_REG2         0x0C // Type: RW, Default: 0b00000000
#define ISM330DHCX_INT1_CTRL                0x0D // Type: RW, Default: 0b00000000
#define ISM330DHCX_INT2_CTRL                0x0E // Type: RW, Default: 0b00000000
#define ISM330DHCX_WHO_AM_I                 0x0F // Type: R,  Default: 0b01101011
#define ISM330DHCX_CTRL1_XL                 0x10 // Type: RW, Default: 0b00000000
#define ISM330DHCX_CTRL2_G                  0x11 // Type: RW, Default: 0b00000000
#define ISM330DHCX_CTRL3_C                  0x12 // Type: RW, Default: 0b00000100
#define ISM330DHCX_CTRL4_C                  0x13 // Type: RW, Default: 0b00000000
#define ISM330DHCX_CTRL5_C                  0x14 // Type: RW, Default: 0b00000000
#define ISM330DHCX_CTRL6_C                  0x15 // Type: RW, Default: 0b00000000
#define ISM330DHCX_CTRL7_G                  0x16 // Type: RW, Default: 0b00000000
#define ISM330DHCX_CTRL8_XL                 0x17 // Type: RW, Default: 0b00000000
#define ISM330DHCX_CTRL9_XL                 0x18 // Type: RW, Default: 0b11100000
#define ISM330DHCX_CTRL10_C                 0x19 // Type: RW, Default: 0b00000000
#define ISM330DHCX_ALL_INT_SRC              0x1A // Type: R,  Default: output
#define ISM330DHCX_WAKE_UP_SRC              0x1B // Type: R,  Default: output
#define ISM330DHCX_TAP_SRC                  0x1C // Type: R,  Default: output
#define ISM330DHCX_D6D_SRC                  0x1D // Type: R,  Default: output
#define ISM330DHCX_STATUS_REG               0x1E // Type: R,  Default: output
#define ISM330DHCX_OUT_TEMP_L               0x20 // Type: R,  Default: output
#define ISM330DHCX_OUT_TEMP_H               0x21 // Type: R,  Default: output
#define ISM330DHCX_OUTX_L_G                 0x22 // Type: R,  Default: output
#define ISM330DHCX_OUTX_H_G                 0x23 // Type: R,  Default: output
#define ISM330DHCX_OUTY_L_G                 0x24 // Type: R,  Default: output
#define ISM330DHCX_OUTY_H_G                 0x25 // Type: R,  Default: output
#define ISM330DHCX_OUTZ_L_G                 0x26 // Type: R,  Default: output
#define ISM330DHCX_OUTZ_H_G                 0x27 // Type: R,  Default: output
#define ISM330DHCX_OUTX_L_A                 0x28 // Type: R,  Default: output
#define ISM330DHCX_OUTX_H_A                 0x29 // Type: R,  Default: output
#define ISM330DHCX_OUTY_L_A                 0x2A // Type: R,  Default: output
#define ISM330DHCX_OUTY_H_A                 0x2B // Type: R,  Default: output
#define ISM330DHCX_OUTZ_L_A                 0x2C // Type: R,  Default: output
#define ISM330DHCX_OUTZ_H_A                 0x2D // Type: R,  Default: output
#define ISM330DHCX_EMB_FUNC_STATUS_MAINPAGE 0x35 // Type: R,  Default: output
#define ISM330DHCX_FSM_STATUS_A_MAINPAGE    0x36 // Type: R,  Default: output
#define ISM330DHCX_FSM_STATUS_B_MAINPAGE    0x37 // Type: R,  Default: output
#define ISM330DHCX_MLC_STATUS_MAINPAGE      0x38 // Type: R,  Default: output
#define ISM330DHCX_STATUS_MASTER_MAINPAGE   0x39 // Type: R,  Default: output
#define ISM330DHCX_FIFO_STATUS1             0x3A // Type: R,  Default: output
#define ISM330DHCX_FIFO_STATUS2             0x3B // Type: R,  Default: output
#define ISM330DHCX_TIMESTAMP0               0x40 // Type: R,  Default: output
#define ISM330DHCX_TIMESTAMP1               0x41 // Type: R,  Default: output
#define ISM330DHCX_TIMESTAMP2               0x42 // Type: R,  Default: output
#define ISM330DHCX_TIMESTAMP3               0x43 // Type: R,  Default: output
#define ISM330DHCX_TAP_CFG0                 0x56 // Type: RW, Default: 0b00000000
#define ISM330DHCX_TAP_CFG1                 0x57 // Type: RW, Default: 0b00000000
#define ISM330DHCX_TAP_CFG2                 0x58 // Type: RW, Default: 0b00000000
#define ISM330DHCX_TAP_THS_6D               0x59 // Type: RW, Default: 0b00000000
#define ISM330DHCX_INT_DUR2                 0x5A // Type: RW, Default: 0b00000000
#define ISM330DHCX_WAKE_UP_THS              0x5B // Type: RW, Default: 0b00000000
#define ISM330DHCX_WAKE_UP_DUR              0x5C // Type: RW, Default: 0b00000000
#define ISM330DHCX_FREE_FALL                0x5D // Type: RW, Default: 0b00000000
#define ISM330DHCX_MD1_CFG                  0x5E // Type: RW, Default: 0b00000000
#define ISM330DHCX_MD2_CFG                  0x5F // Type: RW, Default: 0b00000000
#define ISM330DHCX_INTERNAL_FREQ_FINE       0x63 // Type: R,  Default: output
#define ISM330DHCX_INT_OIS                  0x6F // Type: R,  Default: 0b00000000
#define ISM330DHCX_CTRL1_OIS                0x70 // Type: RW, Default: 0b00000000
#define ISM330DHCX_CTRL2_OIS                0x71 // Type: R,  Default: 0b00000000
#define ISM330DHCX_CTRL3_OIS                0x72 // Type: R,  Default: 0b00000000
#define ISM330DHCX_X_OFS_USR                0x73 // Type: RW, Default: 0b00000000
#define ISM330DHCX_Y_OFS_USR                0x74 // Type: RW, Default: 0b00000000
#define ISM330DHCX_Z_OFS_USR                0x75 // Type: RW, Default: 0b00000000
#define ISM330DHCX_FIFO_DATA_OUT_TAG        0x78 // Type: R,  Default: output
#define ISM330DHCX_FIFO_DATA_OUT_X_L        0x79 // Type: R,  Default: output
#define ISM330DHCX_FIFO_DATA_OUT_X_H        0x7A // Type: R,  Default: output
#define ISM330DHCX_FIFO_DATA_OUT_Y_L        0x7B // Type: R,  Default: output
#define ISM330DHCX_FIFO_DATA_OUT_Y_H        0x7C // Type: R,  Default: output
#define ISM330DHCX_FIFO_DATA_OUT_Z_L        0x7D // Type: R,  Default: output
#define ISM330DHCX_FIFO_DATA_OUT_Z_H        0x7E // Type: R,  Default: output

#define ISM330DHCX_FS_XL_2     0b00    // +- 2 g
#define ISM330DHCX_FS_XL_4     0b10    // +- 4 g
#define ISM330DHCX_FS_XL_8     0b11    // +- 8 g
#define ISM330DHCX_FS_XL_16    0b01    // +- 16 g

#define ISM330DHCX_FS_G_125    0b0010  // +-125 dps
#define ISM330DHCX_FS_G_250    0b0000  // +-250 dps
#define ISM330DHCX_FS_G_500    0b0100  // +-500 dps
#define ISM330DHCX_FS_G_1000   0b1000  // +-1000 dps
#define ISM330DHCX_FS_G_2000   0b1100  // +-2000 dps
#define ISM330DHCX_FS_G_4000   0b0001  // +-4000 dps

#define ISM330DHCX_ODR_1_6_Hz  0b1011  // low power only, XL only, XL_HM_MODE = 1 in CTRL6_C!
#define ISM330DHCX_ODR_12_5_Hz 0b0001  // low power
#define ISM330DHCX_ODR_26_Hz   0b0010  // low power
#define ISM330DHCX_ODR_52_Hz   0b0011  // low power
#define ISM330DHCX_ODR_104_Hz  0b0100  // normal mode
#define ISM330DHCX_ODR_208_Hz  0b0101  // normal mode
#define ISM330DHCX_ODR_416_Hz  0b0110  // high performance
#define ISM330DHCX_ODR_833_Hz  0b0111  // high performance
#define ISM330DHCX_ODR_1660_Hz 0b1000  // high performance
#define ISM330DHCX_ODR_3330_Hz 0b1001  // high performance
#define ISM330DHCX_ODR_6660_Hz 0b1010  // high performance

#endif /* ISM330DHCX_H_ */