#ifndef IMUS_H_
#define IMUS_H_

#include "calibration_data.h"
#include "stm32h7xx_hal.h"
#include "stdbool.h"

extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi3;

extern SensorStatus imu1_status;
extern SensorStatus imu2_status;

typedef enum {
    IMU1 = 0,
    IMU2 = 1,
} IMU;

typedef struct {
    float accel[3];  // X, Y, Z
    float gyro[3];   // X, Y, Z
    float temp;      // temperature
    int16_t xl_fs_lsb;
    int16_t g_fs_lsb;
    CalibrationData_t calibration;
    float acc_data_history[8][3];
    bool active;
    bool passed_self_test;
    IMU imu;
} IMU_Data_t;

typedef struct {
    float accel[3];
    float gyro[3];
} IMU_AverageData_t;

typedef enum {
    ACC_FILTER_MODE_LOW_PASS = 0b0,
    ACC_FILTER_MODE_HIGH_PASS = 0b1,
} AccFilterMode;

typedef enum {
    ACC_FILTER_STAGE_FIRST = 0b0,
    ACC_FILTER_STAGE_SECOND = 0b1,
} AccFilterStage;

typedef enum {
    ACC_FILTER_BANDWIDTH_ODR_OVER_4 = 0b000,
    ACC_FILTER_BANDWIDTH_ODR_OVER_10 = 0b001,
    ACC_FILTER_BANDWIDTH_ODR_OVER_20 = 0b010,
    ACC_FILTER_BANDWIDTH_ODR_OVER_45 = 0b011,
    ACC_FILTER_BANDWIDTH_ODR_OVER_100 = 0b100,
    ACC_FILTER_BANDWIDTH_ODR_OVER_200 = 0b101,
    ACC_FILTER_BANDWIDTH_ODR_OVER_400 = 0b110,
    ACC_FILTER_BANDWIDTH_ODR_OVER_800 = 0b111,
} AccFilterBandwidth;

typedef enum {
    // See the data sheet (Table 58) for which bandwidth mode corresponds to which cutoff frequency.
    GYRO_FILTER_BANDWIDTH_1 = 0b000,
    GYRO_FILTER_BANDWIDTH_2 = 0b001,
    GYRO_FILTER_BANDWIDTH_3 = 0b010,
    GYRO_FILTER_BANDWIDTH_4 = 0b011,
    GYRO_FILTER_BANDWIDTH_5 = 0b100,
    GYRO_FILTER_BANDWIDTH_6 = 0b101,
    GYRO_FILTER_BANDWIDTH_7 = 0b110,
    GYRO_FILTER_BANDWIDTH_8 = 0b111,
} GyroFilterBandwidth;

typedef enum {
    INTERRUPT_PINS_1_AND_2 = 0b0,
    INTERRUPT_PIN_1 = 0b1,
} InterruptPins;

extern IMU_Data_t imu1_data;
extern IMU_Data_t imu2_data;
extern IMU_AverageData_t average_imu_data;

HAL_StatusTypeDef IMU_InitImu(IMU_Data_t *imu_data, IMU imu, GPA_Mega gpa_mega);
HAL_StatusTypeDef IMU_Update(IMU_Data_t *imu_data);
void IMU_Average(IMU_Data_t *imu_data_1, IMU_Data_t *imu_data_2, IMU_AverageData_t *average_imu_data);

void IMU_SwitchSensors(IMU_Data_t *imu_data);
uint8_t IMU_SelfTest(IMU_Data_t *imu_data);
uint8_t IMU_VerifyDataReady(const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_ConfigXL(uint8_t ODR, uint8_t FS, bool LPF2, IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_ConfigG(uint8_t ODR, uint8_t FS, IMU_Data_t *imu_data);

HAL_StatusTypeDef IMU_ReadSensorData(IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetAccFilterMode(AccFilterMode filter_mode, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetAccFilterStage(AccFilterStage filter_stage, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetAccFilterBandwidth(AccFilterBandwidth filter_bandwidth, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetGyroLowPassFilter(bool low_pass_enabled, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetGyroFilterBandwidth(GyroFilterBandwidth filter_bandwidth, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetInterrupt1Gyro(bool interrupt_enabled, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetInterrupt1Acc(bool interrupt_enabled, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetInterrupt2Gyro(bool interrupt_enabled, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetInterrupt2Acc(bool interrupt_enabled, const IMU_Data_t *imu_data);
HAL_StatusTypeDef IMU_SetInterruptPins(InterruptPins interrupt_pins, const IMU_Data_t *imu_data);

#define IMU_INACTIVE_EQUAL_COUNT 4

#define IMU1_SPI        hspi4
#define IMU1_CS_PORT    GPIOE
#define IMU1_CS_PIN     GPIO_PIN_4
#define IMU2_SPI        hspi3
#define IMU2_CS_PORT    GPIOD
#define IMU2_CS_PIN     GPIO_PIN_0

#define IMU_WHO_AM_I_VAL 0x6B

#define IMU_SPI_READ     0x80  // RWB=1 for read

#define IMU_FUNC_CFG_ACCESS          0x01 // Type: RW, Default: 0b00000000
#define IMU_PIN_CTRL                 0x02 // Type: RW, Default: 0b00111111
// only for LSM6DSR
#define LSM6DSR_S4S_TPH_L            0x04 // Type: RW, Default: 0b00000000
// only for LSM6DSR
#define LSM6DSR_S4S_TPH_H            0x05 // Type: RW, Default: 0b00000000
// only for LSM6DSR
#define LSM6DSR_S4S_RR               0x06 // Type: RW, Default: 0b00000000
#define IMU_FIFO_CTRL1               0x07 // Type: RW, Default: 0b00000000
#define IMU_FIFO_CTRL2               0x08 // Type: RW, Default: 0b00000000
#define IMU_FIFO_CTRL3               0x09 // Type: RW, Default: 0b00000000
#define IMU_FIFO_CTRL4               0x0A // Type: RW, Default: 0b00000000
#define IMU_COUNTER_BDR_REG1         0x0B // Type: RW, Default: 0b00000000
#define IMU_COUNTER_BDR_REG2         0x0C // Type: RW, Default: 0b00000000
#define IMU_INT1_CTRL                0x0D // Type: RW, Default: 0b00000000
#define IMU_INT2_CTRL                0x0E // Type: RW, Default: 0b00000000
#define IMU_WHO_AM_I                 0x0F // Type: R,  Default: 0b01101011
#define IMU_CTRL1_XL                 0x10 // Type: RW, Default: 0b00000000
#define IMU_CTRL2_G                  0x11 // Type: RW, Default: 0b00000000
#define IMU_CTRL3_C                  0x12 // Type: RW, Default: 0b00000100
#define IMU_CTRL4_C                  0x13 // Type: RW, Default: 0b00000000
#define IMU_CTRL5_C                  0x14 // Type: RW, Default: 0b00000000
#define IMU_CTRL6_C                  0x15 // Type: RW, Default: 0b00000000
#define IMU_CTRL7_G                  0x16 // Type: RW, Default: 0b00000000
#define IMU_CTRL8_XL                 0x17 // Type: RW, Default: 0b00000000
#define IMU_CTRL9_XL                 0x18 // Type: RW, Default: 0b11100000
#define IMU_CTRL10_C                 0x19 // Type: RW, Default: 0b00000000
#define IMU_ALL_INT_SRC              0x1A // Type: R,  Default: output
#define IMU_WAKE_UP_SRC              0x1B // Type: R,  Default: output
#define IMU_TAP_SRC                  0x1C // Type: R,  Default: output
#define IMU_D6D_SRC                  0x1D // Type: R,  Default: output
#define IMU_STATUS_REG               0x1E // Type: R,  Default: output
#define IMU_OUT_TEMP_L               0x20 // Type: R,  Default: output
#define IMU_OUT_TEMP_H               0x21 // Type: R,  Default: output
#define IMU_OUTX_L_G                 0x22 // Type: R,  Default: output
#define IMU_OUTX_H_G                 0x23 // Type: R,  Default: output
#define IMU_OUTY_L_G                 0x24 // Type: R,  Default: output
#define IMU_OUTY_H_G                 0x25 // Type: R,  Default: output
#define IMU_OUTZ_L_G                 0x26 // Type: R,  Default: output
#define IMU_OUTZ_H_G                 0x27 // Type: R,  Default: output
#define IMU_OUTX_L_A                 0x28 // Type: R,  Default: output
#define IMU_OUTX_H_A                 0x29 // Type: R,  Default: output
#define IMU_OUTY_L_A                 0x2A // Type: R,  Default: output
#define IMU_OUTY_H_A                 0x2B // Type: R,  Default: output
#define IMU_OUTZ_L_A                 0x2C // Type: R,  Default: output
#define IMU_OUTZ_H_A                 0x2D // Type: R,  Default: output
#define IMU_EMB_FUNC_STATUS_MAINPAGE 0x35 // Type: R,  Default: output
#define IMU_FSM_STATUS_A_MAINPAGE    0x36 // Type: R,  Default: output
#define IMU_FSM_STATUS_B_MAINPAGE    0x37 // Type: R,  Default: output
// only for ISM330DHCX
#define IMU_MLC_STATUS_MAINPAGE      0x38 // Type: R,  Default: output
#define IMU_STATUS_MASTER_MAINPAGE   0x39 // Type: R,  Default: output
#define IMU_FIFO_STATUS1             0x3A // Type: R,  Default: output
#define IMU_FIFO_STATUS2             0x3B // Type: R,  Default: output
#define IMU_TIMESTAMP0               0x40 // Type: R,  Default: output
#define IMU_TIMESTAMP1               0x41 // Type: R,  Default: output
#define IMU_TIMESTAMP2               0x42 // Type: R,  Default: output
#define IMU_TIMESTAMP3               0x43 // Type: R,  Default: output
#define IMU_TAP_CFG0                 0x56 // Type: RW, Default: 0b00000000
#define IMU_TAP_CFG1                 0x57 // Type: RW, Default: 0b00000000
#define IMU_TAP_CFG2                 0x58 // Type: RW, Default: 0b00000000
#define IMU_TAP_THS_6D               0x59 // Type: RW, Default: 0b00000000
#define IMU_INT_DUR2                 0x5A // Type: RW, Default: 0b00000000
#define IMU_WAKE_UP_THS              0x5B // Type: RW, Default: 0b00000000
#define IMU_WAKE_UP_DUR              0x5C // Type: RW, Default: 0b00000000
#define IMU_FREE_FALL                0x5D // Type: RW, Default: 0b00000000
#define IMU_MD1_CFG                  0x5E // Type: RW, Default: 0b00000000
#define IMU_MD2_CFG                  0x5F // Type: RW, Default: 0b00000000
// only for LSM6DSR
#define LSM6DSR_S4S_ST_CMD_CODE      0x60 // Type: RW, Default: 0b00000000
// only for LSM6DSR
#define LSM6DSR_S4S_DT_REG           0x61 // Type: RW, Default: 0b00000000
// only for LSM6DSR
#define LSM6DSR_I3C_BUS_AVB          0x62 // Type: RW, Default: 0b00000000
#define IMU_INTERNAL_FREQ_FINE       0x63 // Type: R,  Default: output
#define IMU_INT_OIS                  0x6F // Type: R,  Default: 0b00000000
#define IMU_CTRL1_OIS                0x70 // Type: R, Default: 0b00000000
#define IMU_CTRL2_OIS                0x71 // Type: R,  Default: 0b00000000
#define IMU_CTRL3_OIS                0x72 // Type: R,  Default: 0b00000000
#define IMU_X_OFS_USR                0x73 // Type: RW, Default: 0b00000000
#define IMU_Y_OFS_USR                0x74 // Type: RW, Default: 0b00000000
#define IMU_Z_OFS_USR                0x75 // Type: RW, Default: 0b00000000
#define IMU_FIFO_DATA_OUT_TAG        0x78 // Type: R,  Default: output
#define IMU_FIFO_DATA_OUT_X_L        0x79 // Type: R,  Default: output
#define IMU_FIFO_DATA_OUT_X_H        0x7A // Type: R,  Default: output
#define IMU_FIFO_DATA_OUT_Y_L        0x7B // Type: R,  Default: output
#define IMU_FIFO_DATA_OUT_Y_H        0x7C // Type: R,  Default: output
#define IMU_FIFO_DATA_OUT_Z_L        0x7D // Type: R,  Default: output
#define IMU_FIFO_DATA_OUT_Z_H        0x7E // Type: R,  Default: output

#define IMU_FS_XL_2     0b00    // +- 2 g
#define IMU_FS_XL_4     0b10    // +- 4 g
#define IMU_FS_XL_8     0b11    // +- 8 g
#define IMU_FS_XL_16    0b01    // +- 16 g

#define IMU_FS_G_125    0b0010  // +-125 dps
#define IMU_FS_G_250    0b0000  // +-250 dps
#define IMU_FS_G_500    0b0100  // +-500 dps
#define IMU_FS_G_1000   0b1000  // +-1000 dps
#define IMU_FS_G_2000   0b1100  // +-2000 dps
#define IMU_FS_G_4000   0b0001  // +-4000 dps

#define IMU_ODR_1_6_Hz  0b1011  // low power only, XL only, XL_HM_MODE = 1 in CTRL6_C!
#define IMU_ODR_12_5_Hz 0b0001  // low power
#define IMU_ODR_26_Hz   0b0010  // low power
#define IMU_ODR_52_Hz   0b0011  // low power
#define IMU_ODR_104_Hz  0b0100  // normal mode
#define IMU_ODR_208_Hz  0b0101  // normal mode
#define IMU_ODR_416_Hz  0b0110  // high performance
#define IMU_ODR_833_Hz  0b0111  // high performance
#define IMU_ODR_1660_Hz 0b1000  // high performance
#define IMU_ODR_3330_Hz 0b1001  // high performance
#define IMU_ODR_6660_Hz 0b1010  // high performance

#endif /* IMUS_H_ */