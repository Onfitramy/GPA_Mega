#ifndef PowerUnit_H_
#define PowerUnit_H_

#include "main.h"
#include "math.h"

#define MAX_CURRENT         32.768f // A
#define SHUNT_RESISTANCE    0.0016f  // Ohm

extern I2C_HandleTypeDef hi2c2;

// R = 1 mOhm
// Current_LSB = 0.001
// Power_LSB = 0.020
// Shunt Voltage LSB = 10 uV
// Bus Voltage LSB = 4 mV

HAL_StatusTypeDef INA219_writeRegister(uint8_t reg, uint16_t data);
HAL_StatusTypeDef INA219_readRegister(uint8_t reg, uint16_t *data);

HAL_StatusTypeDef INA219_reset();

void INA219_setBusVoltageRange(uint8_t range);
void INA219_setShuntVoltageRange(uint8_t range);
void INA219_setBusADC(uint8_t mode);
void INA219_setShuntADC(uint8_t mode);
void INA219_setOperatingMode(uint8_t mode);
void INA219_setCurrentCalibration(float max_current, float shunt_resistance);

void INA219_readShuntVoltage(float *voltage);
void INA219_readBusVoltage(float *voltage);
void INA219_readPower(float *power);
void INA219_readCurrent(float *current);

#define INA219_I2C_ADDR (0x40 << 1)

/* Registers */
#define INA219_CONFIG_REG           0x00    // RW   default: 0b00111001 10011111
#define INA219_SHUNT_VOLTAGE_REG    0x01    // R    
#define INA219_BUS_VOLTAGE_REG      0x02    // R    
#define INA219_POWER_REG            0x03    // R    default: 0b00000000 00000000
#define INA219_CURRENT_REG          0x04    // R    default: 0b00000000 00000000
#define INA219_CALIBRATION_REG      0x05    // RW   default: 0b00000000 00000000

/* Commands */
#define INA219_RESET_CMD    0xB99F

/* Settings */
#define INA219_BUS_RANGE_16V    0b0
#define INA219_BUS_RANGE_32V    0b1

#define INA219_SHUNT_RANGE_40mV     0b00    // GAIN = 1
#define INA219_SHUNT_RANGE_80mV     0b01    // GAIN = 1/2
#define INA219_SHUNT_RANGE_160mV    0b10    // GAIN = 1/4
#define INA219_SHUNT_RANGE_320mV    0b11    // GAIN = 1/8

#define INA219_ADC_MODE_9_BIT       0b0000  // Conversion Time: 84 us
#define INA219_ADC_MODE_10_BIT      0b0001  // Conversion Time: 148 us
#define INA219_ADC_MODE_11_BIT      0b0010  // Conversion Time: 276 us
#define INA219_ADC_MODE_12_BIT      0b0011  // Conversion Time: 532 us
#define INA219_ADC_SAMPLES_2        0b1001  // Conversion Time: 1.06 ms
#define INA219_ADC_SAMPLES_4        0b1010  // Conversion Time: 2.13 ms
#define INA219_ADC_SAMPLES_8        0b1011  // Conversion Time: 4.26 ms
#define INA219_ADC_SAMPLES_16       0b1100  // Conversion Time: 8.51 ms
#define INA219_ADC_SAMPLES_32       0b1101  // Conversion Time: 17.02 ms
#define INA219_ADC_SAMPLES_64       0b1110  // Conversion Time: 34.05 ms
#define INA219_ADC_SAMPLES_128      0b1111  // Conversion Time: 68.10 ms

#define INA219_MODE_POWER_DOWN              0b000
#define INA219_MODE_SHUNT_TRIGGERED         0b001
#define INA219_MODE_BUS_TRIGGERED           0b010
#define INA219_MODE_SHUNT_BUS_TRIGGERED     0b011
#define INA219_MODE_ADC_OFF                 0b100
#define INA219_MODE_SHUNT_CONTINUOUS        0b101
#define INA219_MODE_BUS_CONTINUOUS          0b110
#define INA219_MODE_SHUNT_BUS_CONTINUOUS    0b111

void PU_enableRecovery();
void PU_enableCamera();
void PU_enableACS();
void PU_disableRecovery();
void PU_disableCamera();
void PU_disableACS();

void Camera_SwitchOn();
void Camera_WifiOn();
void Camera_StartRecording();
void Camera_SwitchOff();
void Camera_WifiOff();
void Camera_StopRecording();
void Camera_SkipDate();

#endif /* PowerUnit_H_ */