#include "PowerUnit.h"

HAL_StatusTypeDef AS6500_I2C_status;

float current_LSB;
float power_LSB;

// Write 16-bit value to INA219 register
HAL_StatusTypeDef INA219_writeRegister(uint8_t reg, uint16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;            // Register address
    buffer[1] = (data >> 8);    // MSB
    buffer[2] = (data & 0xFF);  // LSB

    return HAL_I2C_Master_Transmit(&hi2c2, INA219_I2C_ADDR, buffer, 3, HAL_MAX_DELAY);
}

// Read 16-bit value from INA219 register
HAL_StatusTypeDef INA219_readRegister(uint8_t start_reg, uint16_t *data) {
    uint8_t buffer[2] = {0};
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&hi2c2, INA219_I2C_ADDR, &start_reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(&hi2c2, INA219_I2C_ADDR, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    return status;
}

HAL_StatusTypeDef INA219_reset() {
    return INA219_writeRegister(INA219_CONFIG_REG, INA219_RESET_CMD);
}

void INA219_setBusVoltageRange(uint8_t range) {
    uint16_t data = 0;
    INA219_readRegister(INA219_CONFIG_REG, &data);

    data &= 0x5FFF;
    data |= (uint16_t)range << 13;
    INA219_writeRegister(INA219_CONFIG_REG, data);
}

// set PGA gain and range; Gain = 2^(-range)
void INA219_setShuntVoltageRange(uint8_t range) {
    uint16_t data = 0;
    INA219_readRegister(INA219_CONFIG_REG, &data);

    data &= 0x67FF;
    data |= (uint16_t)range << 11;
    INA219_writeRegister(INA219_CONFIG_REG, data);
}

// set Bus ADC resolution or number of samples
void INA219_setBusADC(uint8_t mode) {
    uint16_t data = 0;
    INA219_readRegister(INA219_CONFIG_REG, &data);

    data &= 0x787F;
    data |= (uint16_t)mode << 7;
    INA219_writeRegister(INA219_CONFIG_REG, data);
}

// set Shunt ADC resolution or number of samples
void INA219_setShuntADC(uint8_t mode) {
    uint16_t data = 0;
    INA219_readRegister(INA219_CONFIG_REG, &data);

    data &= 0x7F87;
    data |= (uint16_t)mode << 3;
    INA219_writeRegister(INA219_CONFIG_REG, data);
}

void INA219_setOperatingMode(uint8_t mode) {
    uint16_t data = 0;
    INA219_readRegister(INA219_CONFIG_REG, &data);

    data &= 0xFFF8;
    data |= (uint16_t)mode;
    INA219_writeRegister(INA219_CONFIG_REG, data);
}

void INA219_setCurrentCalibration(float max_current, float shunt_resistance) {
    current_LSB = max_current / 32768.f;
    power_LSB = current_LSB * 20.f;
    
    uint16_t calibration_value = round(0.04096 / current_LSB / shunt_resistance);

    if(calibration_value > 65535) {
        calibration_value = 65535.f;
        current_LSB = 0.04096 / calibration_value / shunt_resistance;
    }

    INA219_writeRegister(INA219_CALIBRATION_REG, calibration_value);
}

void INA219_readShuntVoltage(float *voltage) {
    uint16_t data = 0;
    INA219_readRegister(INA219_CONFIG_REG, &data);
    uint8_t gain = (data & 0x1800) >> 11;

    INA219_readRegister(INA219_SHUNT_VOLTAGE_REG, &data);

    int16_t data_signed = 0;
    switch(gain) {
        case 0b00:
            data_signed = ((int16_t)(data << 3));
            *voltage = (float)data_signed / 800000.f;
            break;
        case 0b01:
            data_signed = ((int16_t)(data << 2));
            *voltage = (float)data_signed / 400000.f;
            break;
        case 0b10:
            data_signed = ((int16_t)(data << 1));
            *voltage = (float)data_signed / 200000.f;
            break;
        case 0b11:
            data_signed = ((int16_t)(data));
            *voltage = (float)data_signed / 100000.f;
            break;
    }
}

void INA219_readBusVoltage(float *voltage) {
    uint16_t data = 0;
    INA219_readRegister(INA219_BUS_VOLTAGE_REG, &data);
    data = data >> 3;

    *voltage = 0.004f * data;
}

void INA219_readPower(float *power) {
    uint16_t data = 0;
    INA219_readRegister(INA219_POWER_REG, &data);

    *power = (float)data * power_LSB;
}

void INA219_readCurrent(float *current) {
    uint16_t data = 0;
    int16_t data_signed;
    INA219_readRegister(INA219_CURRENT_REG, &data);
    data_signed = (int16_t)data;

    *current = (float)data_signed * current_LSB;
}

void PU_enableRecovery() {
    HAL_GPIO_WritePin(Recovery_GPIO_Port, Recovery_Pin, 1);
}

void PU_enableCamera() {
    HAL_GPIO_WritePin(CAMS_GPIO_Port, CAMS_Pin, 1);
}

void PU_enableACS() {
    HAL_GPIO_WritePin(ACS_GPIO_Port, ACS_Pin, 1);
}

void PU_disableRecovery() {
    HAL_GPIO_WritePin(Recovery_GPIO_Port, Recovery_Pin, 0);
}

void PU_disableCamera() {
    HAL_GPIO_WritePin(CAMS_GPIO_Port, CAMS_Pin, 0);
}

void PU_disableACS() {
    HAL_GPIO_WritePin(ACS_GPIO_Port, ACS_Pin, 0);
}

void Camera_SwitchOn() {
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 0);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 1);
}

void Camera_WifiOn() {
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 1);
}

void Camera_StartRecording() {
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
}

void Camera_SwitchOff() {
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 0);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 1);
}

void Camera_WifiOff() {
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
}

void Camera_StopRecording() {
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
}

void Camera_SkipDate() {
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
}