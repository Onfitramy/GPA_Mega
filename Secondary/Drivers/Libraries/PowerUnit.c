#include "PowerUnit.h"

HAL_StatusTypeDef INA219_I2C_status;

float current_LSB;
float power_LSB;

camera_status_t CameraStatus;

// Write 16-bit value to INA219 register
HAL_StatusTypeDef INA219_writeRegister(uint8_t reg, uint16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;            // Register address
    buffer[1] = (data >> 8);    // MSB
    buffer[2] = (data & 0xFF);  // LSB

    INA219_I2C_status = HAL_I2C_Master_Transmit(&hi2c2, INA219_I2C_ADDR, buffer, 3, HAL_MAX_DELAY);
    return INA219_I2C_status;
}

// Read 16-bit value from INA219 register
HAL_StatusTypeDef INA219_readRegister(uint8_t start_reg, uint16_t *data) {
    uint8_t buffer[2] = {0};

    INA219_I2C_status = HAL_I2C_Master_Transmit(&hi2c2, INA219_I2C_ADDR, &start_reg, 1, HAL_MAX_DELAY);
    if (INA219_I2C_status != HAL_OK) return INA219_I2C_status;

    INA219_I2C_status = HAL_I2C_Master_Receive(&hi2c2, INA219_I2C_ADDR, buffer, 2, HAL_MAX_DELAY);
    if (INA219_I2C_status != HAL_OK) return INA219_I2C_status;

    *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    return INA219_I2C_status;
}

HAL_StatusTypeDef INA219_reset() {
    return INA219_writeRegister(INA219_CONFIG_REG, INA219_RESET_CMD);
}

HAL_StatusTypeDef INA219_setBusVoltageRange(uint8_t range) {
    uint16_t data = 0;
    if (INA219_readRegister(INA219_CONFIG_REG, &data) != HAL_OK) return INA219_I2C_status;

    data &= 0x5FFF;
    data |= (uint16_t)range << 13;
    return INA219_writeRegister(INA219_CONFIG_REG, data);
}

// set PGA gain and range; Gain = 2^(-range)
HAL_StatusTypeDef INA219_setShuntVoltageRange(uint8_t range) {
    uint16_t data = 0;
    if (INA219_readRegister(INA219_CONFIG_REG, &data) != HAL_OK) return INA219_I2C_status;

    data &= 0x67FF;
    data |= (uint16_t)range << 11;
    return INA219_writeRegister(INA219_CONFIG_REG, data);
}

// set Bus ADC resolution or number of samples
HAL_StatusTypeDef INA219_setBusADC(uint8_t mode) {
    uint16_t data = 0;
    if (INA219_readRegister(INA219_CONFIG_REG, &data) != HAL_OK) return INA219_I2C_status;

    data &= 0x787F;
    data |= (uint16_t)mode << 7;
    return INA219_writeRegister(INA219_CONFIG_REG, data);
}

// set Shunt ADC resolution or number of samples
HAL_StatusTypeDef INA219_setShuntADC(uint8_t mode) {
    uint16_t data = 0;
    if (INA219_readRegister(INA219_CONFIG_REG, &data) != HAL_OK) return INA219_I2C_status;

    data &= 0x7F87;
    data |= (uint16_t)mode << 3;
    return INA219_writeRegister(INA219_CONFIG_REG, data);
}

HAL_StatusTypeDef INA219_setOperatingMode(uint8_t mode) {
    uint16_t data = 0;
    if (INA219_readRegister(INA219_CONFIG_REG, &data) != HAL_OK) return INA219_I2C_status;

    data &= 0xFFF8;
    data |= (uint16_t)mode;
    return INA219_writeRegister(INA219_CONFIG_REG, data);
}

HAL_StatusTypeDef INA219_setCurrentCalibration(float max_current, float shunt_resistance) {
    current_LSB = max_current / 32768.f;
    power_LSB = current_LSB * 20.f;
    
    uint16_t calibration_value = round(0.04096 / current_LSB / shunt_resistance);

    if (calibration_value > 65535) {
        calibration_value = 65535.f;
        current_LSB = 0.04096 / calibration_value / shunt_resistance;
    }

    return INA219_writeRegister(INA219_CALIBRATION_REG, calibration_value);
}

HAL_StatusTypeDef INA219_readShuntVoltage(float *voltage) {
    uint16_t data = 0;
    if (INA219_readRegister(INA219_CONFIG_REG, &data) != HAL_OK) return INA219_I2C_status;
    uint8_t gain = (data & 0x1800) >> 11;

    if (INA219_readRegister(INA219_SHUNT_VOLTAGE_REG, &data) != HAL_OK) return INA219_I2C_status;

    int16_t data_signed = 0;
    switch (gain) {
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

    return HAL_OK;
}

HAL_StatusTypeDef INA219_readBusVoltage(float *voltage) {
    uint16_t data = 0;
    if (INA219_readRegister(INA219_BUS_VOLTAGE_REG, &data) != HAL_OK) return INA219_I2C_status;
    data = data >> 3;

    *voltage = 0.004f * data;

    return HAL_OK;
}

HAL_StatusTypeDef INA219_readPower(float *power) {
    uint16_t data = 0;
    if (INA219_readRegister(INA219_POWER_REG, &data) != HAL_OK) return INA219_I2C_status;

    *power = (float)data * power_LSB;

    return HAL_OK;
}

HAL_StatusTypeDef INA219_readCurrent(float *current) {
    uint16_t data = 0;
    int16_t data_signed;
    if (INA219_readRegister(INA219_CURRENT_REG, &data) != HAL_OK) return INA219_I2C_status;
    data_signed = (int16_t)data;

    *current = (float)data_signed * current_LSB;

    return HAL_OK;
}

HAL_StatusTypeDef INA219_init() {
    if (INA219_setBusADC(INA219_BUS_RANGE_16V) != HAL_OK) return INA219_I2C_status;
    if (INA219_setShuntADC(INA219_ADC_MODE_12_BIT) != HAL_OK) return INA219_I2C_status;
    if (INA219_setOperatingMode(INA219_MODE_SHUNT_BUS_CONTINUOUS) != HAL_OK) return INA219_I2C_status;
    if (INA219_setShuntVoltageRange(INA219_SHUNT_RANGE_40mV) != HAL_OK) return INA219_I2C_status;
    if (INA219_setCurrentCalibration(MAX_CURRENT, SHUNT_RESISTANCE) != HAL_OK) return INA219_I2C_status;

    return HAL_OK;
}

HAL_StatusTypeDef INA219_readAll(health_t *health_struct) {
    if(INA219_readBusVoltage(&health_struct->voltage.bus_pu_bat) != HAL_OK) return INA219_I2C_status;
    if(INA219_readShuntVoltage(&health_struct->voltage.shunt_pu) != HAL_OK) return INA219_I2C_status;
    if(INA219_readPower(&health_struct->power.out_pu) != HAL_OK) return INA219_I2C_status;
    if(INA219_readCurrent(&health_struct->current.out_pu) != HAL_OK) return INA219_I2C_status;

    return HAL_OK;
}

bool INA219_SelfTest() {
    if (INA219_init() == HAL_OK) return 1;
    return 0;
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
    if (CameraStatus.powered) return;
    if (tim7_task != none) return;

    CameraStatus.powered = true;

    // first action
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 0);

    // initial delay
    tim7_target_ms = 2000;
    tim7_task = cam_toggle_power;
    HAL_TIM_Base_Start_IT(&htim7);
}

void Camera_WifiOn() {
    if (!CameraStatus.powered) return;
    if (CameraStatus.wifi) return;
    if (tim7_task != none) return;

    CameraStatus.wifi = true;

    // first action
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 0);

    // initial delay
    tim7_target_ms = 100;
    tim7_task = cam_wifi_on;
    HAL_TIM_Base_Start_IT(&htim7);
}

void Camera_StartRecording() {
    if (!CameraStatus.powered) return;
    if (CameraStatus.recording) return;
    if (tim7_task != none) return;

    CameraStatus.recording = true;

    // first action
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);

    // initial delay
    tim7_target_ms = 100;
    tim7_task = cam_record;
    HAL_TIM_Base_Start_IT(&htim7);
}

void Camera_SwitchOff() {
    if (!CameraStatus.powered) return;
    if (tim7_task != none) return;

    CameraStatus.powered = false;

    // first action
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 0);

    // initial delay
    tim7_target_ms = 2000;
    tim7_task = cam_toggle_power;
    HAL_TIM_Base_Start_IT(&htim7);
}

void Camera_WifiOff() {
    if (!CameraStatus.powered) return;
    if (!CameraStatus.wifi) return;
    if (tim7_task != none) return;

    CameraStatus.wifi = false;

    // first action
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 0);

    // initial delay
    tim7_target_ms = 100;
    tim7_task = cam_wifi_off;
    HAL_TIM_Base_Start_IT(&htim7);
}

void Camera_StopRecording() {
    if (!CameraStatus.powered) return;
    if (!CameraStatus.recording) return;
    if (tim7_task != none) return;

    CameraStatus.recording = false;

    // first action
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    
    // initial delay
    tim7_target_ms = 100;
    tim7_task = cam_record;
    HAL_TIM_Base_Start_IT(&htim7);
}

void Camera_SkipDate() {
    if (!CameraStatus.powered) return;
    if (CameraStatus.recording) return;
    if (tim7_task != none) return;

    // first action
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);

    // initial delay
    tim7_target_ms = 100;
    tim7_task = cam_skip_date;
    HAL_TIM_Base_Start_IT(&htim7);
}