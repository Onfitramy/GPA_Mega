#include "ISM330DHCX.h"
//#include "calibration_data.h"
#include "main.h" // ONLY FOR TESTING WITHE LED
#include <stdint.h>

HAL_StatusTypeDef IMU2_SPI_status;

int16_t IMU2XL_FS_LSB = 16384;
int16_t IMU2G_FS_LSB = 35;

static void ISM330DHCX_Select(void) {
    HAL_GPIO_WritePin(IMU2_CS_PORT, IMU2_CS_PIN, GPIO_PIN_RESET);
}

static void ISM330DHCX_Deselect(void) {
    HAL_GPIO_WritePin(IMU2_CS_PORT, IMU2_CS_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef IMU2_write_reg(uint8_t address, uint8_t len, uint8_t *data) {
    ISM330DHCX_Select();
    uint8_t tx[len + 1];
    tx[0] = address;
    for (int i = 0; i < len; i++) {
        tx[i + 1] = *(data + i);
    }
    IMU2_SPI_status = HAL_SPI_Transmit(&IMU2_SPI, tx, len + 1, HAL_MAX_DELAY);
    ISM330DHCX_Deselect();
    return IMU2_SPI_status;
}

HAL_StatusTypeDef IMU2_read_reg(uint8_t address, uint8_t len, uint8_t *data) {
    ISM330DHCX_Select();
    uint8_t tx[len + 1], rx[len + 1];
    tx[0] = address | ISM330DHCX_SPI_READ;
    IMU2_SPI_status = HAL_SPI_TransmitReceive(&IMU2_SPI, tx, rx, len + 1, HAL_MAX_DELAY);
    for (int i = 0; i < len; i++) {
        *(data + i) = rx[i + 1];
    }
    ISM330DHCX_Deselect();
    return IMU2_SPI_status;
}

uint8_t IMU2_SelfTest(void) {
    uint8_t Who_Am_I_return = 0;
    IMU2_read_reg(ISM330DHCX_WHO_AM_I, 1, &Who_Am_I_return);
    if (Who_Am_I_return == ISM330DHCX_WHO_AM_I_VAL) return 1;
    else return 0;
}

/* returns 0b00000TGA where 1 = data available for T,G,A sensors */
uint8_t IMU2_VerifyDataReady(void) {
    uint8_t status_reg_return = 0;
    IMU2_read_reg(ISM330DHCX_STATUS_REG, 1, &status_reg_return);
    return status_reg_return;
}

HAL_StatusTypeDef IMU2_Init(void) {
    uint8_t tx[3] = {0x60, 0x60, 0x04};
    uint8_t rx[3] = {0};
    IMU2_write_reg(ISM330DHCX_CTRL1_XL, 3, tx);
    IMU2_read_reg(ISM330DHCX_CTRL1_XL, 3, rx);

    if (rx[0] != 0x60 || rx[1] != 0x60 || rx[2] != 0x04) return HAL_ERROR;
    else return HAL_OK;

    // CTRL7_G, CTRL6_C, CTRL1_OIS,
}

HAL_StatusTypeDef IMU2_ConfigXL(uint8_t ODR, uint8_t FS, bool LPF2) {
    switch(FS) {
        case ISM330DHCX_FS_XL_2:    IMU2XL_FS_LSB = 16384; break;
        case ISM330DHCX_FS_XL_4:    IMU2XL_FS_LSB = 8192;  break;
        case ISM330DHCX_FS_XL_8:    IMU2XL_FS_LSB = 4096;  break;
        case ISM330DHCX_FS_XL_16:   IMU2XL_FS_LSB = 2048;  break;
    }
    uint8_t tx[1] = { ODR << 4 | FS << 2 | LPF2 << 1 };
    uint8_t rx[1] = { 0 };
    IMU2_write_reg(ISM330DHCX_CTRL1_XL, 1, tx);
    IMU2_read_reg(ISM330DHCX_CTRL1_XL, 1, rx);
    if (tx[1] == rx[1]) return HAL_OK;
    else return HAL_ERROR;
}

HAL_StatusTypeDef IMU2_ConfigG(uint8_t ODR, uint8_t FS) {
    switch(FS) {
        case ISM330DHCX_FS_G_125:  IMU2G_FS_LSB = 35;   break;
        case ISM330DHCX_FS_G_250:  IMU2G_FS_LSB = 70;   break;
        case ISM330DHCX_FS_G_500:  IMU2G_FS_LSB = 140;  break;
        case ISM330DHCX_FS_G_1000: IMU2G_FS_LSB = 280;  break;
        case ISM330DHCX_FS_G_2000: IMU2G_FS_LSB = 560;  break;
        case ISM330DHCX_FS_G_4000: IMU2G_FS_LSB = 1120; break;
    }
    uint8_t tx[1] = { ODR << 4 | FS };
    uint8_t rx[1] = { 0 };
    IMU2_write_reg(ISM330DHCX_CTRL2_G, 1, tx);
    IMU2_read_reg(ISM330DHCX_CTRL2_G, 1, rx);
    if (tx[1] == rx[1]) return HAL_OK;
    else return HAL_ERROR;
}

HAL_StatusTypeDef IMU2_ReadSensorData(ISM330DHCX_Data_t *data) {
    int16_t temp_raw;
    HAL_StatusTypeDef status;

    uint8_t available = IMU2_VerifyDataReady();

    if (available & 0x01) {
        uint8_t rx[6] = {0};
        status = IMU2_read_reg(ISM330DHCX_OUTX_L_A, 6, rx); // read accelerometer data
        if (status != HAL_OK) return status;
        data->accel[0] = (float)((int16_t)(rx[1] << 8 | rx[0])) * 9.81 / IMU2XL_FS_LSB; // accel_X [m/s²]
        data->accel[1] = (float)((int16_t)(rx[3] << 8 | rx[2])) * 9.81 / IMU2XL_FS_LSB; // accel_Y [m/s²]
        data->accel[2] = (float)((int16_t)(rx[5] << 8 | rx[4])) * 9.81 / IMU2XL_FS_LSB; // accel_Z [m/s²]
    }
    if (available & 0x02) {
        uint8_t rx[6] = {0};
        status = IMU2_read_reg(ISM330DHCX_OUTX_L_G, 6, rx); // read gyro data
        if (status != HAL_OK) return status;
        data->gyro[0] = (float)((int16_t)(rx[1] << 8 | rx[0])) * IMU2G_FS_LSB / 8000 * M_PI / 180; // gyro_X [rad/s]
        data->gyro[1] = (float)((int16_t)(rx[3] << 8 | rx[2])) * IMU2G_FS_LSB / 8000 * M_PI / 180; // gyro_Y [rad/s]
        data->gyro[2] = (float)((int16_t)(rx[5] << 8 | rx[4])) * IMU2G_FS_LSB / 8000 * M_PI / 180; // gyro_Z [rad/s]
    }
    if (available & 0x04) {
        uint8_t rx[2] = {0};
        status = IMU2_read_reg(ISM330DHCX_OUT_TEMP_L, 2, rx); // read temperature data
        // TODO: Check status for all sensors
        // TODO: Log HAL_FAIL
        // TODO: Change detection for sensor switch
        if (status != HAL_OK) return status;
        temp_raw = (int16_t) (rx[1] << 8 | rx[0]); // calculate temperature
        data->temp = temp_raw / 256. + 25; // write temperature data to struct
    }

    return HAL_OK;
}

HAL_StatusTypeDef IMU2_SetAccFilterMode(AccFilterMode filter_mode) {
    const uint8_t address = ISM330DHCX_CTRL8_XL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU2_read_reg(address, 1, &data);
    if (status != HAL_OK) return status;

    data &= 0b11111011;
    data |= (uint8_t) filter_mode << 2;

    status = IMU2_write_reg(address, 1, &data);

    return status;
}

HAL_StatusTypeDef IMU2_SetAccFilterStage(AccFilterStage filter_stage) {
    const uint8_t address = ISM330DHCX_CTRL1_XL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU2_read_reg(address, 1, &data);
    if (status != HAL_OK) return status;

    data &= 0b11111101;
    data |= (uint8_t) filter_stage << 1;

    status = IMU2_write_reg(address, 1, &data);

    return status;
}

HAL_StatusTypeDef IMU2_SetAccFilterBandwidth(AccFilterBandwidth filter_bandwidth) {
    const uint8_t address = ISM330DHCX_CTRL8_XL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU2_read_reg(address, 1, &data);
    if (status != HAL_OK) return status;

    data &= 0b00011111;
    data |= (uint8_t) filter_bandwidth << 5;

    status = IMU2_write_reg(address, 1, &data);

    return status;
}

HAL_StatusTypeDef IMU2_SetGyroFilterMode(GyroLowPassMode low_pass_mode) {
    const uint8_t address = ISM330DHCX_CTRL4_C;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU2_read_reg(address, 1, &data);
    if (status != HAL_OK) return status;

    data &= 0b11111101;
    data |= (uint8_t) low_pass_mode << 1;

    status = IMU2_write_reg(address, 1, &data);

    return status;
}

HAL_StatusTypeDef IMU2_SetGyroFilterBandwidth(GyroFilterBandwidth filter_bandwidth) {
    const uint8_t address = ISM330DHCX_CTRL6_C;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU2_read_reg(address, 1, &data);
    if (status != HAL_OK) return status;

    data &= 0b11111000;
    data |= (uint8_t) filter_bandwidth << 0;

    status = IMU2_write_reg(address, 1, &data);

    return status;
}
