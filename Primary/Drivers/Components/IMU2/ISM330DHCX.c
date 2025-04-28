#include "ISM330DHCX.h"
#include "main.h" // ONLY FOR TESTING WITHE LED

HAL_StatusTypeDef IMU2_SPI_status;

static void ISM330DHCX_Select(void) {
    HAL_GPIO_WritePin(IMU2_CS_PORT, IMU2_CS_PIN, GPIO_PIN_RESET);
}

static void ISM330DHCX_Deselect(void) {
    HAL_GPIO_WritePin(IMU2_CS_PORT, IMU2_CS_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef IMU2_write_reg(uint8_t address, uint8_t data) {   
    ISM330DHCX_Select();
    uint8_t tx[2];
    tx[0] = address;
    tx[1] = data;
    IMU2_SPI_status = HAL_SPI_Transmit(&IMU2_SPI, tx, 2, HAL_MAX_DELAY);
    ISM330DHCX_Deselect();
    return IMU2_SPI_status;
}

HAL_StatusTypeDef IMU2_read_reg(uint8_t address, uint8_t len, uint8_t *data) {
    ISM330DHCX_Select();
    uint8_t tx[len+1], rx[len+1];
    tx[0] = address | ISM330DHCX_SPI_READ;
    IMU2_SPI_status = HAL_SPI_TransmitReceive(&IMU2_SPI, tx, rx, len+1, HAL_MAX_DELAY);
    for(int i = 0; i < len; i++) {
        *(data + i) = rx[i + 1];
    }
    ISM330DHCX_Deselect();
    return IMU2_SPI_status;
}

uint8_t IMU2_SelfTest(void) {
    uint8_t Who_Am_I_return = 0;
    IMU2_read_reg(ISM330DHCX_WHO_AM_I, 1, &Who_Am_I_return);
    if(Who_Am_I_return == ISM330DHCX_WHO_AM_I_VAL) return 1;
    else return 0;
}

/* returns 0b00000TGA where 1 = data available for T,G,A sensors */
uint8_t IMU2_VerifyDataReady(void) {
    uint8_t status_reg_return = 0;
    IMU2_read_reg(ISM330DHCX_STATUS_REG, 1, &status_reg_return);
    return status_reg_return;
}

HAL_StatusTypeDef IMU2_Init(void) {
    uint8_t rx[3] = { 0 };
    IMU2_write_reg(ISM330DHCX_CTRL1_XL, 0x60);
    IMU2_write_reg(ISM330DHCX_CTRL2_G, 0x60);
    IMU2_write_reg(ISM330DHCX_CTRL3_C, 0x04);

    IMU2_read_reg(ISM330DHCX_CTRL1_XL, 3, rx);

    if(rx[0] != 0x60 || rx[1] != 0x60 || rx[2] != 0x04) return HAL_ERROR;
    else return HAL_OK;

    // CTRL7_G, CTRL6_C, CTRL1_OIS, 
}

HAL_StatusTypeDef IMU2_ReadSensorData(ISM330DHCX_Data_t *data) {
    int16_t temp_raw;
    HAL_StatusTypeDef status;

    uint8_t available = IMU2_VerifyDataReady();

    if(available & 0x01) {
        uint8_t rx[6] = { 0 };
        status = IMU2_read_reg(ISM330DHCX_OUTX_L_A, 6, rx);     // read accelerometer data
        if (status != HAL_OK) return status;
        data->accel[0] = -(int16_t)(rx[1] << 8 | rx[0]);        // accel_X corrected for global coordinate system
        data->accel[1] = -(int16_t)(rx[3] << 8 | rx[2]);        // accel_Y corrected for global coordinate system
        data->accel[2] = (int16_t)(rx[5] << 8 | rx[4]);         // accel_Z
    }
    if(available & 0x02) {
        uint8_t rx[6] = { 0 };
        status = IMU2_read_reg(ISM330DHCX_OUTX_L_G, 6, rx);     // read gyro data
        if (status != HAL_OK) return status;
        data->gyro[0] = -(int16_t)(rx[1] << 8 | rx[0]);         // gyro_X corrected for global coordinate system
        data->gyro[1] = -(int16_t)(rx[3] << 8 | rx[2]);         // gyro_Y corrected for global coordinate system
        data->gyro[2] = (int16_t)(rx[5] << 8 | rx[4]);          // gyro_Z
    }
    if(available & 0x04) {
        uint8_t rx[2] = { 0 };
        status = IMU2_read_reg(ISM330DHCX_OUT_TEMP_L, 2, rx);   // read temperature data
        if (status != HAL_OK) return status;
        temp_raw = (int16_t)(rx[1] << 8 | rx[0]);               // calculate temperature
        data->temp = temp_raw / 256. + 25;                      // write temperature data to struct
    }

    return HAL_OK;
}