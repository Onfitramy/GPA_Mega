#include "LSM6DSR.h"
//#include "calibration_data.h"
#include "main.h" // ONLY FOR TESTING WITHE LED

HAL_StatusTypeDef IMU1_SPI_status;

int16_t IMU1XL_FS_LSB = 16384;
int16_t IMU1G_FS_LSB = 35;

static void LSM6DSR_Select(void) {
    HAL_GPIO_WritePin(IMU1_CS_PORT, IMU1_CS_PIN, GPIO_PIN_RESET);
}

static void LSM6DSR_Deselect(void) {
    HAL_GPIO_WritePin(IMU1_CS_PORT, IMU1_CS_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef IMU1_write_reg(uint8_t address, uint8_t len, uint8_t *data) {   
    LSM6DSR_Select();
    uint8_t tx[len+1];
    tx[0] = address;
    for(int i = 0; i < len; i++) {
        tx[i + 1] = *(data + i); 
    }
    IMU1_SPI_status = HAL_SPI_Transmit(&IMU1_SPI, tx, len+1, HAL_MAX_DELAY);
    LSM6DSR_Deselect();
    return IMU1_SPI_status;
}

HAL_StatusTypeDef IMU1_read_reg(uint8_t address, uint8_t len, uint8_t *data) {
    LSM6DSR_Select();
    uint8_t tx[len+1], rx[len+1];
    tx[0] = address | LSM6DSR_SPI_READ;
    IMU1_SPI_status = HAL_SPI_TransmitReceive(&IMU1_SPI, tx, rx, len+1, HAL_MAX_DELAY);
    for(int i = 0; i < len; i++) {
        *(data + i) = rx[i + 1];
    }
    LSM6DSR_Deselect();
    return IMU1_SPI_status;
}

uint8_t IMU1_SelfTest(void) {
    uint8_t Who_Am_I_return = 0;
    IMU1_read_reg(LSM6DSR_WHO_AM_I, 1, &Who_Am_I_return);
    if(Who_Am_I_return == LSM6DSR_WHO_AM_I_VAL) return 1;
    else return 0;
}

/* returns 0b00000TGA where 1 = data available for T,G,A sensors */
uint8_t IMU1_VerifyDataReady(void) {
    uint8_t status_reg_return = 0;
    IMU1_read_reg(LSM6DSR_STATUS_REG, 1, &status_reg_return);
    return status_reg_return;
}

HAL_StatusTypeDef IMU1_Init(void) {
    uint8_t tx[3] = { 0x60, 0x60, 0x04 };
    uint8_t rx[3] = { 0 };
    IMU1_write_reg(LSM6DSR_CTRL1_XL, 3, tx);
    IMU1_read_reg(LSM6DSR_CTRL1_XL, 3, rx);

    if(rx[0] != 0x60 || rx[1] != 0x60 || rx[2] != 0x04) return HAL_ERROR;
    else return HAL_OK;

    // CTRL7_G, CTRL6_C, CTRL1_OIS, 
}

HAL_StatusTypeDef IMU1_ConfigXL(uint8_t ODR, uint8_t FS, bool LPF2) {
    switch(FS) {
        case LSM6DSR_FS_XL_2:  IMU1XL_FS_LSB = 16384; break;
        case LSM6DSR_FS_XL_4:  IMU1XL_FS_LSB = 8192;  break;
        case LSM6DSR_FS_XL_8:  IMU1XL_FS_LSB = 4096;  break;
        case LSM6DSR_FS_XL_16: IMU1XL_FS_LSB = 2048;  break;
    }
    uint8_t tx[1] = { ODR << 4 | FS << 2 | LPF2 << 1 };
    uint8_t rx[1] = { 0 };
    IMU1_write_reg(LSM6DSR_CTRL1_XL, 1, tx);
    IMU1_read_reg(LSM6DSR_CTRL1_XL, 1, rx);
    if(tx[1] == rx[1]) return HAL_OK;
    else return HAL_ERROR;
}

HAL_StatusTypeDef IMU1_ConfigG(uint8_t ODR, uint8_t FS) {
    switch(FS) {
        case LSM6DSR_FS_G_125:  IMU1G_FS_LSB = 35;   break;
        case LSM6DSR_FS_G_250:  IMU1G_FS_LSB = 70;   break;
        case LSM6DSR_FS_G_500:  IMU1G_FS_LSB = 140;  break;
        case LSM6DSR_FS_G_1000: IMU1G_FS_LSB = 280;  break;
        case LSM6DSR_FS_G_2000: IMU1G_FS_LSB = 560;  break;
        case LSM6DSR_FS_G_4000: IMU1G_FS_LSB = 1120; break;
    }
    uint8_t tx[1] = { ODR << 4 | FS };
    uint8_t rx[1] = { 0 };
    IMU1_write_reg(LSM6DSR_CTRL2_G, 1, tx);
    IMU1_read_reg(LSM6DSR_CTRL2_G, 1, rx);
    if(tx[1] == rx[1]) return HAL_OK;
    else return HAL_ERROR;
}

HAL_StatusTypeDef IMU1_ReadSensorData(LSM6DSR_Data_t *data) {
    int16_t temp_raw;
    HAL_StatusTypeDef status;

    uint8_t available = IMU1_VerifyDataReady();

    if(available & 0x01) {
        uint8_t rx[6] = { 0 };
        status = IMU1_read_reg(LSM6DSR_OUTX_L_A, 6, rx);    // read accelerometer data
        if (status != HAL_OK) return status;
        data->accel[0] = (float)((int16_t)(rx[1] << 8 | rx[0])) * 9.81 / IMU1XL_FS_LSB; // accel_X [m/s²]
        data->accel[1] = (float)((int16_t)(rx[3] << 8 | rx[2])) * 9.81 / IMU1XL_FS_LSB; // accel_Y [m/s²]
        data->accel[2] = (float)((int16_t)(rx[5] << 8 | rx[4])) * 9.81 / IMU1XL_FS_LSB; // accel_Z [m/s²]
    }
    if(available & 0x02) {
        uint8_t rx[6] = { 0 };
        status = IMU1_read_reg(LSM6DSR_OUTX_L_G, 6, rx);    // read gyro data
        if (status != HAL_OK) return status;
        data->gyro[0] = (float)((int16_t)(rx[1] << 8 | rx[0])) * IMU1G_FS_LSB / 8000; // gyro_X [dps]
        data->gyro[1] = (float)((int16_t)(rx[3] << 8 | rx[2])) * IMU1G_FS_LSB / 8000; // gyro_Y [dps]
        data->gyro[2] = (float)((int16_t)(rx[5] << 8 | rx[4])) * IMU1G_FS_LSB / 8000; // gyro_Z [dps]
    }
    if(available & 0x04) {
        uint8_t rx[2] = { 0 };
        status = IMU1_read_reg(LSM6DSR_OUT_TEMP_L, 2, rx);  // read temperature data
        if (status != HAL_OK) return status;
        temp_raw = (int16_t)(rx[1] << 8 | rx[0]);           // calculate temperature
        data->temp = temp_raw / 256. + 25;                  // write temperature data to struct
    }

    return HAL_OK;
}