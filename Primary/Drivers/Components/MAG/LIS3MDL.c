#include "LIS3MDL.h"
#include "main.h" // ONLY FOR TESTING WITHE LED

HAL_StatusTypeDef MAG_SPI_status;

static void LIS3MDL_Select(void) {
    HAL_GPIO_WritePin(MAG_CS_PORT, MAG_CS_PIN, GPIO_PIN_RESET);
}

static void LIS3MDL_Deselect(void) {
    HAL_GPIO_WritePin(MAG_CS_PORT, MAG_CS_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef MAG_write_reg(uint8_t address, uint8_t len, uint8_t *data) {   
    LIS3MDL_Select();
    uint8_t tx[len+1];
    tx[0] = address | LIS3MDL_SPI_AUTOINC;
    for(int i = 0; i < len; i++) {
        tx[i + 1] = *(data + i); 
    }
    MAG_SPI_status = HAL_SPI_Transmit(&MAG_SPI, tx, len+1, HAL_MAX_DELAY);
    LIS3MDL_Deselect();
    return MAG_SPI_status;
}

HAL_StatusTypeDef MAG_read_reg(uint8_t address, uint8_t len, uint8_t *data) {
    LIS3MDL_Select();
    uint8_t tx[len+1], rx[len+1];
    tx[0] = address | LIS3MDL_SPI_READ | LIS3MDL_SPI_AUTOINC;
    MAG_SPI_status = HAL_SPI_TransmitReceive(&MAG_SPI, tx, rx, len+1, HAL_MAX_DELAY);
    for(int i = 0; i < len; i++) {
        *(data + i) = rx[i + 1];
    }
    LIS3MDL_Deselect();
    return MAG_SPI_status;
}

uint8_t MAG_SelfTest(void) {
    uint8_t Who_Am_I_return = 0;
    MAG_read_reg(LIS3MDL_WHO_AM_I, 1, &Who_Am_I_return);
    if(Who_Am_I_return == LIS3MDL_WHO_AM_I_VAL) return 1;
    else return 0;
}

uint8_t MAG_VerifyDataReady(void) {
    uint8_t status_reg_return = 0;
    MAG_read_reg(LIS3MDL_STATUS_REG, 1, &status_reg_return);
    return status_reg_return;
}

HAL_StatusTypeDef MAG_Init(void) {
    uint8_t tx[3] = { 0x90, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };
    MAG_write_reg(LIS3MDL_CTRL_REG1, 3, tx); // enable temperature sensor and continuous-conversion mode
    MAG_read_reg(LIS3MDL_CTRL_REG1, 3, rx);

    if(rx[0] != 0x90 || rx[2] != 0x00) return HAL_ERROR;
    else return HAL_OK;
}

HAL_StatusTypeDef MAG_ReadSensorData(LIS3MDL_Data_t *data) {
    int16_t temp_raw;
    HAL_StatusTypeDef status;

    uint8_t available = MAG_VerifyDataReady();

    if(available & 0x08) {
        uint8_t rx[6] = { 0 };
        status = MAG_read_reg(LIS3MDL_OUT_X_L, 6, rx);  // read magnetometer data
        if (status != HAL_OK) return status;
        data->field[0] = -(int16_t)(rx[3] << 8 | rx[2]);// field_X
        data->field[1] = (int16_t)(rx[1] << 8 | rx[0]); // field_Y
        data->field[2] = (int16_t)(rx[5] << 8 | rx[4]); // field_Z
    }
    uint8_t rx[2] = { 0 };
    status = MAG_read_reg(LIS3MDL_TEMP_OUT_L, 2, rx);   // read temperature data
    if (status != HAL_OK) return status;
    temp_raw = (int16_t)(rx[1] << 8 | rx[0]);           // calculate temperature
    data->temp = temp_raw / 8. + 25;                    // write temperature data to struct

    return HAL_OK;
}

uint8_t MAG_Offset(int16_t set_x, int16_t set_y, int16_t set_z) {
    uint8_t tx[6] = { 0 };
    uint8_t rx[6] = { 0 };
    tx[0] = (uint8_t)(set_y & 0x00FF);
    tx[1] = (uint8_t)(set_y >> 8);
    tx[2] = (uint8_t)(-set_x & 0x00FF);
    tx[3] = (uint8_t)(-set_x >> 8);
    tx[4] = (uint8_t)(set_z & 0x00FF);
    tx[5] = (uint8_t)(set_z >> 8);
    MAG_write_reg(LIS3MDL_OFFSET_X_REG_L_M, 6, tx);
    MAG_read_reg(LIS3MDL_OFFSET_X_REG_L_M, 6, rx);
    int16_t offset_x = -(int16_t)(rx[3] << 8 | rx[2]); // = -set_x ???
    int16_t offset_y = (int16_t)(rx[1] << 8 | rx[0]);  // = -set_y ???
    int16_t offset_z = (int16_t)(rx[5] << 8 | rx[4]);
    return 1;
}