#include "IMUS.h"
//#include "calibration_data.h"
#include "main.h" // ONLY FOR TESTING WITHE LED
#include <stdint.h>
#include <string.h>

#include "armMathAddon.h"
#include "calibration_data.h"

HAL_StatusTypeDef IMU_SPI_status;

static void IMU_WritePin(GPIO_PinState pin, const IMU_Data_t *imu_data) {
    GPIO_TypeDef *imu_cs_port;
    uint16_t imu_cs_pin;

    switch (imu_data->imu) {
        case IMU1:
            imu_cs_port = IMU1_CS_PORT;
            imu_cs_pin = IMU1_CS_PIN;
            break;
        case IMU2:
            imu_cs_port = IMU2_CS_PORT;
            imu_cs_pin = IMU2_CS_PIN;
            break;
    }

    HAL_GPIO_WritePin(imu_cs_port, imu_cs_pin, pin);
}

static void IMU_Select(const IMU_Data_t *imu_data) {
    IMU_WritePin(GPIO_PIN_RESET, imu_data);
}

static void IMU_Deselect(const IMU_Data_t *imu_data) {
    IMU_WritePin(GPIO_PIN_SET, imu_data);
}

HAL_StatusTypeDef IMU_write_reg(uint8_t address, uint8_t len, uint8_t *data, const IMU_Data_t *imu_data) {
    SPI_HandleTypeDef imu_spi;
    switch (imu_data->imu) {
        case IMU1:
            imu_spi = IMU1_SPI;
            break;
        case IMU2:
            imu_spi = IMU2_SPI;
            break;
    }
    IMU_Select(imu_data);

    uint8_t tx[len + 1];
    tx[0] = address;
    for (int i = 0; i < len; i++) {
        tx[i + 1] = *(data + i);
    }
    IMU_SPI_status = HAL_SPI_Transmit(&imu_spi, tx, len + 1, HAL_MAX_DELAY);

    IMU_Deselect(imu_data);
    return IMU_SPI_status;
}

HAL_StatusTypeDef IMU_read_reg(uint8_t address, uint8_t len, uint8_t *data, const IMU_Data_t *imu_data) {
    SPI_HandleTypeDef imu_spi;
    switch (imu_data->imu) {
        case IMU1:
            imu_spi = IMU1_SPI;
            break;
        case IMU2:
            imu_spi = IMU2_SPI;
            break;
    }
    IMU_Select(imu_data);

    uint8_t tx[len + 1], rx[len + 1];
    tx[0] = address | IMU_SPI_READ;
    IMU_SPI_status = HAL_SPI_TransmitReceive(&imu_spi, tx, rx, len + 1, HAL_MAX_DELAY);
    for (int i = 0; i < len; i++) {
        *(data + i) = rx[i + 1];
    }

    IMU_Deselect(imu_data);
    return IMU_SPI_status;
}

static HAL_StatusTypeDef IMU_Init(const IMU_Data_t *imu_data) {
    uint8_t tx[3] = {0x60, 0x60, 0x04};
    uint8_t rx[3] = {0};
    IMU_write_reg(IMU_CTRL1_XL, 3, tx, imu_data);
    IMU_read_reg(IMU_CTRL1_XL, 3, rx, imu_data);

    if (rx[0] != 0x60 || rx[1] != 0x60 || rx[2] != 0x04) return HAL_ERROR;
    return HAL_OK;

    // CTRL7_G, CTRL6_C, CTRL1_OIS,
}

HAL_StatusTypeDef IMU_InitImu(IMU_Data_t *imu_data, IMU imu, GPA_Mega gpa_mega) {
    imu_data->imu = imu;
    HAL_StatusTypeDef status = IMU_Init(imu_data);
    if (status != HAL_OK) return status;

    memcpy(&imu_data->calibration, &CalibrationData[gpa_mega][imu], sizeof(CalibrationData_t));

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 3; ++j) {
            // Fill the history with dummy data.
            // Since the IMU error detection code looks at the acc data history and checks if the data isn't all equal,
            // we need to initialize the different elements with different values.
            imu_data->acc_data_history[i][j] = (float) (i + j);
        }
    }

    return HAL_OK;
}

static void IMU_UpdateHistory(IMU_Data_t *imu_data) {
    // Shift all elements to the right and add the measured data to the front.
    for (int i = 7; i > 0; --i){
        memcpy(
            imu_data->acc_data_history[i],
            imu_data->acc_data_history[i-1],
            sizeof(imu_data->acc_data_history[0])
        );
    }
    memcpy(
        imu_data->acc_data_history[0],
        imu_data->accel,
        sizeof(imu_data->acc_data_history[0])
    );
}

static bool IMU_IsActive(const IMU_Data_t *imu_data) {
    for (int i = 0; i < 8; ++i) {
        int equal_count = 0;
        const float(*history)[3] = imu_data->acc_data_history;
        const float *reference = history[i];

        for (int j = 0; j < 8; ++j) {
            if (j == i) {
                continue;
            }

            equal_count += reference[0] == history[j][0] && reference[1] == history[j][1] && reference[2] == history[j][2];
        }

        if (equal_count >= IMU_INACTIVE_EQUAL_COUNT) {
            return false;
        }
    }

    return true;
}


HAL_StatusTypeDef IMU_Update(IMU_Data_t *imu_data) {
    HAL_StatusTypeDef status = HAL_OK;
    bool imu_ready = IMU_VerifyDataReady(imu_data) & 0x03 == 0x03;

    if (imu_ready) {
        status = IMU_ReadSensorData(imu_data);
        arm_vec3_sub_f32(imu_data->accel, imu_data->calibration.offset, imu_data->accel);
        arm_vec3_element_product_f32(imu_data->accel, imu_data->calibration.scale, imu_data->accel);
    }

    // if imu_ready is false, this will copy the same value to the front and therefore trigger the inactive detection
    IMU_UpdateHistory(imu_data);

    bool active = IMU_IsActive(imu_data);

    imu_data->active = active;
    if (!imu_ready) {
        imu_data->active = false;
    }

    return status;
}

static void average_arrays(float array_1[3], float array_2[3], float average_array[3]) {
    for (int i = 0; i < 3; ++i) {
        float average = 0.5 * (array_1[i] + array_2[i]);
        memcpy(&average_array[i], &average, sizeof(average));
    }
}

void IMU_Average(IMU_Data_t *imu_data_1, IMU_Data_t *imu_data_2, IMU_AverageData_t *average_imu_data) {
    if (imu_data_1->active && imu_data_2->active) {
        float accel_average[3];
        float gyro_average[3];
        average_arrays(imu_data_1->accel, imu_data_2->accel, accel_average);
        average_arrays(imu_data_1->gyro, imu_data_2->gyro, gyro_average);

        memcpy(average_imu_data->accel, accel_average, sizeof(accel_average));
        memcpy(average_imu_data->gyro, gyro_average, sizeof(gyro_average));
    } else if (imu_data_1->active) {
        memcpy(average_imu_data->accel, imu_data_1->accel, sizeof(imu_data_1->accel));
        memcpy(average_imu_data->gyro, imu_data_1->gyro, sizeof(imu_data_1->gyro));
    } else {
        memcpy(average_imu_data->accel, imu_data_2->accel, sizeof(imu_data_2->accel));
        memcpy(average_imu_data->gyro, imu_data_2->gyro, sizeof(imu_data_2->gyro));
    }
}


uint8_t IMU_SelfTest(const IMU_Data_t *imu_data) {
    uint8_t Who_Am_I_return = 0;
    IMU_read_reg(IMU_WHO_AM_I, 1, &Who_Am_I_return, imu_data);
    if (Who_Am_I_return == IMU_WHO_AM_I_VAL) return 1;
    return 0;
}

/* returns 0b00000TGA where 1 = data available for T,G,A sensors */
uint8_t IMU_VerifyDataReady(const IMU_Data_t *imu_data) {
    uint8_t status_reg_return = 0;
    IMU_read_reg(IMU_STATUS_REG, 1, &status_reg_return, imu_data);
    return status_reg_return;
}

HAL_StatusTypeDef IMU_ConfigXL(uint8_t ODR, uint8_t FS, bool LPF2, IMU_Data_t *imu_data) {
    switch(FS) {
        case IMU_FS_XL_2:
            imu_data->xl_fs_lsb = 16384;
            break;
        case IMU_FS_XL_4:
            imu_data->xl_fs_lsb = 8192;
            break;
        case IMU_FS_XL_8:
            imu_data->xl_fs_lsb = 4096;
            break;
        case IMU_FS_XL_16:
            imu_data->xl_fs_lsb = 2048;
            break;
    }
    uint8_t tx[1] = { ODR << 4 | FS << 2 | LPF2 << 1 };
    uint8_t rx[1] = { 0 };
    IMU_write_reg(IMU_CTRL1_XL, 1, tx, imu_data);
    IMU_read_reg(IMU_CTRL1_XL, 1, rx, imu_data);
    if (tx[1] == rx[1]) return HAL_OK;
    return HAL_ERROR;
}

HAL_StatusTypeDef IMU_ConfigG(uint8_t ODR, uint8_t FS, IMU_Data_t *imu_data) {
    switch(FS) {
        case IMU_FS_G_125:
            imu_data->g_fs_lsb = 35;
            break;
        case IMU_FS_G_250:
            imu_data->g_fs_lsb = 70;
            break;
        case IMU_FS_G_500:
            imu_data->g_fs_lsb = 140;
            break;
        case IMU_FS_G_1000:
            imu_data->g_fs_lsb = 280;
            break;
        case IMU_FS_G_2000:
            imu_data->g_fs_lsb = 560;
            break;
        case IMU_FS_G_4000:
            imu_data->g_fs_lsb = 1120;
            break;
    }
    uint8_t tx[1] = { ODR << 4 | FS };
    uint8_t rx[1] = { 0 };
    IMU_write_reg(IMU_CTRL2_G, 1, tx, imu_data);
    IMU_read_reg(IMU_CTRL2_G, 1, rx, imu_data);
    if (tx[1] == rx[1]) return HAL_OK;
    return HAL_ERROR;
}

HAL_StatusTypeDef IMU_ReadSensorData(IMU_Data_t *imu_data) {
    int16_t temp_raw;
    HAL_StatusTypeDef status;

    uint8_t available = IMU_VerifyDataReady(imu_data);

    if (available & 0x01) {
        uint8_t rx[6] = {0};
        status = IMU_read_reg(IMU_OUTX_L_A, 6, rx, imu_data); // read accelerometer data
        if (status != HAL_OK) return status;
        imu_data->accel[0] = (float)(int16_t)(rx[1] << 8 | rx[0]) * 9.81 / imu_data->xl_fs_lsb; // accel_X [m/s²]
        imu_data->accel[1] = (float)(int16_t)(rx[3] << 8 | rx[2]) * 9.81 / imu_data->xl_fs_lsb; // accel_Y [m/s²]
        imu_data->accel[2] = (float)(int16_t)(rx[5] << 8 | rx[4]) * 9.81 / imu_data->xl_fs_lsb; // accel_Z [m/s²]
    }
    if (available & 0x02) {
        uint8_t rx[6] = {0};
        status = IMU_read_reg(IMU_OUTX_L_G, 6, rx, imu_data); // read gyro data
        if (status != HAL_OK) return status;
        imu_data->gyro[0] = (float)(int16_t)(rx[1] << 8 | rx[0]) * imu_data->g_fs_lsb / 8000 * M_PI / 180; // gyro_X [rad/s]
        imu_data->gyro[1] = (float)(int16_t)(rx[3] << 8 | rx[2]) * imu_data->g_fs_lsb / 8000 * M_PI / 180; // gyro_Y [rad/s]
        imu_data->gyro[2] = (float)(int16_t)(rx[5] << 8 | rx[4]) * imu_data->g_fs_lsb / 8000 * M_PI / 180; // gyro_Z [rad/s]
    }
    if (available & 0x04) {
        uint8_t rx[2] = {0};
        status = IMU_read_reg(IMU_OUT_TEMP_L, 2, rx, imu_data); // read temperature data
        if (status != HAL_OK) return status;
        temp_raw = (int16_t) (rx[1] << 8 | rx[0]); // calculate temperature
        imu_data->temp = temp_raw / 256. + 25; // write temperature data to struct
    }

    // IMU 2 is rotated 180° relative to IMU 1
    if (imu_data->imu == IMU2) {
        imu_data->accel[0] *= -1;
        imu_data->accel[1] *= -1;
        imu_data->gyro[0] *= -1;
        imu_data->gyro[1] *= -1;
    }

    return HAL_OK;
}

HAL_StatusTypeDef IMU_SetAccFilterMode(const AccFilterMode filter_mode, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_CTRL8_XL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11111011;
    data |= (uint8_t) filter_mode << 2;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetAccFilterStage(const AccFilterStage filter_stage, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_CTRL1_XL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11111101;
    data |= (uint8_t) filter_stage << 1;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetAccFilterBandwidth(const AccFilterBandwidth filter_bandwidth, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_CTRL8_XL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b00011111;
    data |= (uint8_t) filter_bandwidth << 5;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetGyroLowPassFilter(const bool low_pass_enabled, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_CTRL4_C;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11111101;
    data |= (uint8_t) low_pass_enabled << 1;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetGyroFilterBandwidth(const GyroFilterBandwidth filter_bandwidth, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_CTRL6_C;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11111000;
    data |= (uint8_t) filter_bandwidth << 0;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetInterrupt1Gyro(const bool interrupt_enabled, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_INT1_CTRL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11111101;
    data |= (uint8_t) interrupt_enabled << 1;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetInterrupt1Acc(const bool interrupt_enabled, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_INT1_CTRL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11111110;
    data |= (uint8_t) interrupt_enabled << 0;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetInterrupt2Gyro(const bool interrupt_enabled, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_INT2_CTRL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11111101;
    data |= (uint8_t) interrupt_enabled << 1;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetInterrupt2Acc(const bool interrupt_enabled, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_INT2_CTRL;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11111110;
    data |= (uint8_t) interrupt_enabled << 0;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}

HAL_StatusTypeDef IMU_SetInterruptPins(const InterruptPins interrupt_pins, const IMU_Data_t *imu_data) {
    const uint8_t address = IMU_CTRL4_C;
    uint8_t data = 0;

    HAL_StatusTypeDef status = IMU_read_reg(address, 1, &data, imu_data);
    if (status != HAL_OK) return status;

    data &= 0b11011111;
    data |= (uint8_t) interrupt_pins << 5;

    status = IMU_write_reg(address, 1, &data, imu_data);

    return status;
}
