#include "LSM6DSR.h"

HAL_StatusTypeDef IMU1_SPI_status;

static void activate_imu(void)
{
    HAL_GPIO_WritePin(IMU1_CS_PORT, IMU1_CS_PIN, GPIO_PIN_RESET);
}

static void deactivate_imu(void)
{
    HAL_GPIO_WritePin(IMU1_CS_PORT, IMU1_CS_PIN, GPIO_PIN_SET);
}

void IMU1_write_reg(uint8_t reg, uint8_t data)
{
    activate_imu();
    IMU1_SPI_status = HAL_SPI_Transmit(&IMU1_SPI, &reg, 1, 100);
    IMU1_SPI_status = HAL_SPI_Transmit(&IMU1_SPI, &data, 1, 100);
    deactivate_imu();
}

void IMU1_read_reg(uint8_t address, uint8_t *data)
{
    uint8_t temp_data = 0x03;//0x80|address;
    activate_imu();
    HAL_Delay(2);
    IMU1_SPI_status = HAL_SPI_Transmit(&IMU1_SPI, &temp_data, 1, 100);
    //IMU1_SPI_status = HAL_SPI_Receive(&IMU1_SPI, data, 1, 100);
    deactivate_imu();

    /*uint8_t txData[2], rxData[2];
    txData[0] = 0x80 | address;
    txData[1] = 0xFF;
    activate_imu();
    IMU1_SPI_status =  HAL_SPI_TransmitReceive(&IMU1_SPI, txData, rxData, 2, 100);
    deactivate_imu();*/
}

bool LSM6DSR_SelfTest(void){
    uint8_t Who_Am_I_return = 0;
    IMU1_read_reg(WHO_AM_I_REG, &Who_Am_I_return);
    if(Who_Am_I_return == 0x6B){
        return true;
    }
    else{
        return false;
    }
}