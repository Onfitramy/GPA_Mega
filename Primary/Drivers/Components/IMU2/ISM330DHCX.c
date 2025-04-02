#include "ISM330DHCX.h"

HAL_StatusTypeDef IMU2_SPI_status;

static void activate_imu(void)
{
    HAL_GPIO_WritePin(IMU2_CS_PORT, IMU2_CS_PIN, GPIO_PIN_RESET);
}

static void deactivate_imu(void)
{
    HAL_GPIO_WritePin(IMU2_CS_PORT, IMU2_CS_PIN, GPIO_PIN_SET);
}

void IMU2_write_reg(uint8_t reg, uint8_t data)
{
    activate_imu();
    IMU2_SPI_status = HAL_SPI_Transmit(&IMU2_SPI, &reg, 1, 100);
    IMU2_SPI_status = HAL_SPI_Transmit(&IMU2_SPI, &data, 1, 100);
    deactivate_imu();
}

void IMU2_read_reg(uint8_t address, uint8_t *data)
{
    /* temp_data = 0x80|address;
    activate_imu();
    IMU2_SPI_status = HAL_SPI_Transmit(&IMU2_SPI, &temp_data, 1, 100);
    IMU2_SPI_status = HAL_SPI_Receive(&IMU2_SPI, data, 1, 100);
    deactivate_imu();*/

    uint8_t txData[2], rxData[2];
    txData[0] = 0x80 | address;
    txData[1] = 0xFF;
    activate_imu();
    IMU2_SPI_status = HAL_SPI_TransmitReceive(&IMU2_SPI, txData, rxData, 2, 100);
    deactivate_imu();
    if(IMU2_SPI_status == HAL_OK & rxData[1] == 0x6B){
        *data = rxData[1];
        HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
    }
    else{
        *data = 0;
    }
}

bool ISM330DHCX_SelfTest(void){
    uint8_t Who_Am_I_return = 0;
    IMU2_read_reg(WHO_AM_I_REG, &Who_Am_I_return);
    if(Who_Am_I_return == 0x6B){
        return true;
    }
    else{
        return false;
    }
}