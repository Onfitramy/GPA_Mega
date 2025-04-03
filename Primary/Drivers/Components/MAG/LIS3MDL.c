#include "LIS3MDL.h"

HAL_StatusTypeDef MAG_SPI_status;

static void activate_mag(void)
{
    HAL_GPIO_WritePin(MAG_CS_PORT, MAG_CS_PIN, GPIO_PIN_RESET);
}

static void deactivate_mag(void)
{
    HAL_GPIO_WritePin(MAG_CS_PORT, MAG_CS_PIN, GPIO_PIN_SET);
}

void MAG_write_reg(uint8_t reg, uint8_t data)
{
    activate_mag();
    MAG_SPI_status = HAL_SPI_Transmit(&MAG_SPI, &reg, 1, 100);
    MAG_SPI_status = HAL_SPI_Transmit(&MAG_SPI, &data, 1, 100);
    deactivate_mag();
}

void MAG_read_reg(uint8_t address, uint8_t *data)
{
    /* temp_data = 0x80|address;
    activate_mag();
    MAG_SPI_status = HAL_SPI_Transmit(&MAG_SPI, &temp_data, 1, 100);
    MAG_SPI_status = HAL_SPI_Receive(&MAG_SPI, data, 1, 100);
    deactivate_mag();*/

    uint8_t txData[2], rxData[2];
    txData[0] = 0x80 | address;
    txData[1] = 0xFF;
    activate_mag();
    MAG_SPI_status = HAL_SPI_TransmitReceive(&MAG_SPI, txData, rxData, 2, 100);
    deactivate_mag();
    *data = rxData[1];
}

bool LIS3MDL_SelfTest(void){
    uint8_t Who_Am_I_return = 0;
    MAG_read_reg(WHO_AM_I_REG, &Who_Am_I_return);
    if(Who_Am_I_return == 0x3d){
        return true;
    }
    else{
        return false;
    }
}