#include "BMP390.h"

HAL_StatusTypeDef BMP_I2C_status;

void BMP_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    BMP_I2C_status = HAL_I2C_Master_Transmit(&hi2c3, BMP390_I2C_ADDR, buffer, 2, 100);
}

void BMP_read_reg(uint8_t reg, uint8_t *data)
{
    BMP_I2C_status = HAL_I2C_Master_Transmit(&hi2c3, BMP390_I2C_ADDR, &reg, 1, 100);
    BMP_I2C_status = HAL_I2C_Master_Receive(&hi2c3, BMP390_I2C_ADDR, data, 1, 100);
}

bool BMP390_SelfTest(void)
{
    uint8_t Who_Am_I_return = 0;
    BMP_read_reg(BMP390_CHIP_ID_REG, &Who_Am_I_return);

    return (Who_Am_I_return == 0x60);  // 0x60 = BMP390 ID
}