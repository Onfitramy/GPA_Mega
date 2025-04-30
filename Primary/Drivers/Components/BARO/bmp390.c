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

uint8_t BMP_SelfTest(void)
{
    uint8_t Who_Am_I_return = 0;
    BMP_read_reg(BMP390_CHIP_ID_REG, &Who_Am_I_return);

    if(Who_Am_I_return == 0x60){ // 0x60 = BMP390 ID
        return 1;
    }else{
        return 0;
    }
}

// Ab hier Test von Daniel
uint8_t BMP_enable(void) {
    uint8_t status = 0;
    BMP_write_reg(0x1B, 0x33);
    BMP_read_reg(0x1B, &status);
    if(status == 0x33) return 1;
    else return 0;
}

uint8_t BMP_VerifyDataReady(void) {
    uint8_t status = 0;
    BMP_read_reg(0x03, &status);
    return status;
}

uint8_t BMP_GetPressureRaw(uint32_t *pressure_raw) {
    if(BMP_VerifyDataReady() == 0x70) {
        uint8_t Data_0, Data_1, Data_2;
        BMP_read_reg(0x04, &Data_0);
        BMP_read_reg(0x05, &Data_1);
        BMP_read_reg(0x06, &Data_2);
        *pressure_raw = (uint32_t)Data_2 << 16 | Data_1 << 8 | Data_0;
        return 1;
    } else return 0;
}