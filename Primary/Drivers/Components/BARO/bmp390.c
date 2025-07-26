#include "BMP390.h"

HAL_StatusTypeDef BMP_I2C_status;

HAL_StatusTypeDef BMP_write_reg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[2] = {reg, data};

    status = HAL_I2C_Master_Transmit(&hi2c3, BMP390_I2C_ADDR, buffer, 2, 100);
    return status;
}

HAL_StatusTypeDef BMP_read_reg(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c3, BMP390_I2C_ADDR, &reg, 1, 100);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(&hi2c3, BMP390_I2C_ADDR, data, 1, 100);
    return status;
}

HAL_StatusTypeDef BMP_read_burst(uint8_t start_reg, uint8_t *data, uint8_t length)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c3, BMP390_I2C_ADDR, &start_reg, 1, 100);
    if (status != HAL_OK) return status;
    
    status = HAL_I2C_Master_Receive(&hi2c3, BMP390_I2C_ADDR, data, length, 100);
    return status;
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
uint8_t BMP_enable(void) 
{
    BMP_write_reg(BMP_390_PWR_CTRL_REG, 0x33);  // press_en=1, temp_en=1, mode=11

    // 2. Oversampling: osrs_p = x8 (0b011), osrs_t = x1 (0b000)
    BMP_write_reg(BMP_390_OSR_REG, 0x03);       // [5:3]=osrs_t=000, [2:0]=osrs_p=011

    // 3. Output Data Rate (ODR): 50 Hz → odr_sel = 0x02
    BMP_write_reg(BMP_390_ODR_REG, 0x02);       // odr_sel = 0x02

    // 4. IIR Filter: IIR coefficient = 2 → ir_filter = 0b001
    BMP_write_reg(BMP_390_IIR_REG, 0x02);       // bits 2:1 = 0b010 (IIR=2), bit 0 = short_in = 0

    return 0;
}

uint8_t BMP_VerifyDataReady(void) {
    uint8_t status = 0;
    BMP_read_reg(0x03, &status);
    return status;
}


uint8_t BMP_GetRawData(uint32_t *pressure_raw, uint32_t *temperature_raw)
{
    if (BMP_VerifyDataReady() == 0x70) {
        uint8_t buffer[6];
        if (BMP_read_burst(0x04, buffer, 6) != HAL_OK)
            return 0;

        *pressure_raw =
            ((uint32_t)buffer[2] << 16) |
            ((uint32_t)buffer[1] << 8) |
            ((uint32_t)buffer[0]);

        *temperature_raw =
            ((uint32_t)buffer[5] << 16) |
            ((uint32_t)buffer[4] << 8) |
            ((uint32_t)buffer[3]);

        return 1;
    }
    return 0;
}


float bmp390_compensate_temperature(uint32_t uncomp_temp, bmp390_handle_t *handle)
{
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)((int32_t)uncomp_temp - handle->par_t1);
    partial_data2 = partial_data1 * handle->par_t2;

    handle->t_lin = partial_data2 + (partial_data1 * partial_data1) * handle->par_t3;

    return handle->t_lin;
}


float bmp390_compensate_pressure(uint32_t uncomp_press, bmp390_handle_t *handle)
{
    float comp_press;
    float partial_data1, partial_data2, partial_data3, partial_data4;
    float partial_out1, partial_out2;

    float t_lin = handle->t_lin; 

    partial_data1 = handle->par_p6 * t_lin;
    partial_data2 = handle->par_p7 * t_lin * t_lin;
    partial_data3 = handle->par_p8 * t_lin * t_lin * t_lin;
    partial_out1 = handle->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = handle->par_p2 * t_lin;
    partial_data2 = handle->par_p3 * t_lin * t_lin;
    partial_data3 = handle->par_p4 * t_lin * t_lin * t_lin;
    partial_out2 = (float)uncomp_press * (handle->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = handle->par_p9 + handle->par_p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 * (float)uncomp_press * handle->par_p11;

    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}


void BMP_Read_Calibration_Params(bmp390_handle_t *handle) {
    uint8_t buf[2];

    // T1
    BMP_read_reg(BMP390_REG_NVM_PAR_T1_L, &buf[0]);
    BMP_read_reg(BMP390_REG_NVM_PAR_T1_H, &buf[1]);
    handle->t1 = (uint16_t)(buf[1] << 8 | buf[0]);

    // T2
    BMP_read_reg(BMP390_REG_NVM_PAR_T2_L, &buf[0]);
    BMP_read_reg(BMP390_REG_NVM_PAR_T2_H, &buf[1]);
    handle->t2 = (int16_t)(buf[1] << 8 | buf[0]);

    // T3
    BMP_read_reg(BMP390_REG_NVM_PAR_T3, &buf[0]);
    handle->t3 = (int8_t)buf[0];

    // P1
    BMP_read_reg(BMP390_REG_NVM_PAR_P1_L, &buf[0]);
    BMP_read_reg(BMP390_REG_NVM_PAR_P1_H, &buf[1]);
    handle->p1 = (int16_t)(buf[1] << 8 | buf[0]);

    // P2
    BMP_read_reg(BMP390_REG_NVM_PAR_P2_L, &buf[0]);
    BMP_read_reg(BMP390_REG_NVM_PAR_P2_H, &buf[1]);
    handle->p2 = (int16_t)(buf[1] << 8 | buf[0]);

    // P3
    BMP_read_reg(BMP390_REG_NVM_PAR_P3, &buf[0]);
    handle->p3 = (int8_t)buf[0];

    // P4
    BMP_read_reg(BMP390_REG_NVM_PAR_P4, &buf[0]);
    handle->p4 = (int8_t)buf[0];

    // P5
    BMP_read_reg(BMP390_REG_NVM_PAR_P5_L, &buf[0]);
    BMP_read_reg(BMP390_REG_NVM_PAR_P5_H, &buf[1]);
    handle->p5 = (uint16_t)(buf[1] << 8 | buf[0]);

    // P6
    BMP_read_reg(BMP390_REG_NVM_PAR_P6_L, &buf[0]);
    BMP_read_reg(BMP390_REG_NVM_PAR_P6_H, &buf[1]);
    handle->p6 = (uint16_t)(buf[1] << 8 | buf[0]);

    // P7
    BMP_read_reg(BMP390_REG_NVM_PAR_P7, &buf[0]);
    handle->p7 = (int8_t)buf[0];

    // P8
    BMP_read_reg(BMP390_REG_NVM_PAR_P8, &buf[0]);
    handle->p8 = (int8_t)buf[0];

    // P9
    BMP_read_reg(BMP390_REG_NVM_PAR_P9_L, &buf[0]);
    BMP_read_reg(BMP390_REG_NVM_PAR_P9_H, &buf[1]);
    handle->p9 = (int16_t)(buf[1] << 8 | buf[0]);

    // P10
    BMP_read_reg(BMP390_REG_NVM_PAR_P10, &buf[0]);
    handle->p10 = (int8_t)buf[0];

    // P11
    BMP_read_reg(BMP390_REG_NVM_PAR_P11, &buf[0]);
    handle->p11 = (int8_t)buf[0];

    // Temperatur-Koeffizienten
handle->par_t1 = handle->t1 / powf(2.0f, -8);              
handle->par_t2 = handle->t2 / powf(2.0f, 30);
handle->par_t3 = handle->t3 / powf(2.0f, 48);

// Druck-Koeffizienten
handle->par_p1  = (handle->p1 - powf(2.0f, 14)) / powf(2.0f, 20);
handle->par_p2  = (handle->p2 - powf(2.0f, 14)) / powf(2.0f, 29);
handle->par_p3  = handle->p3 / powf(2.0f, 32);
handle->par_p4  = handle->p4 / powf(2.0f, 37);
handle->par_p5  = handle->p5 / powf(2.0f, -3);              
handle->par_p6  = handle->p6 / powf(2.0f, 6);
handle->par_p7  = handle->p7 / powf(2.0f, 8);
handle->par_p8  = handle->p8 / powf(2.0f, 15);
handle->par_p9  = handle->p9 / powf(2.0f, 48);
handle->par_p10 = handle->p10 / powf(2.0f, 48);
handle->par_p11 = handle->p11 / powf(2.0f, 65);
}
