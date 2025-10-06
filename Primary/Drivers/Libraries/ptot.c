#include "ptot.h"

ptotcb_handle_t ptot_data;

bool ptot_readData(float *pressure, float *temperature) {
    uint8_t buffer[4] = {0};

    uint8_t status_bits = 0;
    uint16_t pressure_bits = 0;
    uint16_t temperature_bits = 0;

    HAL_GPIO_WritePin(EXT2_CS_GPIO_Port, EXT2_CS_Pin, GPIO_PIN_RESET); // Activate CS

    HAL_SPI_Receive(&hspi2, buffer, 4, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(EXT2_CS_GPIO_Port, EXT2_CS_Pin, GPIO_PIN_SET); // Deactivate CS

    status_bits = (buffer[0] & 0xC0) >> 6;

    if (status_bits != 0) return 0;

    pressure_bits = buffer[1];
    pressure_bits |= (buffer[0] & 0x3F) << 8;

    temperature_bits = buffer[3] >> 5;
    temperature_bits |= buffer[2] << 3;

    *pressure = PRESSURE_MIN + (pressure_bits - OUTPUT_MIN) * (PRESSURE_MAX - PRESSURE_MIN) / (OUTPUT_MAX - OUTPUT_MIN);
    *temperature = temperature_bits / 2047.f * 200.f - 50.f;

    return 1;
}