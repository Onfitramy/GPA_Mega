#include "VR.h"
#include "main.h"

/*1 => +5V, 2 => +BAT*/
double voltageRead(int channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    if(channel == 1) {
        sConfig.Channel = ADC_CHANNEL_2;
    } else if(channel == 2) {
        sConfig.Channel = ADC_CHANNEL_3;
    } else {
        return -1;
    }
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    uint16_t adcValue = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    double V_read = 3.3 * adcValue / 4096;

    return V_read;
}