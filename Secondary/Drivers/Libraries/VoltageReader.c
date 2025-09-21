#include "VoltageReader.h"
#include "main.h"
#include "math.h"

/*1 => +5V, 2 => +BAT*/
float readVoltage(int channel) {
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

    float V_read = 3.3 * adcValue / 4096;

    return V_read;
}

/*1 => NTC_GPA, 2 => NTC_BAT*/
float readTemperature(int channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    if(channel == 1) {
        sConfig.Channel = ADC_CHANNEL_14;
    } else if(channel == 2) {
        sConfig.Channel = ADC_CHANNEL_15;
    } else {
        return -1;
    }
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    uint16_t adcValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float Temperature = (double)3380/log((double)adcValue / (4096 - adcValue) * exp((double)3380/298.15)) - 273.15;

    return Temperature;
}