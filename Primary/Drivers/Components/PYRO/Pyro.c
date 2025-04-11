#include "Pyro.h"
#include "main.h"
#include "adc.h"

/*Pyro Channel 1 or 2 | Firing duration [ms]*/
int pyroFIRE(uint16_t channel, int duration) {
    if(channel == 1) {
        HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, GPIO_PIN_SET);
    } else if(channel == 2) {
        HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, GPIO_PIN_SET);
    } else {
        return 0;
    }
    HAL_Delay(duration);
    HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, GPIO_PIN_RESET);
    return channel;
}

/*Pyro Channel 1 or 2, returns calculated resistance*/
double pyroRead(int16_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    if(channel == 1) {
        sConfig.Channel = ADC_CHANNEL_10;
    } else if(channel == 2) {
        sConfig.Channel = ADC_CHANNEL_11;
    } else {
        return -1;
    }
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;

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

    double V_read = 3.3 * adcValue / 65536;
    double V_bat = 4.65;         // THIS VALUE NEEDS TO BE READ FROM F4 ADC!

    double R_read = V_read * 10000 / (V_bat - V_read * 61 / 11);
    
    return R_read;
}