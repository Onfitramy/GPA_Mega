#ifndef VoltageReader_H_
#define VoltageReader_H_

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

float readVoltage(int channel);
float readTemperature(int channel);

#endif /*VoltageReader_H_*/