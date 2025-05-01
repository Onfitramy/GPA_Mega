#ifndef VR_H_
#define VR_H_

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

double readVoltage(int channel);
double readTemperature(int channel);

#endif /*VR_H_*/