#ifndef VoltageReader_H_
#define VoltageReader_H_

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

typedef struct {
  struct {
    float reg_3V3;
    float battery;
  } temperature;
  struct {
    float bus_pu_bat;
    float bus_gpa_bat;
    float bus_5V;
    float shunt_pu;
  } voltage;
  struct {
    float out_pu;
  } current;
  struct {
    float out_pu;
  } power;
} health_t;

float readVoltage(int channel);
float readTemperature(int channel);

void FilterLP(health_t *health_raw, health_t *health_filt);

#endif /*VoltageReader_H_*/