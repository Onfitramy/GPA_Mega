#ifndef Pyro_H_
#define Pyro_H_

#include "stm32h7xx_hal.h"

int pyroFIRE(uint16_t channel, int duration);
double pyroRead(int16_t channel);

#endif /*Pyro_H_*/