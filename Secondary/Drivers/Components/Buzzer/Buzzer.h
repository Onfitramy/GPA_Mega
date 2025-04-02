#ifndef Buzzer_H_
#define Buzzer_H_

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;

int presForFrequency (int frequency);

#endif /* Buzzer_H_ */