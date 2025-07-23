#ifndef ws2812_H_
#define ws2812_H_

#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim3;

void Set_LED (int Red, int Green, int Blue);

void WS2812_Send (void);

#endif /* ws8212_H_ */