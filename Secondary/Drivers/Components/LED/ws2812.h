#ifndef ws2812_H_
#define ws2812_H_

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;

void Set_LED (int Red, int Green, int Blue);

void WS2812_Send (void);

#endif /* ws8212_H_ */