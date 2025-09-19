#ifndef ws2812_H_
#define ws2812_H_

#include "stm32f4xx_hal.h"

#define RGB_MAX_BRIGHTNESS 100

extern TIM_HandleTypeDef htim2;

void Set_LED (uint8_t Red, uint8_t Green, uint8_t Blue);

void WS2812_Send (void);

#endif /* ws8212_H_ */