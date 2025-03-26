#ifndef ws2812_H_
#define ws2812_H_

#define MAX_LED 1
#define USE_BRIGHTNESS 1

#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim3;

void Set_LED (int LEDnum, int Red, int Green, int Blue);

void Set_Brightness (int brightness);  // 0-45

void WS2812_Send (void);

#endif /* ws8212_H_ */