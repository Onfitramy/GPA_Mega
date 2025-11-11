#ifndef ws2812_H_
#define ws2812_H_

#include "main.h"

#define RGB_MAX_BRIGHTNESS 255

extern TIM_HandleTypeDef htim8;

extern volatile uint8_t dma_waiting_ws2812;

extern uint32_t led_skip_counter;

void Set_LED (uint8_t Red, uint8_t Green, uint8_t Blue);

void WS2812_Send (void);

#endif /* ws8212_H_ */