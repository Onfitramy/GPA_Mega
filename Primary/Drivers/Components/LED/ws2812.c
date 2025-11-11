#include "ws2812.h"

uint8_t LED_Data[3];
uint32_t led_skip_counter;

void Set_LED (uint8_t Red, uint8_t Green, uint8_t Blue)
{
	LED_Data[0] = round((float)Green*RGB_MAX_BRIGHTNESS/255.f);
	LED_Data[1] = round((float)Red*RGB_MAX_BRIGHTNESS/255.f);
	LED_Data[2] = round((float)Blue*RGB_MAX_BRIGHTNESS/255.f);
}

uint16_t pwmData_RGB[24+50] = { 0 };

void WS2812_Send (void)
{
	if (led_skip_counter > 10)
		dma_waiting_ws2812 = 0;
	if (dma_waiting_ws2812) {
		led_skip_counter++;
		return; // If DMA is already waiting, do not start a new transfer
	}

	uint32_t indx=0;
	uint32_t color;

	color = ((LED_Data[0]<<16) | (LED_Data[1]<<8) | (LED_Data[2]));

	for (int i=23; i>=0; i--)
	{
		if (color&(1<<i)) 
			pwmData_RGB[indx] = 50;  //50 2/3 of 75

		else
			pwmData_RGB[indx] = 25;  //25 1/3 of 75

		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint32_t *)pwmData_RGB, (24 + 50));
	dma_waiting_ws2812 = 1;
}