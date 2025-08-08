#include "ws2812.h"

uint8_t LED_Data[3];

void Set_LED (int Red, int Green, int Blue)
{
	LED_Data[0] = Green;
	LED_Data[1] = Red;
	LED_Data[2] = Blue;
}

uint16_t pwmData[24+50] = { 0 };

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;

	color = ((LED_Data[0]<<16) | (LED_Data[1]<<8) | (LED_Data[2]));

	for (int i=23; i>=0; i--)
	{
		if (color&(1<<i)) 
			pwmData[indx] = 50;  //50 2/3 of 75

		else
			pwmData[indx] = 25;  //25 1/3 of 75

		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint32_t *)pwmData, (24 + 50));
	while (!datasentflag_ws2812){}; // !FIX! This blocks the thread until the DMA transfer is complete if the DMA transfer is not complete, the thread will not continue
	datasentflag_ws2812 = 0;
}