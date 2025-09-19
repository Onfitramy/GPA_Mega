#include "ws2812.h"

volatile uint8_t datasentflag;

uint8_t LED_Data[3];

void Set_LED (uint8_t Red, uint8_t Green, uint8_t Blue)
{
	LED_Data[0] = round((float)Green*RGB_MAX_BRIGHTNESS/255.f);
	LED_Data[1] = round((float)Red*RGB_MAX_BRIGHTNESS/255.f);
	LED_Data[2] = round((float)Blue*RGB_MAX_BRIGHTNESS/255.f);
}

uint32_t pwmData[24+64] = { 0 };

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;

	color = ((LED_Data[0]<<16) | (LED_Data[1]<<8) | (LED_Data[2]));

	for (int i=23; i>=0; i--)
	{
		if (color&(1<<i)) 
			pwmData[indx] = 70;

		else
			pwmData[indx] = 35;

		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)pwmData, (24 + 64));
	
	while (!datasentflag){}; // !FIX! This blocks the thread until the DMA transfer is complete if the DMA transfer is not complete, the thread will not continue
	datasentflag = 0;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_2);
	datasentflag = 1;
}