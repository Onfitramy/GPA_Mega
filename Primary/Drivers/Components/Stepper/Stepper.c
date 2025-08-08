#include "Stepper.h"
#include "NRF24L01P.h"
#include "main.h"

float pos_Stepper = 0;

/*
 *  NOTES:
 *  GPIO11: TIM3_CH3 -> Stepper STEP Pin
 *  GPIO12: TIM3_CH4
 */

void Stepper_moveSteps(int steps) {
    HAL_GPIO_WritePin(GPIO13_GPIO_Port, GPIO13_Pin, GPIO_PIN_RESET); // enable

    // Set direction based on sign of steps
    if (steps > 0) {
        HAL_GPIO_WritePin(GPIO12_GPIO_Port, GPIO12_Pin, GPIO_PIN_SET);      // Forward
    } else {
        HAL_GPIO_WritePin(GPIO12_GPIO_Port, GPIO12_Pin, GPIO_PIN_RESET);    // Reverse
        steps = -steps;          // Make step count positive
    }

    uint16_t pwmData[steps];

	for (int i = 0; i < steps; i++)
	{
			pwmData[i] = 10;
	}

	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)pwmData, steps);
	while (!datasentflag_Stepper){}; // !FIX! This blocks the thread until the DMA transfer is complete if the DMA transfer is not complete, the thread will not continue
	datasentflag_Stepper = 0;

    HAL_GPIO_WritePin(GPIO13_GPIO_Port, GPIO13_Pin, GPIO_PIN_SET);  // disable
}

void Stepper_movetoPos(float pos_cmd) {
    int steps = 200. * (pos_Stepper - pos_cmd) / rod_inclination; // calculate step count
    Stepper_moveSteps(steps); // move
    pos_Stepper = pos_cmd; // update position
}