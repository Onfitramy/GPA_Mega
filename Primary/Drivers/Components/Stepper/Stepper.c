#include "Stepper.h"
#include "NRF24L01P.h"
#include "main.h"

float pos_Stepper = 0;

/*
 *  NOTES:
 *  GPIO11: TIM3_CH3 -> Stepper STEP Pin
 *  GPIO12: TIM3_CH4
 */

void Stepper_Init(void) {
    for (int i = 0; i < MAX_STEPPER_STEPS; i++) {
        pwmData[i] = 10; // Initialize the PWM data array
    }
}

int16_t Stepper_moveSteps(int16_t steps) {
    if (dma_waiting_stepper)
        return steps;   // If DMA is already waiting, do not start a new transfer

    if (steps == 0)
        return 0;       // No steps to move

    int16_t steps_cmd;

    if (steps > MAX_STEPPER_STEPS)
        steps_cmd = MAX_STEPPER_STEPS;  // Limit to maximum steps
    else if (steps < -MAX_STEPPER_STEPS)
        steps_cmd = -MAX_STEPPER_STEPS; // Limit to minimum steps
    
    HAL_GPIO_WritePin(GPIO13_GPIO_Port, GPIO13_Pin, GPIO_PIN_RESET);        // enable

    // Set direction based on sign of steps
    if (steps > 0) {
        HAL_GPIO_WritePin(GPIO12_GPIO_Port, GPIO12_Pin, GPIO_PIN_SET);      // Forward
        steps_cmd = steps;          // Use positive step count
    } else {
        HAL_GPIO_WritePin(GPIO12_GPIO_Port, GPIO12_Pin, GPIO_PIN_RESET);    // Reverse
        steps_cmd = -steps;         // Make step count positive
    }

	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)pwmData, steps_cmd);
    dma_waiting_stepper = 1;

    HAL_GPIO_WritePin(GPIO13_GPIO_Port, GPIO13_Pin, GPIO_PIN_SET);  // disable

    return steps - steps_cmd; // Return remaining steps
}

void Stepper_movetoPos(float pos_cmd) {
    int steps = 200. * (pos_Stepper - pos_cmd) / rod_inclination; // calculate step count
    Stepper_moveSteps(steps); // move
    pos_Stepper = pos_cmd; // update position
}