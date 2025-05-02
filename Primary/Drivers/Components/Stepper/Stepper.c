#include "Stepper.h"
#include "main.h"

void Stepper_moveSteps(int steps) {
    // Set direction based on sign of steps
    if (steps > 0) {
        HAL_GPIO_WritePin(GPIO12_GPIO_Port, GPIO12_Pin, GPIO_PIN_SET);      // Forward
    } else {
        HAL_GPIO_WritePin(GPIO12_GPIO_Port, GPIO12_Pin, GPIO_PIN_RESET);    // Reverse
        steps = -steps;          // Make step count positive
    }

    for (int i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(GPIO11_GPIO_Port, GPIO11_Pin, GPIO_PIN_SET);
        //delay_us(1000);
        HAL_GPIO_WritePin(GPIO11_GPIO_Port, GPIO11_Pin, GPIO_PIN_RESET);
        //delay_us(1000);
    }
}