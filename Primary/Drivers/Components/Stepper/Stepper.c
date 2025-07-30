#include "Stepper.h"
#include "NRF24L01P.h"
#include "main.h"

float pos_Stepper = 0;

void Stepper_moveSteps(int steps, int us) {
    HAL_GPIO_WritePin(GPIO13_GPIO_Port, GPIO13_Pin, GPIO_PIN_RESET); // enable
    HAL_Delay(5);
    // Set direction based on sign of steps
    if (steps > 0) {
        HAL_GPIO_WritePin(GPIO11_GPIO_Port, GPIO11_Pin, GPIO_PIN_SET);      // Forward
    } else {
        HAL_GPIO_WritePin(GPIO11_GPIO_Port, GPIO11_Pin, GPIO_PIN_RESET);    // Reverse
        steps = -steps;          // Make step count positive
    }

    for (int i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(GPIO12_GPIO_Port, GPIO12_Pin, GPIO_PIN_SET);
        delay_us(us);
        HAL_GPIO_WritePin(GPIO12_GPIO_Port, GPIO12_Pin, GPIO_PIN_RESET);
        delay_us(us);
    }
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIO13_GPIO_Port, GPIO13_Pin, GPIO_PIN_SET);  // disable
}

void Stepper_movetoPos(float pos_cmd, int us) {
    int steps = 200. * (pos_Stepper - pos_cmd) / rod_inclination; // calculate step count
    Stepper_moveSteps(steps, us); // move
    pos_Stepper = pos_cmd; // update position
}