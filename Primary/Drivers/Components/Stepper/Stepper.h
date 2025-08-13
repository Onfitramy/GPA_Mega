#ifndef Stepper_H_
#define Stepper_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "main.h"

#define MAX_STEPPER_STEPS 100   // Maximum number of steps to move in one call
#define STEPPER_STEP_TIME 2     // Time in ms for one step

#define rod_inclination 8

extern TIM_HandleTypeDef htim3;

extern float pos_Stepper;
extern uint16_t pwmData[MAX_STEPPER_STEPS];

extern volatile uint8_t dma_waiting_stepper;

int16_t Stepper_moveSteps(int16_t steps);
void Stepper_movetoPos(float pos_cmd);
void Stepper_Send (int16_t steps);

#endif /* Stepper_H_ */