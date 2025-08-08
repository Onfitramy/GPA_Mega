#ifndef Stepper_H_
#define Stepper_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "main.h"

extern float pos_Stepper;

extern TIM_HandleTypeDef htim3;

extern volatile uint8_t datasentflag_Stepper;

void Stepper_moveSteps(int steps);
void Stepper_movetoPos(float pos_cmd);
void Stepper_Send (int16_t steps);

#define rod_inclination 8

#endif /* Stepper_H_ */