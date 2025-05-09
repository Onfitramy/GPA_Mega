#ifndef Stepper_H_
#define Stepper_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"

extern float pos_Stepper;

void Stepper_moveSteps(int steps, int ms);
void Stepper_movetoPos(float pos_cmd, int ms);

#define rod_inclination 8

#endif /* Stepper_H_ */