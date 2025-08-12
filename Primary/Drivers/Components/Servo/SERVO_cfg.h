#ifndef SERVO_CFG_H_
#define SERVO_CFG_H_

#include "SERVO.h"

#define SERVO_TIM_BASE_FREQ	125 // [MHz] Timer base frequency (see CubeMX)
#define SERVO_TIM_FREQ 	    50	// [Hz] Servo PWM Signal Output Frequency
#define SERVO_TIM_RES	    1 	// [us] Servo PWM Signal Output Resolution

#define SERVO_TIM_PERIOD    1e6 / SERVO_TIM_FREQ / SERVO_TIM_RES    // for TIM configuration
#define SERVO_TIM_PRESCALER SERVO_TIM_BASE_FREQ * SERVO_TIM_RES     // for TIM configuration

extern SERVO_CfgType SERVO_Data[SERVO_NUM];

#endif /* SERVO_CFG_H_ */