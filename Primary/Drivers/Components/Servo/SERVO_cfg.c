/*
 *  Servo Configuration. Cheap Servos go from 500us to 2400us while expensive ones can go from 1000us to 2000us.
 */

#include "SERVO.h"

SERVO_CfgType SERVO_Data[SERVO_NUM] =
{
	/* CH1 (PA0) PWM Configurations */
    {
		TIM2,           // TIM Instance
		&TIM2->CCR1, 	// Pointer to Capture/Compare Register
		TIM_CHANNEL_1,  // Timer Channel
		1500,              // Min Pulse in microseconds
		2500,              // Max Pulse in microseconds
		0,              // Min Period in microseconds
		0,              // Max Period in microseconds
		135               // Max Angle in degrees
	},

    /* CH2 (PA1) PWM Configurations */
    {
		TIM2,
		&TIM2->CCR2,
		TIM_CHANNEL_2,
		1500,
		2500,
		0,
		0,
		135
	},

    /* CH3 (PA2) PWM Configurations */
    {
		TIM2,
		&TIM2->CCR3,
		TIM_CHANNEL_3,
		1500,
		2500,
		0,
		0,
		135
	},

    /* CH4 (PA3) PWM Configurations */
    {
		TIM2,
		&TIM2->CCR4,
		TIM_CHANNEL_4,
		1500,
		2500,
		0,
		0,
		135
	},

    /* CH5 (PD12) PWM Configurations */
    {
		TIM4,
		&TIM4->CCR1,
		TIM_CHANNEL_1,
		1500,
		2500,
		0,
		0,
		135
	},

    /* CH6 (PD13) PWM Configurations */
    {
		TIM4,
		&TIM4->CCR2,
		TIM_CHANNEL_2,
		1500,
		2500,
		0,
		0,
		135
	},

    /* CH7 (PD14) PWM Configurations */
    {
		TIM4,
		&TIM4->CCR3,
		TIM_CHANNEL_3,
		1500,
		2500,
		0,
		0,
		135
	},

    /* CH8 (PD15) PWM Configurations */
    {
		TIM4,
		&TIM4->CCR4,
		TIM_CHANNEL_4,
		1500,
		2500,
		0,
		0,
		135
	}

};
