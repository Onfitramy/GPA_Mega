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
	},

    /* CH2 (PA1) PWM Configurations */
    {
		TIM2,
		&TIM2->CCR2,
		TIM_CHANNEL_2,
	},

    /* CH3 (PA2) PWM Configurations */
    {
		TIM2,
		&TIM2->CCR3,
		TIM_CHANNEL_3,
	},

    /* CH4 (PA3) PWM Configurations */
    {
		TIM2,
		&TIM2->CCR4,
		TIM_CHANNEL_4,
	},

    /* CH5 (PD12) PWM Configurations */
    {
		TIM4,
		&TIM4->CCR1,
		TIM_CHANNEL_1,
	},

    /* CH6 (PD13) PWM Configurations */
    {
		TIM4,
		&TIM4->CCR2,
		TIM_CHANNEL_2,
	},

    /* CH7 (PD14) PWM Configurations */
    {
		TIM4,
		&TIM4->CCR3,
		TIM_CHANNEL_3,
	},

    /* CH8 (PD15) PWM Configurations */
    {
		TIM4,
		&TIM4->CCR4,
		TIM_CHANNEL_4,
	}

};
