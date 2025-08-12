/*
 *  Servo Configuration. Cheap Servos go from 500us to 2400us while expensive ones can go from 1000us to 2000us.
 */

#include "SERVO.h"

SERVO_CfgType SERVO_Data[SERVO_NUM] =
{
	/* CH1 PWM Configurations */
    {
	    GPIOA,          // GPIO Port
		GPIO_PIN_0,     // GPIO_PIN
		TIM2,           // TIM Instance
		&TIM2->CCR1, 	// Pointer to Capture/Compare Register
		TIM_CHANNEL_1,  // Timer Channel
	},

    /* CH2 PWM Configurations */
    {
	    GPIOA,
		GPIO_PIN_1,
		TIM2,
		&TIM2->CCR2,
		TIM_CHANNEL_2,
	},

    /* CH3 PWM Configurations */
    {
	    GPIOA,
		GPIO_PIN_2,
		TIM2,
		&TIM2->CCR3,
		TIM_CHANNEL_3,
	},

    /* CH4 PWM Configurations */
    {
	    GPIOA,
		GPIO_PIN_3,
		TIM2,
		&TIM2->CCR4,
		TIM_CHANNEL_4,
	},

    /* CH5 PWM Configurations */
    {
	    GPIOD,
		GPIO_PIN_12,
		TIM4,
		&TIM4->CCR1,
		TIM_CHANNEL_1,
	},

    /* CH6 PWM Configurations */
    {
	    GPIOD,
		GPIO_PIN_13,
		TIM4,
		&TIM4->CCR2,
		TIM_CHANNEL_2,
	},

    /* CH7 PWM Configurations */
    {
	    GPIOD,
		GPIO_PIN_14,
		TIM4,
		&TIM4->CCR3,
		TIM_CHANNEL_3,
	},

    /* CH8 PWM Configurations */
    {
	    GPIOD,
		GPIO_PIN_15,
		TIM4,
		&TIM4->CCR4,
		TIM_CHANNEL_4,
	}

};
