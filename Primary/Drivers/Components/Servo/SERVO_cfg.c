/*
    Servo Configuration, added two Example Servos based on the Test ones. NOT FINAL!
 */

#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM + 1] =
{
	{
		// Servo 0 does not exist.
	},
	/* CH1 PWM Configurations */
    {
	    GPIOA,          // GPIO Port
		GPIO_PIN_0,     // GPIO_PIN
		TIM2,           // TIM Instance
		&TIM2->CCR1, 	// Pointer to Capture/Compare Register
		TIM_CHANNEL_1,  // Timer Channel
		1000,          	// Timer Clock Frequency (Update Rate, this is how many different Steps there are in Voltage)
		2,              // Max pulse width
		1               // min Pulse width
	},

    /* CH2 PWM Configurations */
    {
	    GPIOA,
		GPIO_PIN_1,
		TIM2,
		&TIM2->CCR2,
		TIM_CHANNEL_2,
		1000,
		2,
		1
	},

    /* CH3 PWM Configurations */
    {
	    GPIOA,
		GPIO_PIN_2,
		TIM2,
		&TIM2->CCR3,
		TIM_CHANNEL_3,
		1000,
		2,
		1
	},

    /* CH4 PWM Configurations */
    {
	    GPIOA,
		GPIO_PIN_3,
		TIM2,
		&TIM2->CCR4,
		TIM_CHANNEL_4,
		1000,
		2,
		1
	},

    /* CH5 PWM Configurations */
    {
	    GPIOD,
		GPIO_PIN_12,
		TIM4,
		&TIM4->CCR1,
		TIM_CHANNEL_1,
		1000,
		2,
		1
	},

    /* CH6 PWM Configurations */
    {
	    GPIOD,
		GPIO_PIN_13,
		TIM4,
		&TIM4->CCR2,
		TIM_CHANNEL_2,
		1000,
		2,
		1
	},

    /* CH7 PWM Configurations */
    {
	    GPIOD,
		GPIO_PIN_14,
		TIM4,
		&TIM4->CCR3,
		TIM_CHANNEL_3,
		1000,
		2,
		1
	},

    /* CH8 PWM Configurations */
    {
	    GPIOD,
		GPIO_PIN_15,
		TIM4,
		&TIM4->CCR4,
		TIM_CHANNEL_4,
		1000,
		2,
		1
	}

};
