/*  Main Servo Lib
    !WORK IN PROGRESS!
    !SERVOS ONLY WORK WITH EXTERNAL POWER THROUGH USB OR BATTERY(not tested), THEY DONT WORK WITH ST_LINK!
    HOW TO USE: 1.  Configure attached Sero in SERVO_cfg.c
                2.  Call SERVO_Init in main with the SERVO number from SERVO_cfg
                3.  Use SERVO_MoveToAngle() in while
*/

#include "SERVO.h"
#include "SERVO_cfg.h"
#include "tim.h"

uint8_t drogue_deploy_attempts = 0;
uint8_t main_deploy_attempts = 0;

SERVO_Instance_t drogue_servo;
SERVO_Instance_t main_servo;

/*Initiate a Servo defined by a 16bit number */
void SERVO_Init(SERVO_CfgType *config_list, SERVO_Instance_t *instance, uint16_t servo_num,
                TIM_HandleTypeDef *delay_tim, uint16_t MinPulse_us, uint16_t MaxPulse_us, uint16_t max_angle) {
    servo_num--; // Convert to 0 based index
    instance->servo_index = servo_num;
    instance->config = &config_list[servo_num];
    instance->move_tim = delay_tim;
    instance->config->MinPulse = MinPulse_us;
    instance->config->MaxPulse = MaxPulse_us;
    instance->config->MaxAngle = max_angle;

    // Channel Number 1 - 8
    TIM_HandleTypeDef htim;

    /*Initialysing like MX_TIM3_Init but with the specific details pulled from SERVO_cfg*/
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /*Configure general TIM_HandleTypeDef settings*/
    htim.Instance = instance->config->TIM_Instance; 
    htim.Init.Prescaler = SERVO_TIM_PRESCALER - 1; // 125 MHz / (250 * 10000) = 50 Hz
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = SERVO_TIM_PERIOD - 1;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /*Initialise the TIM PWM Time Base*/
    if (HAL_TIM_PWM_Init(&htim) != HAL_OK)
    {
        Error_Handler();
    }

    /*Master Configs*/
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /*Polarity and Modes for the OC Config*/
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, instance->config->PWM_TIM_CH) != HAL_OK)
    {
        Error_Handler();
    }

    /*Calculate & Save The Servo Pulse Information*/
	instance->config->MinPeriod = (uint16_t)(SERVO_TIM_PERIOD * (instance->config->MinPulse * 1e-6 * SERVO_TIM_FREQ));
	instance->config->MaxPeriod = (uint16_t)(SERVO_TIM_PERIOD * (instance->config->MaxPulse * 1e-6 * SERVO_TIM_FREQ));

    /*Finish Initialisation*/
    HAL_TIM_MspPostInit(&htim);

    /*Startup current Servo*/
    HAL_TIM_PWM_Start(&htim, instance->config->PWM_TIM_CH);

}

/* Moves A Specific Motor With A Raw Pulse Width Value 1 is Min, 1.5 Mid, 2 high */
void SERVO_RawMove(SERVO_Instance_t *instance, uint16_t Pulse_us) {
    // calculate the pulse width
    uint16_t au16_Pulse = (uint16_t)round((1e-6 * SERVO_TIM_PERIOD * Pulse_us * SERVO_TIM_FREQ));

    // Set the CCRx register to the calculated pulse width
	*(instance->config->TIM_CCRx) = au16_Pulse;
}

/* Move Servo to specific angle in degrees */
void SERVO_MoveToAngle(SERVO_Instance_t *instance, float af_Angle) {
    // calculate the pulse width
    uint16_t au16_Pulse = 0;
    au16_Pulse = instance->config->MinPeriod + (af_Angle/instance->config->MaxAngle)*(instance->config->MaxPeriod - instance->config->MinPeriod);

    // Set the CCRx register to the calculated pulse width
    *(instance->config->TIM_CCRx) = au16_Pulse;
}

void SERVO_MoveSequence(SERVO_Instance_t *instance, float start_angle, float end_angle, uint16_t duration_ms, uint8_t interpolate) {
    if (duration_ms == 0) {
        SERVO_MoveToAngle(instance, end_angle);
        return;
    }

    // move to start angle
    SERVO_MoveToAngle(instance, start_angle);

    // store start and end angles, duration, and interpolate flag
    instance->start_angle      = start_angle;
    instance->end_angle        = end_angle;
    instance->move_duration    = duration_ms;
    instance->move_interpolate = !(interpolate == 0);

    // start timer
    instance->move_tim_counter = 0;
    HAL_TIM_Base_Start_IT(instance->move_tim);
}

void Servo_TimeHandler(SERVO_Instance_t *instance, TIM_HandleTypeDef *htim) {
    // check whether our timer got called
    if (instance->move_tim->Instance != htim->Instance)
        return;
    
    // increment counter
    instance->move_tim_counter++;

    // have we reached the target time?
    if (instance->move_tim_counter < instance->move_duration) {
        // move to interpolated angle if desired
        if (instance->move_interpolate) {
            float progress = instance->move_tim_counter / instance->move_duration;
            float angle_inter = instance->start_angle * (1.0f - progress) + instance->end_angle * progress;
            SERVO_MoveToAngle(&main_servo, angle_inter);
        }
        return;
    }

    // halt & reset the timer
    HAL_TIM_Base_Stop_IT(instance->move_tim);
    instance->move_tim_counter = 0;

    // move to the target position
    SERVO_MoveToAngle(instance, instance->end_angle);
}