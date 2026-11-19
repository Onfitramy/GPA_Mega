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

SERVO_Instance_t servo_instance;

/*Initiate a Servo defined by a 16bit number */
void SERVO_Init(SERVO_Instance_t *instance, uint16_t servo_num, uint16_t MinPulse_us, uint16_t MaxPulse_us, uint16_t max_angle) {
    servo_num--; // Convert to 0 based index
    instance->configs[servo_num].MinPulse = MinPulse_us;
    instance->configs[servo_num].MaxPulse = MaxPulse_us;
    instance->configs[servo_num].MaxAngle = max_angle;

    // Channel Number 1 - 8
    TIM_HandleTypeDef htim;

    /*Initialysing like MX_TIM3_Init but with the specific details pulled from SERVO_cfg*/
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /*Configure general TIM_HandleTypeDef settings*/
    htim.Instance = instance->configs[servo_num].TIM_Instance; 
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
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, instance->configs[servo_num].PWM_TIM_CH) != HAL_OK)
    {
        Error_Handler();
    }

    /*Calculate & Save The Servo Pulse Information*/
	instance->configs[servo_num].MinPeriod = (uint16_t)(SERVO_TIM_PERIOD * (instance->configs[servo_num].MinPulse * 1e-6 * SERVO_TIM_FREQ));
	instance->configs[servo_num].MaxPeriod = (uint16_t)(SERVO_TIM_PERIOD * (instance->configs[servo_num].MaxPulse * 1e-6 * SERVO_TIM_FREQ));

    /*Finish Initialisation*/
    HAL_TIM_MspPostInit(&htim);

    /*Startup current Servo*/
    HAL_TIM_PWM_Start(&htim, instance->configs[servo_num].PWM_TIM_CH);

}

/* Moves A Specific Motor With A Raw Pulse Width Value 1 is Min, 1.5 Mid, 2 high */
void SERVO_RawMove(SERVO_Instance_t *instance, uint16_t servo_num, uint16_t Pulse_us)
{
    servo_num--; // Convert to 0 based index

    uint16_t au16_Pulse = (uint16_t)round((1e-6 * SERVO_TIM_PERIOD * Pulse_us * SERVO_TIM_FREQ));
    // Set the CCRx register to the calculated pulse width
	*(instance->configs[servo_num].TIM_CCRx) = au16_Pulse;
}

/*Move Servo to specific degree*/
void SERVO_MoveToAngle(SERVO_Instance_t *instance, uint16_t servo_num, float af_Angle)
{
    servo_num--; // Convert to 0 based index

    // Channel Number 1 - 8
    uint16_t au16_Pulse = 0;

    au16_Pulse = instance->configs[servo_num].MinPeriod + (af_Angle/instance->configs[servo_num].MaxAngle)*(instance->configs[servo_num].MaxPeriod - instance->configs[servo_num].MinPeriod);

    *(instance->configs[servo_num].TIM_CCRx) = au16_Pulse;
}

void SERVO_ZeroAll(SERVO_Instance_t *instance) {
    for(uint16_t i = 1; i <= SERVO_NUM; i++) {
        SERVO_MoveToAngle(instance, i, 0);
    }
}

void DeployDrogue(SERVO_Instance_t *instance, uint16_t servo_num, float angle_deploy, float move_delay) {
    SERVO_MoveToAngle(instance, servo_num, angle_deploy);
    drogue_deploy_attempts++;

    instance->move_duration = move_delay;
    HAL_TIM_Base_Start_IT(instance->move_tim);
}

void RetractMain(SERVO_Instance_t *instance, uint16_t move_time_ms) {
    if (move_time_ms == 0) {
        SERVO_MoveToAngle(instance, MAIN_SERVO, MAIN_NEUTRAL_ANGLE);
        return;
    }
    SERVO_MoveToAngle(instance, MAIN_SERVO, MAIN_DEPLOY_ANGLE);
    
    tim16_target_ms = move_time_ms;
    HAL_TIM_Base_Start_IT(&htim16);
}

void Servo_TimeHandler(SERVO_Instance_t *instance, TIM_HandleTypeDef *htim) {
    // check whether our timer got called
    if (instance->move_tim->Instance != htim->Instance)
        return;
    
    // increment counter
    instance->move_tim_counter++;

    // have we reached the target time?
    if (instance->move_tim_counter < instance->move_duration)
        return;

    // halt & reset the timer
    HAL_TIM_Base_Stop_IT(instance->move_tim);
    instance->move_tim_counter = 0;

    // move to the target position
    SERVO_MoveToAngle(instance, instance->move_servo_num, instance->next_move_angle);
}