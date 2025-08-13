/*  Main Servo Lib
    !WORK IN PROGRESS!
    !SERVOS ONLY WORK WITH EXTERNAL POWER THROUGH USB OR BATTERY(not tested), THEY DONT WORK WITH ST_LINK!
    HOW TO USE: 1.  Configure attached Sero in SERVO_cfg.c
                2.  Call SERVO_Init in main with the SERVO number from SERVO_cfg
                3.  Use SERVO_MoveToAngle() in while
*/

#include "SERVO.h"
#include "SERVO_cfg.h"
#include "main.h"

/*Initiate a Servo defined by a 16bit number */
void SERVO_Init(uint16_t au16_SERVO_Instance, uint16_t MinPulse_us, uint16_t MaxPulse_us) {
    au16_SERVO_Instance--; // Convert to 0 based index
    SERVO_Data[au16_SERVO_Instance].MinPulse = MinPulse_us;
    SERVO_Data[au16_SERVO_Instance].MaxPulse = MaxPulse_us;

    // Channel Number 1 - 8
    TIM_HandleTypeDef htim;

    /*Initialysing like MX_TIM3_Init but with the specific details pulled from SERVO_cfg*/
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /*Configure general TIM_HandleTypeDef settings*/
    htim.Instance = SERVO_Data[au16_SERVO_Instance].TIM_Instance; 
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
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, SERVO_Data[au16_SERVO_Instance].PWM_TIM_CH) != HAL_OK)
    {
        Error_Handler();
    }

    /*Calculate & Save The Servo Pulse Information*/
	SERVO_Data[au16_SERVO_Instance].MinPeriod = (uint16_t)(SERVO_TIM_PERIOD * (SERVO_Data[au16_SERVO_Instance].MinPulse * 1e-6 * SERVO_TIM_FREQ));
	SERVO_Data[au16_SERVO_Instance].MaxPeriod = (uint16_t)(SERVO_TIM_PERIOD * (SERVO_Data[au16_SERVO_Instance].MaxPulse * 1e-6 * SERVO_TIM_FREQ));

    /*Finish Initialisation*/
    HAL_TIM_MspPostInit(&htim);

    /*Startup current Servo*/
    HAL_TIM_PWM_Start(&htim, SERVO_Data[au16_SERVO_Instance].PWM_TIM_CH);

}

/* Moves A Specific Motor With A Raw Pulse Width Value 1 is Min, 1.5 Mid, 2 high */
void SERVO_RawMove(uint16_t au16_SERVO_Instance, uint16_t Pulse_us)
{
    au16_SERVO_Instance--; // Convert to 0 based index

    uint16_t au16_Pulse = (uint16_t)round((1e-6 * SERVO_TIM_PERIOD * Pulse_us * SERVO_TIM_FREQ));
    // Set the CCRx register to the calculated pulse width
	*(SERVO_Data[au16_SERVO_Instance].TIM_CCRx) = au16_Pulse;
}

/*Move Servo to specific degree*/
void SERVO_MoveToAngle(uint16_t au16_SERVO_Instance, float af_Angle)
{
    au16_SERVO_Instance--; // Convert to 0 based index

    // Channel Number 1 - 8
    uint16_t au16_Pulse = 0;

    au16_Pulse = SERVO_Data[au16_SERVO_Instance].MinPeriod + (af_Angle/180.0)*(SERVO_Data[au16_SERVO_Instance].MaxPeriod - SERVO_Data[au16_SERVO_Instance].MinPeriod);

    *(SERVO_Data[au16_SERVO_Instance].TIM_CCRx) = au16_Pulse;
}

void SERVO_Test(uint8_t Servo, uint8_t Zero_val, int8_t angle, uint8_t dt) {
    SERVO_MoveToAngle(Servo, Zero_val);
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(Servo, Zero_val + i);
        HAL_Delay(dt);
    }
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(Servo, Zero_val - i);
        HAL_Delay(dt);
    }
    SERVO_MoveToAngle(Servo, Zero_val);
}

void SERVO_ZeroAll() {
    for(uint16_t i = 1; i <= SERVO_NUM; i++) {
        SERVO_MoveToAngle(i, 0);
    }
}