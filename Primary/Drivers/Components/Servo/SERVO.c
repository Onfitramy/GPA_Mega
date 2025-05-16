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


/*Saves individual Servo Data*/
typedef struct
{
	uint16_t  Period_Min;
	uint16_t  Period_Max;
}SERVO_info;

static SERVO_info gs_SERVO_info[SERVO_NUM + 1] = {0};


/*Initiate a Servo defined by a 16bit number */
void SERVO_Init(uint16_t au16_SERVO_Instance)
{
    // Channel Number 1 - 8
    TIM_HandleTypeDef htim;

    /*Initialysing like MX_TIM3_Init but with the specific details pulled from SERVO_cfg*/
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /*Configure general TIM_HandleTypeDef settings*/
    htim.Instance = SERVO_CfgParam[au16_SERVO_Instance].TIM_Instance; 
    htim.Init.Prescaler = 2500; /*34MHz / 1000 -> 34000Hz / 680 -> 50Hz*/
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = SERVO_CfgParam[au16_SERVO_Instance].TIM_CLK;
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
    if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, SERVO_CfgParam[au16_SERVO_Instance].PWM_TIM_CH) != HAL_OK)
    {
        Error_Handler();
    }

    /*Calculate & Save The Servo Pulse Information*/
	gs_SERVO_info[au16_SERVO_Instance].Period_Min = (uint16_t) (1000.0 * (SERVO_CfgParam[au16_SERVO_Instance].MinPulse/20.0));
	gs_SERVO_info[au16_SERVO_Instance].Period_Max = (uint16_t) (1000.0 * (SERVO_CfgParam[au16_SERVO_Instance].MaxPulse/20.0));

    /*Finish Initialisation*/
    HAL_TIM_MspPostInit(&htim);

    /*Startup current Servo*/
    HAL_TIM_PWM_Start(&htim, SERVO_CfgParam[au16_SERVO_Instance].PWM_TIM_CH);

}

/* Moves A Specific Motor With A Raw Pulse Width Value 1 is Min, 1.5 Mid, 2 high */
void SERVO_RawMove(uint16_t au16_SERVO_Instance, uint16_t au16_Pulse)
{
     
	*(SERVO_CfgParam[au16_SERVO_Instance].TIM_CCRx) = au16_Pulse * 50;
}

/*Move Servo to specific degree*/
void SERVO_MoveToAngle(uint16_t au16_SERVO_Instance, float af_Angle)
{
    // Channel Number 1 - 8
    uint16_t au16_Pulse = 0;

    au16_Pulse = 50 + (af_Angle/180.0)*50;

    *(SERVO_CfgParam[au16_SERVO_Instance].TIM_CCRx) = au16_Pulse;
}

void SERVO_Sweep(int8_t angle, uint8_t dt) {
    SERVO_ZeroAll();
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(PX_SERVO, PX_ZERO + i);
        SERVO_MoveToAngle(NX_SERVO, NX_ZERO + i);
        SERVO_MoveToAngle(PZ_SERVO, PZ_ZERO + i);
        SERVO_MoveToAngle(NZ_SERVO, NZ_ZERO + i);
        HAL_Delay(dt);
    }

    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(PX_SERVO, PX_ZERO - i);
        SERVO_MoveToAngle(NX_SERVO, NX_ZERO - i);
        SERVO_MoveToAngle(PZ_SERVO, PZ_ZERO - i);
        SERVO_MoveToAngle(NZ_SERVO, NZ_ZERO - i);
        HAL_Delay(dt);
    }
    SERVO_ZeroAll();
}

void SERVO_Test(uint8_t Servo, uint8_t Zero_val, int8_t angle, uint8_t dt) {
    SERVO_ZeroAll();
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(Servo, Zero_val + i);
        HAL_Delay(dt);
    }
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(Servo, Zero_val - i);
        HAL_Delay(dt);
    }
    SERVO_ZeroAll();
}

void SERVO_TestX(int8_t angle, uint8_t dt) {
    SERVO_ZeroAll();
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(PX_SERVO, PX_ZERO + i);
        SERVO_MoveToAngle(NX_SERVO, NX_ZERO - i);
        HAL_Delay(dt);
    }
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(PX_SERVO, PX_ZERO - i);
        SERVO_MoveToAngle(NX_SERVO, NX_ZERO + i);
        HAL_Delay(dt);
    }
    SERVO_ZeroAll();
}

void SERVO_TestZ(int8_t angle, uint8_t dt) {
    SERVO_ZeroAll();
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(PZ_SERVO, PZ_ZERO + i);
        SERVO_MoveToAngle(NZ_SERVO, NZ_ZERO - i);
        HAL_Delay(dt);
    }
    for(int i = 0; i < angle; i++) {
        SERVO_MoveToAngle(PZ_SERVO, PZ_ZERO - i);
        SERVO_MoveToAngle(NZ_SERVO, NZ_ZERO + i);
        HAL_Delay(dt);
    }
    SERVO_ZeroAll();
}

void SERVO_ZeroAll() {
    SERVO_MoveToAngle(PX_SERVO, PX_ZERO);
    SERVO_MoveToAngle(NX_SERVO, NX_ZERO);
    SERVO_MoveToAngle(PZ_SERVO, PZ_ZERO);
    SERVO_MoveToAngle(NZ_SERVO, NZ_ZERO);
}

void SERVO_TestSequence() {
    SERVO_MoveToAngle(PY_MOTOR, 180);
    SERVO_MoveToAngle(NY_MOTOR, 180);
    SERVO_Test(PX_SERVO, PX_ZERO, 60, 10);
    SERVO_Test(PZ_SERVO, PZ_ZERO, 60, 10);
    SERVO_Test(NX_SERVO, NX_ZERO, 60, 10);
    SERVO_Test(NZ_SERVO, NZ_ZERO, 60, 10);
    SERVO_MoveToAngle(PY_MOTOR, 0);
    SERVO_MoveToAngle(NY_MOTOR, 0);
    HAL_Delay(100);
    SERVO_TestX(60, 10);
    SERVO_TestZ(60, 10);
    HAL_Delay(100);
    SERVO_Sweep(60, 10);
}

void SERVO_TVC(float angle_x, float angle_z, float twist_y) {
    twist_y = fconstrain(twist_y, -30, 30);
    SERVO_MoveToAngle(PX_SERVO, (float)PX_ZERO + fconstrain(twist_y + angle_x, -60, 60));
    SERVO_MoveToAngle(NX_SERVO, (float)NX_ZERO + fconstrain(twist_y - angle_x, -60, 60));
    SERVO_MoveToAngle(PZ_SERVO, (float)PZ_ZERO + fconstrain(twist_y - angle_z, -60, 60));
    SERVO_MoveToAngle(NZ_SERVO, (float)NZ_ZERO + fconstrain(twist_y + angle_z, -60, 60));
}

float fconstrain(float variable, float min, float max) {
    if(variable > max) {
        return max;
    } else if(variable < min) {
        return min;
    } else {
        return variable;
    }
}