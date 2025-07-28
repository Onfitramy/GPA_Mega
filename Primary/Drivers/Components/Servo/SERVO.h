#ifndef SERVO_H_
#define SERVO_H_

#define HAL_TIM_MODULE_ENABLED

#include "stm32h7xx_hal.h"

// The Number OF Servo Motors To Be Used In The Project
#define SERVO_NUM  8

#define PX_SERVO 1
#define NX_SERVO 8
#define PZ_SERVO 2
#define NZ_SERVO 7

#define NY_MOTOR 3
#define PY_MOTOR 6

#define PX_ZERO 87
#define NX_ZERO 85
#define PZ_ZERO 81
#define NZ_ZERO 85
 
typedef struct
{
    GPIO_TypeDef * SERVO_GPIO;
    uint16_t       SERVO_PIN;
    TIM_TypeDef*   TIM_Instance;
    uint32_t*      TIM_CCRx;
    uint32_t       PWM_TIM_CH;
    uint32_t       TIM_CLK;
    float          MinPulse;
    float          MaxPulse;
}SERVO_CfgType;
 
/*-----[ Prototypes For All Functions ]-----*/
 
void SERVO_Init(uint16_t au16_SERVO_Instance);

void SERVO_RawMove(uint16_t au16_SERVO_Instance, uint16_t au16_Pulse);

void SERVO_MoveToAngle(uint16_t au16_SERVO_Instance, float af_Angle);
 
void SERVO_Sweep(int8_t angle, uint8_t dt);

void SERVO_Test(uint8_t Servo, uint8_t Zero_val, int8_t angle, uint8_t dt);

void SERVO_TestX(int8_t angle, uint8_t dt);

void SERVO_TestZ(int8_t angle, uint8_t dt);

void SERVO_ZeroAll();

void SERVO_TestSequence();

#endif /* SERVO_H_ */