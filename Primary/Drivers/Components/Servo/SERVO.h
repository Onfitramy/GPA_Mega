#ifndef SERVO_H_
#define SERVO_H_

#define HAL_TIM_MODULE_ENABLED

#include "stm32h7xx_hal.h"

// The Number OF Servo Motors To Be Used In The Project
#define SERVO_NUM  8
 
typedef struct {
    GPIO_TypeDef * SERVO_GPIO;
    uint16_t       SERVO_PIN;
    TIM_TypeDef*   TIM_Instance;
    uint32_t*      TIM_CCRx;
    uint32_t       PWM_TIM_CH;
    uint16_t       MinPulse;
    uint16_t       MaxPulse;
    uint16_t       MinPeriod;
	uint16_t       MaxPeriod;
} SERVO_CfgType;
 
/*-----[ Prototypes For All Functions ]-----*/
 
void SERVO_Init(uint16_t au16_SERVO_Instance, uint16_t MinPulse_us, uint16_t MaxPulse_us);

void SERVO_RawMove(uint16_t au16_SERVO_Instance, uint16_t Pulse_us);

void SERVO_MoveToAngle(uint16_t au16_SERVO_Instance, float af_Angle);

void SERVO_Test(uint8_t Servo, uint8_t Zero_val, int8_t angle, uint8_t dt);

void SERVO_ZeroAll();

#endif /* SERVO_H_ */