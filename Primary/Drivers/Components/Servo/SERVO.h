#ifndef SERVO_H_
#define SERVO_H_

#define HAL_TIM_MODULE_ENABLED

#include "stm32h7xx_hal.h"

// The Number OF Servo Motors To Be Used In The Project
#define SERVO_NUM  8

#define DROGUE_SERVO            1
#define DROGUE_NEUTRAL_ANGLE    0.f
#define DROGUE_DEPLOY_ANGLE     35.f
#define DROGUE_MOVE_DELAY_MS    300

#define MAIN_SERVO              2
#define MAIN_NEUTRAL_ANGLE      0.f
#define MAIN_DEPLOY_ANGLE       -90.f
#define MAIN_MOVE_DELAY_MS      700

typedef struct {
    TIM_TypeDef*   TIM_Instance;
    volatile uint32_t*      TIM_CCRx;
    uint32_t       PWM_TIM_CH;
    uint16_t       MinPulse;
    uint16_t       MaxPulse;
    uint16_t       MinPeriod;
	uint16_t       MaxPeriod;
    uint16_t       MaxAngle;
} SERVO_CfgType;

typedef struct {
    SERVO_CfgType *config;
    uint16_t servo_index;           // the index of the servo
    TIM_HandleTypeDef *move_tim;    // timer used to schedule servo movements in the future
    uint32_t move_duration;         // the duration of the current position
    uint32_t move_tim_counter;      // counter used by move timer
    uint8_t move_interpolate;       // 0 -> the angle is interpolated | 1 -> abrupt change
    float start_angle;
    float end_angle;
} SERVO_Instance_t;

extern SERVO_Instance_t drogue_servo;
extern SERVO_Instance_t main_servo;

extern uint8_t drogue_deploy_attempts;
extern uint8_t main_deploy_attempts;

/*-----[ Prototypes For All Functions ]-----*/
 
void SERVO_Init(SERVO_CfgType *config_list, SERVO_Instance_t *instance, uint16_t servo_num,
                TIM_HandleTypeDef *delay_tim, uint16_t MinPulse_us, uint16_t MaxPulse_us, uint16_t max_angle);
void SERVO_RawMove(SERVO_Instance_t *instance, uint16_t Pulse_us);
void SERVO_MoveToAngle(SERVO_Instance_t *instance, float af_Angle);
void SERVO_MoveSequence(SERVO_Instance_t *instance, float start_angle, float end_angle, uint16_t duration_ms, uint8_t interpolate);
void Servo_TimeHandler(SERVO_Instance_t *instance, TIM_HandleTypeDef *htim);

#endif /* SERVO_H_ */