#ifndef Buzzer_H_
#define Buzzer_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

extern bool buzzerPeriodicMode;
extern bool buzzerActivePeriod;

void buzzerInit();
void buzzerStop();
void buzzerStart();
void buzzerPlayNote(char *note, uint32_t duration_ms);
void buzzerEnablePeriodicMode(char *note, uint32_t duration_on_ms, uint32_t duration_off_ms);

void playMelody();

float get_frequency(const char *note_name);

extern uint32_t tim10_target1_ms;
extern uint32_t tim10_target2_ms;
extern uint32_t tim11_target_ms;

#endif /* Buzzer_H_ */