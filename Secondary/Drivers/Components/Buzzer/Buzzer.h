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
extern uint8_t buzzerPattern;
extern uint8_t buzzerEventNumber;

void buzzerInit();
void buzzerStop();
void buzzerStart();
void buzzerStopPeriodic();
void buzzerStartPeriodic();
void buzzerPlayNote(char *note, uint32_t duration_ms);
void buzzerDelay(uint32_t duration_ms);
void buzzerEnablePeriodicMode1(char *note, uint32_t duration_on_ms, uint32_t duration_off_ms);
void buzzerEnablePeriodicMode2(uint8_t pattern, uint32_t period_ms);
void buzzerPlayPattern(uint8_t pattern);

void playMelody();

float get_frequency(const char *note_name);

extern uint32_t tim10_target1_ms;
extern uint32_t tim10_target2_ms;
extern uint32_t tim11_target_ms;

void ASESongTable(uint8_t event_num);
void FailureSongTable(uint8_t event_num);

#endif /* Buzzer_H_ */