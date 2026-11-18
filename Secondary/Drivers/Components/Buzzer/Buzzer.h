#ifndef Buzzer_H_
#define Buzzer_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "packets.h"

#define BUZZER_BEEP_CODES
#define BUZZER_BEEP_REPEAT_TIME  15000

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

extern bool buzzerPeriodicMode;
extern bool buzzerActivePeriod;
extern BuzzerPattern_t buzzerPattern;
extern BuzzerPattern_t primaryPattern;
extern BuzzerPattern_t secondaryPattern;
extern bool primaryActive;
extern uint8_t buzzerEventNumber;

void buzzerInit();
void buzzerStop();
void buzzerStart();
void buzzerStopPeriodic();
void buzzerStartPeriodic();
void buzzerPlayNote(char *note, uint32_t duration_ms);
void buzzerDelay(uint32_t duration_ms);
void buzzerEnablePeriodicMode1(char *note, uint32_t duration_on_ms, uint32_t duration_off_ms);
void buzzerEnablePeriodicMode2(BuzzerPattern_t pattern, uint32_t period_ms);
void buzzerDisablePeriodicMode2();
void buzzerPlayPattern(BuzzerPattern_t pattern);
void buzzerPlaySecondaryStatus(sm_state_t flight_state);

void playMelody();

float get_frequency(const char *note_name);

extern uint32_t tim10_target1_ms;
extern uint32_t tim10_target2_ms;
extern uint32_t tim11_target_ms;
extern volatile uint32_t tim11_ms;

void buzzerPatternTable(BuzzerPattern_t pattern, uint8_t event_num);

#endif /* Buzzer_H_ */