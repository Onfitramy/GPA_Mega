#ifndef Buzzer_H_
#define Buzzer_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "packets.h"

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
void buzzerPlayPattern(BuzzerPattern_t pattern);
void buzzerPlaySecondaryStatus(sm_state_t flight_state);

void playMelody();

float get_frequency(const char *note_name);

extern uint32_t tim10_target1_ms;
extern uint32_t tim10_target2_ms;
extern uint32_t tim11_target_ms;

void ASESongTable(uint8_t event_num);
void FailureSongTable(uint8_t event_num);
void BeepCodeTable(uint8_t event_num, BuzzerPattern_t beep_code);

// #define BUZZER_BEEP_CODES
#define BUZZER_BEEP_SHORT  200
#define BUZZER_BEEP_LONG   500
#define BUZZER_BEEP_DELAY  200
#define BUZZER_BEEP_REPEAT_TIME  30000
#define BUZZER_BEEP_HIGH "B5"
#define BUZZER_BEEP_LOW "E2"

static uint8_t BEEP_CODE_PRIMARY_ARMED_TABLE[] = { 0, 0 };
static uint8_t BEEP_CODE_PRIMARY_ABORT_TABLE[] = { 1, 1, 1, 1 };
static uint8_t BEEP_CODE_SECONDARY_BATTERY_LOW_TABLE[] = { 1, 1 };
static uint8_t BEEP_CODE_SECONDARY_BATTERY_CRITICAL_TABLE[] = { 1, 1, 1 };
static uint8_t BEEP_CODE_SECONDARY_FAULT_TABLE[] = { 1, 1, 1, 1 };

#endif /* Buzzer_H_ */