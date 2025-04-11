#ifndef Buzzer_H_
#define Buzzer_H_

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;

int presForFrequency(int frequency);

void buzzerInit();

void buzzerSetFreq(int frequency);

void buzzerSetNote(char *note);

float get_frequency(const char *note_name);

void playMelody();

#endif /* Buzzer_H_ */