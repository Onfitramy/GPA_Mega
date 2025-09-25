#include "Buzzer.h"
#include "main.h"
#include <string.h>

#define TIM_FREQ 84000000

int presForFrequency(int frequency) {
	if (frequency == 0) return 0;
	return ((TIM_FREQ/(1000*frequency))-1);  // 1 is added in the register
}

void buzzerInit() {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 250);
}

void buzzerSetFreq(int frequency) {
	__HAL_TIM_SET_PRESCALER(&htim3, presForFrequency(frequency));
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

/*Set any note between C0 and B8 (C2 - B6 recommended, unless you want to be annoying)*/
void buzzerPlayNote(char *note, uint32_t duration_ms) {
	float frequency = get_frequency(note);
	buzzerSetFreq(frequency);
    HAL_GPIO_TogglePin(M2_LED_GPIO_Port, M2_LED_Pin);
    HAL_Delay(duration_ms);
    buzzerSetFreq(0);
}

// Define the frequency for each note
typedef struct {
    const char *name;  // Note name
    float frequency;   // Corresponding frequency in Hz
} Note;

Note notes[] = {
    {"C0", 16.35f},
    {"C#0", 17.32f},
    {"D0", 18.35f},
    {"D#0", 19.45f},
    {"E0", 20.60f},
    {"F0", 21.83f},
    {"F#0", 23.12f},
    {"G0", 24.50f},
    {"G#0", 25.96f},
    {"A0", 27.50f},
    {"A#0", 29.14f},
    {"B0", 30.87f},
    
    {"C1", 32.70f},
    {"C#1", 34.65f},
    {"D1", 36.71f},
    {"D#1", 38.89f},
    {"E1", 41.20f},
    {"F1", 43.65f},
    {"F#1", 46.25f},
    {"G1", 49.00f},
    {"G#1", 51.91f},
    {"A1", 55.00f},
    {"A#1", 58.27f},
    {"B1", 61.74f},
    
    {"C2", 65.41f},
    {"C#2", 69.30f},
    {"D2", 73.42f},
    {"D#2", 77.78f},
    {"E2", 82.41f},
    {"F2", 87.31f},
    {"F#2", 92.50f},
    {"G2", 98.00f},
    {"G#2", 103.83f},
    {"A2", 110.00f},
    {"A#2", 116.54f},
    {"B2", 123.47f},
    
    {"C3", 130.81f},
    {"C#3", 138.59f},
    {"D3", 146.83f},
    {"D#3", 155.56f},
    {"E3", 164.81f},
    {"F3", 174.61f},
    {"F#3", 185.00f},
    {"G3", 196.00f},
    {"G#3", 207.65f},
    {"A3", 220.00f},
    {"A#3", 233.08f},
    {"B3", 246.94f},
    
    {"C4", 261.63f},  // Middle C
    {"C#4", 277.18f},
    {"D4", 293.66f},
    {"D#4", 311.13f},
    {"E4", 329.63f},
    {"F4", 349.23f},
    {"F#4", 369.99f},
    {"G4", 392.00f},
    {"G#4", 415.30f},
    {"A4", 440.00f},  // A4 = 440 Hz
    {"A#4", 466.16f},
    {"B4", 493.88f},
    
    {"C5", 523.25f},
    {"C#5", 554.37f},
    {"D5", 587.33f},
    {"D#5", 622.25f},
    {"E5", 659.26f},
    {"F5", 698.46f},
    {"F#5", 739.99f},
    {"G5", 783.99f},
    {"G#5", 830.61f},
    {"A5", 880.00f},
    {"A#5", 932.33f},
    {"B5", 987.77f},
    
    {"C6", 1046.50f},
    {"C#6", 1108.73f},
    {"D6", 1174.66f},
    {"D#6", 1244.51f},
    {"E6", 1318.51f},
    {"F6", 1396.91f},
    {"F#6", 1479.98f},
    {"G6", 1567.98f},
    {"G#6", 1661.22f},
    {"A6", 1760.00f},
    {"A#6", 1864.66f},
    {"B6", 1975.53f},
    
    {"C7", 2093.00f},
    {"C#7", 2217.46f},
    {"D7", 2349.32f},
    {"D#7", 2489.02f},
    {"E7", 2637.02f},
    {"F7", 2793.83f},
    {"F#7", 2959.96f},
    {"G7", 3135.96f},
    {"G#7", 3322.44f},
    {"A7", 3520.00f},
    {"A#7", 3729.31f},
    {"B7", 3951.07f},
    
    {"C8", 4186.01f},
    {"C#8", 4434.92f},
    {"D8", 4698.64f},
    {"D#8", 4978.03f},
    {"E8", 5274.04f},
    {"F8", 5587.65f},
    {"F#8", 5919.91f},
    {"G8", 6271.93f},
    {"G#8", 6644.88f},
    {"A8", 7040.00f},
    {"A#8", 7458.62f},
    {"B8", 7902.13f}
};

// Function to get the frequency by note name
float get_frequency(const char *note_name) {
    for (int i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
        if (strcmp(notes[i].name, note_name) == 0) {
            return notes[i].frequency;
        }
    }
    return -1.0f;  // Return -1 if the note isn't found
}

void playMelody() {
    buzzerPlayNote("B3", 200);
    HAL_Delay(10);
    buzzerPlayNote("B3", 200);
    HAL_Delay(10);
    buzzerPlayNote("A#3", 200);
    HAL_Delay(10);
    buzzerPlayNote("B3", 200);
    HAL_Delay(10);
    buzzerPlayNote("G#3", 200);
    HAL_Delay(10);
    buzzerPlayNote("G#3", 200);
    HAL_Delay(10);
    buzzerPlayNote("G#3", 200);
    HAL_Delay(10);
    buzzerPlayNote("G#3", 200);
    HAL_Delay(10);
    buzzerPlayNote("G#3", 200);
    HAL_Delay(10);
    buzzerPlayNote("D#4", 200);
    HAL_Delay(10);
    buzzerPlayNote("C#4", 200);
    HAL_Delay(10);
    buzzerPlayNote("B3", 200);
    HAL_Delay(10);
    buzzerPlayNote("C#4", 600);
    HAL_Delay(10);
    buzzerPlayNote("C#4", 200);
    HAL_Delay(10);
    buzzerPlayNote("C#4", 200);
    HAL_Delay(10);
    buzzerPlayNote("C#4", 200);
    HAL_Delay(10);
    buzzerPlayNote("C#4", 200);
    HAL_Delay(10);
    buzzerPlayNote("C#4", 400);
    HAL_Delay(10);
    buzzerPlayNote("C#4", 200);
    HAL_Delay(10);
    buzzerPlayNote("D#4", 200);
    HAL_Delay(10);
    buzzerPlayNote("A#3", 200);
    HAL_Delay(10);
    buzzerPlayNote("B3", 400);
}