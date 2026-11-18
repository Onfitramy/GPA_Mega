#include "Buzzer.h"
#include "packets.h"

#define TIM_FREQ 84000000

#define BUZZER_PAUSE(duration_ms) .note = "0", .duration = (uint32_t)(duration_ms)
#define MELODY_LIST_ENTRY(mtable) .melody_table = mtable, .num_entries = sizeof(mtable) / sizeof(Melody_entry_t)

bool buzzerPeriodicMode = false;
bool buzzerActivePeriod = false;
BuzzerPattern_t buzzerPattern = BUZZER_PATTERN_PLAY_NOTE;
BuzzerPattern_t primaryPattern = BUZZER_PATTERN_STOP;
BuzzerPattern_t secondaryPattern = BUZZER_PATTERN_STOP;
bool primaryActive = true;
uint8_t buzzerEventNumber = 0;

// Define the frequency for each note
typedef struct {
    const char *name;  // Note name
    float frequency;   // Corresponding frequency in Hz
} Note;

typedef struct {
    char *note;
    uint32_t duration;
} Melody_entry_t;

typedef struct {
    Melody_entry_t *melody_table;
    uint8_t num_entries;
} Melody_list_t;

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

#define DURATION_ASE_SONG_1_8 210
#define DURATION_FIX_YOU_1_8  250
#define DURATION_COW_1_8      100 // x62
#define DURATION_PAUSE        10

Melody_entry_t melodyTable_ASE[] = {
    {.note = "B3", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "B3", .duration = DURATION_ASE_SONG_1_8},
    {.note = "A#3", .duration = DURATION_ASE_SONG_1_8},
    {.note = "B3", .duration = DURATION_ASE_SONG_1_8},
    {.note = "G#3", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "G#3", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "G#3", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "G#3", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "G#3", .duration = DURATION_ASE_SONG_1_8},
    {.note = "D#4", .duration = DURATION_ASE_SONG_1_8},
    {.note = "C#4", .duration = DURATION_ASE_SONG_1_8},
    {.note = "B3", .duration = DURATION_ASE_SONG_1_8},
    {.note = "C#4", .duration = 3 * DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "C#4", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "C#4", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "C#4", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "C#4", .duration = DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "C#4", .duration = 2 * DURATION_ASE_SONG_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "C#4", .duration = DURATION_ASE_SONG_1_8},
    {.note = "D#4", .duration = DURATION_ASE_SONG_1_8},
    {.note = "A#3", .duration = DURATION_ASE_SONG_1_8},
    {.note = "B3", .duration = 2 * DURATION_ASE_SONG_1_8},
};

Melody_entry_t melodyTable_FixYou[] = {
    {.note = "A#3", .duration = DURATION_FIX_YOU_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "A#3", .duration = DURATION_FIX_YOU_1_8},
    {.note = "G4", .duration = 2 * DURATION_FIX_YOU_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "G4", .duration = 2 * DURATION_FIX_YOU_1_8},
    {.note = "D#4", .duration = 2 * DURATION_FIX_YOU_1_8},
    {.note = "A#3", .duration = DURATION_FIX_YOU_1_8 - DURATION_PAUSE},
    {BUZZER_PAUSE(DURATION_PAUSE)},
    {.note = "A#3", .duration = DURATION_FIX_YOU_1_8},
    {.note = "D4", .duration = 2 * DURATION_FIX_YOU_1_8},
    {.note = "D#4", .duration = 2 * DURATION_FIX_YOU_1_8},
    {.note = "D4", .duration = 4 * DURATION_FIX_YOU_1_8},
    {.note = "C4", .duration = 4 * DURATION_FIX_YOU_1_8},
};

Melody_entry_t melodyTable_Cow[] = {
    {.note = "A4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "A4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "A4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "A4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "A4", .duration = 3 * DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "A4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(3 * DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "E4", .duration = 3 * DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "D4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(3 * DURATION_COW_1_8)},
    {.note = "D4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "D4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "D4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "D4", .duration = 3 * DURATION_COW_1_8},
    {BUZZER_PAUSE(3 * DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "A4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "A4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "C4", .duration = 3 * DURATION_COW_1_8},
    {BUZZER_PAUSE(DURATION_COW_1_8)},
    {.note = "E4", .duration = DURATION_COW_1_8},
};

Melody_list_t melodyList[] = {
    [BUZZER_PATTERN_SONG_ASE_SONG - 1] = {MELODY_LIST_ENTRY(melodyTable_ASE)},
    [BUZZER_PATTERN_SONG_FIX_YOU - 1]  = {MELODY_LIST_ENTRY(melodyTable_FixYou)},
    [BUZZER_PATTERN_SONG_COW - 1]      = {MELODY_LIST_ENTRY(melodyTable_Cow)},
    [BUZZER_PATTERN_SONG_RESERVED_6 - 1] = { 0 },
};

#define BUZZER_BEEP_SHORT  200
#define BUZZER_BEEP_LONG   500
#define BUZZER_BEEP_DELAY  200
#define BUZZER_BEEP_HIGH "D#6"
#define BUZZER_BEEP_LOW "E2"

static uint8_t BEEP_CODE_PRIMARY_ARMED_TABLE[]              = { 2, 2, 2, 2 };
static uint8_t BEEP_CODE_PRIMARY_ABORT_TABLE[]              = { 3, 3, 3, 3 };
static uint8_t BEEP_CODE_PRIMARY_RECOVERY_TABLE[]           = { 2, 2, 2, 2, 2, 2, 2, 2};
static uint8_t BEEP_CODE_SECONDARY_BATTERY_LOW_TABLE[]      = { 3, 1 };
static uint8_t BEEP_CODE_SECONDARY_BATTERY_CRITICAL_TABLE[] = { 0, 2, 0 };
static uint8_t BEEP_CODE_SECONDARY_FAULT_TABLE[]            = { 1, 1, 1, 1 };

static int prescalerForFrequency(float frequency) {
	if (frequency == 0) return 0;
	return (TIM_FREQ / (1000.f * frequency)) - 0.5f;
}

void buzzerInit() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 250);
}

static void buzzerSetFreq(float frequency) {
	__HAL_TIM_SET_PRESCALER(&htim3, prescalerForFrequency(frequency));
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void buzzerStop() {
    buzzerActivePeriod = false;
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

void buzzerStart() {
    buzzerActivePeriod = true;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void buzzerStopPeriodic() {
    HAL_TIM_Base_Stop_IT(&htim10);
    buzzerStop();
}

void buzzerStartPeriodic() {
    HAL_TIM_Base_Start_IT(&htim10);
    buzzerStart();
}

/*Set any note between C0 and B8 (C2 - B6 recommended, unless you want to be annoying)*/
void buzzerPlayNote(char *note, uint32_t duration_ms) {
    strupr(note);
    if (strcmp(note, "0") == 0) {
        buzzerDelay(duration_ms);
        return;
    }

    buzzerPeriodicMode = false;
    HAL_TIM_Base_Stop_IT(&htim10);

    float frequency;
	if ((frequency = get_frequency(note)) == -1) return;
	buzzerSetFreq(frequency);

    tim10_target1_ms = duration_ms;
    HAL_TIM_Base_Start_IT(&htim10);
}

void buzzerDelay(uint32_t duration_ms) {
    buzzerStop();
    tim10_target1_ms = duration_ms;
    HAL_TIM_Base_Start_IT(&htim10);
}

void buzzerEnablePeriodicMode1(char *note, uint32_t duration_on_ms, uint32_t duration_off_ms) {
    buzzerPattern = BUZZER_PATTERN_PLAY_NOTE;
    buzzerPeriodicMode = true;
    buzzerActivePeriod = true;
    HAL_TIM_Base_Stop_IT(&htim10);

    strupr(note);
    float frequency = get_frequency(note);
	buzzerSetFreq(frequency);

    tim10_target1_ms = duration_on_ms;
    tim10_target2_ms = duration_off_ms;
    HAL_TIM_Base_Start_IT(&htim10);
}

void buzzerEnablePeriodicMode2(BuzzerPattern_t pattern, uint32_t period_ms) {
    tim11_target_ms = period_ms;
    tim11_ms = 0;
    buzzerPattern = pattern;
    HAL_TIM_Base_Start_IT(&htim11);
}

void buzzerDisablePeriodicMode2() {
    tim11_ms = 0;
    HAL_TIM_Base_Stop_IT(&htim11);
}

void buzzerPlayPattern(BuzzerPattern_t pattern) {

#ifdef BUZZER_BEEP_CODES
    if (pattern > sizeof(melodyList) / sizeof(Melody_list_t)) {
        if (primaryActive) {
            pattern = primaryPattern;
            if (secondaryPattern != BUZZER_PATTERN_STOP) {
                primaryActive = false;
            }
        } else {
            pattern = secondaryPattern;
            if (secondaryPattern != BUZZER_PATTERN_STOP) {
                primaryActive = true;
            }
        }
    }
#endif

    buzzerPattern = pattern;
    buzzerPeriodicMode = false;
    buzzerEventNumber = 0;
    HAL_TIM_Base_Stop_IT(&htim10);

    tim10_target1_ms = 0;
    HAL_TIM_Base_Start_IT(&htim10);
}

void buzzerPlaySecondaryStatus(sm_state_t flight_state) {
    #ifndef BUZZER_BEEP_CODES
    return;
    #endif
    BuzzerPattern_t pattern;

    switch (flight_state) {
    case STATE_FAULT:
        pattern = BUZZER_PATTERN_CODE_SECONDARY_FAULT;
        break;
    case STATE_BATTERY_LOW:
        pattern = BUZZER_PATTERN_CODE_SECONDARY_BATTERY_LOW;
        break;
    case STATE_BATTERY_CRITICAL:
        pattern = BUZZER_PATTERN_CODE_SECONDARY_BATTERY_CRITICAL;
        break;
    default:
        pattern = BUZZER_PATTERN_STOP;
        break;
    }

    buzzerStop();
    secondaryPattern = pattern;
    buzzerPlayPattern(pattern);
    if (pattern != BUZZER_PATTERN_STOP)
        buzzerEnablePeriodicMode2(pattern, BUZZER_BEEP_REPEAT_TIME);
}

// Function to get the frequency by note name
float get_frequency(const char *note_name) {
    for (uint16_t i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
        if (strcmp(notes[i].name, note_name) == 0) {
            return notes[i].frequency;
        }
    }
    return -1.0f;  // Return -1 if the note isn't found
}

void runMelodyTable(BuzzerPattern_t pattern, uint8_t event_num) {
    uint8_t num = pattern - 1;
    if (event_num >= melodyList[num].num_entries) {
        buzzerStopPeriodic();
        return;
    }

    buzzerPlayNote(melodyList[num].melody_table[event_num].note, melodyList[num].melody_table[event_num].duration);
}

void buzzerPatternTable(BuzzerPattern_t pattern, uint8_t event_num) {
    if (pattern <= sizeof(melodyList) / sizeof(Melody_list_t)) {
        runMelodyTable(pattern, event_num);
        return;
    }

#ifndef BUZZER_BEEP_CODES
    return;
#endif

    uint8_t* beeps;
    uint32_t size;
    char note[3];

    switch (pattern) {
    case BUZZER_PATTERN_CODE_PRIMARY_ABORT:
        beeps = BEEP_CODE_PRIMARY_ABORT_TABLE;
        size = sizeof(BEEP_CODE_PRIMARY_ABORT_TABLE);
        break;
    case BUZZER_PATTERN_CODE_PRIMARY_ARMED:
        beeps = BEEP_CODE_PRIMARY_ARMED_TABLE;
        size = sizeof(BEEP_CODE_PRIMARY_ARMED_TABLE);
        break;
    case BUZZER_PATTERN_CODE_PRIMARY_RECOVERY:
        beeps = BEEP_CODE_PRIMARY_RECOVERY_TABLE;
        size = sizeof(BEEP_CODE_PRIMARY_RECOVERY_TABLE);
        break;
    case BUZZER_PATTERN_CODE_SECONDARY_FAULT:
        beeps = BEEP_CODE_SECONDARY_FAULT_TABLE;
        size = sizeof(BEEP_CODE_SECONDARY_FAULT_TABLE);
        break;
    case BUZZER_PATTERN_CODE_SECONDARY_BATTERY_LOW:
        beeps = BEEP_CODE_SECONDARY_BATTERY_LOW_TABLE;
        size = sizeof(BEEP_CODE_SECONDARY_BATTERY_LOW_TABLE);
        break;
    case BUZZER_PATTERN_CODE_SECONDARY_BATTERY_CRITICAL:
        beeps = BEEP_CODE_SECONDARY_BATTERY_CRITICAL_TABLE;
        size = sizeof(BEEP_CODE_SECONDARY_BATTERY_CRITICAL_TABLE);
        break;
    default:
        return;
    }

    if (event_num >= size * 2) {
        buzzerStopPeriodic();
        return;
    }

    if (event_num % 2 == 0) {
        uint8_t beep_pos = event_num / 2;
        uint32_t duration;

        if (beeps[beep_pos] == 0) {
            duration = BUZZER_BEEP_SHORT;
            strcpy(note, BUZZER_BEEP_LOW);
        } else if (beeps[beep_pos] == 1) {
            duration = BUZZER_BEEP_LONG;
            strcpy(note, BUZZER_BEEP_LOW);
        } else if (beeps[beep_pos] == 2) {
            duration = BUZZER_BEEP_SHORT;
            strcpy(note, BUZZER_BEEP_HIGH);
        } else if (beeps[beep_pos] == 3) {
            duration = BUZZER_BEEP_LONG;
            strcpy(note, BUZZER_BEEP_HIGH);
        } else {
            return;
        }

        buzzerPlayNote(note, duration);
    } else {
        buzzerDelay(BUZZER_BEEP_DELAY);
    }
}