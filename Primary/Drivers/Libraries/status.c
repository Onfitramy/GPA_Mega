#include "status.h"

float LED_pulse = 0;
uint8_t LED_blink = 0;

void Hex_to_RGB(uint32_t hex, uint8_t *R, uint8_t *G, uint8_t *B) {
    *R = (hex & 0xFF0000) >> 16;
    *G = (hex & 0x00FF00) >> 8;
    *B = (hex & 0x0000FF);
}

// color1 matches pulse function
void SetLED_slide(color_handle color1, color_handle color2) {
    uint8_t colors[6];
    uint8_t R, G, B;
    float weighing_factor = (float)LED_pulse / 255.f;
    Hex_to_RGB(color1, &colors[0], &colors[1], &colors[2]);
    Hex_to_RGB(color2, &colors[3], &colors[4], &colors[5]);
    R = weighing_factor * colors[0] + (1 - weighing_factor) * colors[3];
    G = weighing_factor * colors[1] + (1 - weighing_factor) * colors[4];
    B = weighing_factor * colors[2] + (1 - weighing_factor) * colors[5];
    Set_LED(R, G, B);
    WS2812_Send();
}

void SetLED_color(color_handle color) {
    uint8_t R, G, B;
    Hex_to_RGB(color, &R, &G, &B);
    Set_LED(R, G, B);
    WS2812_Send();
}

void SetLED_blink(color_handle color) {
    if(LED_blink) SetLED_color(color);
    else SetLED_color(COLOR_OFF);
}  

void SetLED_pulse(color_handle color) {
    uint8_t R, G, B;
    float brightness = (float)LED_pulse / 255.f;
    Hex_to_RGB(color, &R, &G, &B);
    R *= brightness;
    G *= brightness;
    B *= brightness;
    Set_LED(R, G, B);
    WS2812_Send();
}


void ShowStatus(int8_t status, float freq_cycle, float freq_call) {
    float freq_ratio = freq_cycle / freq_call * 510.f;
    if((LED_blink == 0) && (LED_pulse + freq_ratio <= 255)) LED_pulse += freq_ratio;
    else if((LED_blink == 1) && (LED_pulse - freq_ratio >= 0)) LED_pulse -= freq_ratio;
    else LED_blink = !LED_blink;
    
    switch(status) {
        case STATE_FLIGHT_ABORT:
            SetLED_pulse(COLOR_RED);
            break;
        case STATE_FLIGHT_STARTUP:
            SetLED_pulse(COLOR_PINK);
            break;
        case STATE_FLIGHT_INIT:
            SetLED_pulse(COLOR_BLUE);
            break;
        case STATE_FLIGHT_GNC_ALIGN:
            SetLED_pulse(COLOR_GREEN);
            break;
        case STATE_FLIGHT_CHECKOUTS:
            SetLED_pulse(COLOR_VIOLET);
            break;
        case STATE_FLIGHT_ARMED:
            SetLED_slide(COLOR_WHITE, COLOR_ORANGE);
            break;
        case STATE_FLIGHT_BURN:
            SetLED_color(COLOR_YELLOW);
            break;
        case STATE_FLIGHT_COAST:
            SetLED_color(COLOR_CYAN);
            break;
        case STATE_FLIGHT_DESCEND_UNBRAKED:
            SetLED_slide(COLOR_CYAN, COLOR_PINK);
            break;
        case STATE_FLIGHT_DESCEND_DROGUE:
            SetLED_pulse(COLOR_LIME);
            break;
        case STATE_FLIGHT_DESCEND_MAIN:
            SetLED_slide(COLOR_LIME, COLOR_BLUE);
            break;
        case STATE_FLIGHT_LANDED:
            //SetLED_color(COLOR_OFF);
            SetLED_blink(COLOR_GREEN);
            break;

        case STATE_TEST_INIT:
            SetLED_blink(COLOR_BLUE);
            break;
        case STATE_TEST_CALIB:
            SetLED_blink(COLOR_VIOLET);
            break;
    }
}

void tasksStatus(void) {
    UBaseType_t uxTaskCount = uxTaskGetNumberOfTasks();
    TaskStatus_t *pxTaskStatusArray = pvPortMalloc(uxTaskCount * sizeof(TaskStatus_t));
    if (pxTaskStatusArray != NULL) {
        uxTaskCount = uxTaskGetSystemState(pxTaskStatusArray, uxTaskCount, NULL);
        for (UBaseType_t i = 0; i < uxTaskCount; i++) {
            // Process each task's status information
            // Example: Log task name and state
            // printf("Task: %s, State: %d\n", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].eCurrentState);
        }
        vPortFree(pxTaskStatusArray);
    }
}