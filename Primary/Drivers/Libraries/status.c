#include "status.h"

float LED_pulse = 0;
uint8_t LED_blink = 0;

void ShowStatus(device_handle device, int8_t status, float freq_cycle, float freq_call) {
    float freq_ratio = freq_cycle / freq_call * 510.f;
    if((LED_blink == 0) && (LED_pulse + freq_ratio <= 255)) LED_pulse += freq_ratio;
    else if((LED_blink == 1) && (LED_pulse - freq_ratio >= 0)) LED_pulse -= freq_ratio;
    else LED_blink = !LED_blink;

    if(device == RGB_PRIMARY) {

        switch(status) {
            case -4: // Selftest, error
                SetLED_slide(COLOR_RED, COLOR_ORANGE);
                break;
            case -3: // Error, other
                SetLED_blink(COLOR_RED);
                break;
            case -2: // MEMS Sensor Error
                SetLED_pulse(COLOR_RED);
                break;
            case -1: // Fatal Error, Hard Fault
                SetLED_color(COLOR_RED);
                break;
            case 0: // Startup
                SetLED_pulse(COLOR_MAGENTA);
                break;
            case 1: // GNSS Alignment
                SetLED_pulse(COLOR_BLUE);
                break;
            case 2: // Standby
                SetLED_color(COLOR_GREEN);
                break;
            case 3: // GNSS 2D Fix
                SetLED_slide(COLOR_BLUE, COLOR_GREEN);
                break;
            case 4: // Servo moving
                SetLED_slide(COLOR_YELLOW, COLOR_GREEN);
                break;
            case 5: // Move complete
                SetLED_color(COLOR_CYAN);
                break;
        }
    }
    if(device == RGB_SECONDARY) {
        
    }
    if(device == LED_PRIMARY) {

        HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
    
    }
    if(device == LED_SECONDARY) {
        
    }
    if(device == BUZZER) {
        
    }

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

void Hex_to_RGB(uint32_t hex, uint8_t *R, uint8_t *G, uint8_t *B) {
    *R = (hex & 0xFF0000) >> 16;
    *G = (hex & 0x00FF00) >> 8;
    *B = (hex & 0x0000FF);
}

