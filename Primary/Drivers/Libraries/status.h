#ifndef STATUS_H
#define STATUS_H

#include "main.h"
#include "ws2812.h"

typedef uint8_t device_handle;
typedef uint32_t color_handle;

void ShowStatus(device_handle device, int8_t status, float freq_cycle, float freq_call);

void SetLED_color(color_handle color);

#define RGB_PRIMARY     1
#define RGB_SECONDARY   2
#define LED_PRIMARY     3
#define LED_SECONDARY   4
#define BUZZER          5

#define STATUS_ERROR_STARTUP -4
#define STATUS_ERROR_OTHER  -3
#define STATUS_ERROR_MEMS   -2
#define STATUS_ERROR_FATAL  -1
#define STATUS_STARTUP      0
#define STATUS_GNSS_ALIGN   1
#define STATUS_STANDBY      2
#define STATUS_GNSS_2D      3

#define COLOR_OFF       0x000000
#define COLOR_RED       0xFF0000
#define COLOR_ORANGE    0xFF3F00
#define COLOR_YELLOW    0xFFFF00
#define COLOR_LIME      0x3FFF00
#define COLOR_GREEN     0x00FF00
#define COLOR_CYAN      0x00FFFF
#define COLOR_BLUE      0x0000FF
#define COLOR_VIOLET    0x3F00FF
#define COLOR_MAGENTA   0xFF00FF
#define COLOR_PINK      0xFF003F
#define COLOR_WHITE     0xFFFFFF

#endif