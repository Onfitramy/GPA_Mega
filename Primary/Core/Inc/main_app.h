#ifndef MAIN_APP_H
#define MAIN_APP_H

#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"

typedef enum {
    APP_MODE_NORMAL = 0,
    APP_MODE_TEST_IDLE,
    APP_MODE_TEST_RUNNING,
    APP_MODE_TEST_SCENARIO,
} app_mode_t;

extern volatile app_mode_t app_mode;

void Startup();

void Task1000Hz_Step(void *argument);

void Task100Hz_Step(void *argument);

void Task10Hz_Step(void *argument);

void InterruptTask(void *argument);

void USBTask(void *argument);

#endif /* MAIN_APP_H */