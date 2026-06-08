#ifndef MAIN_APP_H
#define MAIN_APP_H

#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"

void Startup();

void Task1000Hz(void *argument);

void Task100Hz(void *argument);

void Task10Hz(void *argument);

void InterruptTask(void *argument);

void USBTask(void *argument);

#endif /* MAIN_APP_H */