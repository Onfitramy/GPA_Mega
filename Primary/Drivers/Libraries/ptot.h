#ifndef PTOT_H_
#define PTOT_H_

#include "main.h"
#include "stdbool.h"

typedef struct {
    float pressure;
    float temperature;
    bool connected;
} ptotcb_handle_t;

extern SPI_HandleTypeDef hspi2;

extern ptotcb_handle_t ptot_data;

extern HAL_StatusTypeDef PtotCB_SPI_status;

#define OUTPUT_MAX      14745.f
#define OUTPUT_MIN      1638.f
#define PRESSURE_MAX    160000.f
#define PRESSURE_MIN    0.f

bool ptot_readData(ptotcb_handle_t *PtotCB);

#endif