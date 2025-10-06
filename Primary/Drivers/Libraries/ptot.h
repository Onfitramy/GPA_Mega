#ifndef PTOT_H_
#define PTOT_H_

#include "main.h"
#include "stdbool.h"

typedef struct {
    float pressure;
    float temperature;
} ptotcb_handle_t;

extern SPI_HandleTypeDef hspi2;

extern ptotcb_handle_t ptot_data;

#define OUTPUT_MAX      14745
#define OUTPUT_MIN      1638
#define PRESSURE_MAX    160000
#define PRESSURE_MIN    0

bool ptot_readData(float *pressure, float *temperature);

#endif