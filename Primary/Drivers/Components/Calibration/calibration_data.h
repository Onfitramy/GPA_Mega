#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "stdbool.h"

/* ### CALIBRATION AND OFFSET DATA FOR BOARDS 1 - 5 ### */

typedef enum {
    GPA_MEGA_1 = 0,
    GPA_MEGA_2 = 1,
    GPA_MEGA_3 = 2,
    GPA_MEGA_4 = 3,
    GPA_MEGA_5 = 4,
} GPA_Mega;

typedef struct {
    float offset[3];
    float scale[3];
} CalibrationData_t;

typedef struct {
  HAL_StatusTypeDef hal_status;
  bool active;
} SensorStatus;

extern const CalibrationData_t CalibrationData[5][3];

GPA_Mega GPA_MegaFromUID(uint32_t uid[3]);

#endif // CALIBRATION_DATA_H