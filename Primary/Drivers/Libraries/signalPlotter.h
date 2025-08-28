#ifndef signalPlotter_H_
#define signalPlotter_H_

#include "stm32h7xx_hal.h"

//#define SIGNAL_PLOTTER_OUT_1 // position ekf testing
//#define SIGNAL_PLOTTER_OUT_2 // raw sensor data
#define SIGNAL_PLOTTER_OUT_3 // orientation ekf testing

// actually performs send operation to signal plotter
// (is done automatically)
void signalPlotter_executeTransmission(uint32_t millisTime);

// sets the name of a signal-id (id 0...31)
void signalPlotter_setSignalName(uint8_t id,char *name);
// sends the current value of a signal (id 0...31) to the signal plotter
void signalPlotter_sendData(uint8_t id, float value);

// starts a timer for measuring the time between start and stop
void TimeMeasureStart(void);

// stops the timer and sends the time to the signal plotter
void TimeMeasureStop(void);
#endif /* signalPlotter_H_ */
