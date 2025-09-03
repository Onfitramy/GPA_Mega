#ifndef XBEE_H_
#define XBEE_H_

#include "stm32h7xx_hal.h"

void XBee_TestDeviceIdentifier();
void XBee_GetTemperature();
void XBee_Broadcast(uint8_t* data, uint16_t length);
void XBee_Transmit(uint8_t* data, uint16_t length, uint64_t destinationAddress);
void XBee_Receive(uint8_t* response_buffer);

#endif /* XBEE_H_ */