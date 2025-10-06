#ifndef SPARK_H_
#define SPARK_H_

#include "main.h"

extern SPI_HandleTypeDef hspi2;

void spark_sendCommand(DataPacket_t *packet);

#endif /* SPARK_H_ */