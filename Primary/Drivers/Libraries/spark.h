#ifndef SPARK_H_
#define SPARK_H_

#include "main.h"

extern SPI_HandleTypeDef hspi2;

extern DataPacket_t spark_data;

void SPARK_sendCommand(DataPacket_t *packet);

void SPARK_ReadData();

void SPARK_SetAngle(float angle_deg);
void SPARK_SetSpeed(float speed_deg_s);
void SPARK_ExitMode();
void SPARK_ZeroStepper();
void SPARK_FindMax();
void SPARK_TargetPositionMode(uint8_t torque_16);
void SPARK_TargetSpeedMode(uint8_t torque_16);
void SPARK_Reset();

#endif /* SPARK_H_ */