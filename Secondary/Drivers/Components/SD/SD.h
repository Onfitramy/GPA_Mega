#ifndef SD_H_
#define SD_H_

#include "stm32f4xx_hal.h"
#include "packets.h"

uint8_t SD_SaveBuffer(char *filename);
uint8_t SD_AppendDataPacketToBuffer(DataPacket_t* packet);
void SD_ResetBufferIndex();

uint8_t SD_Mount(void);
uint8_t SD_Unmount(void);
uint8_t SD_Open(char* filename, uint8_t mode);
uint8_t SD_Read(char* buffer, uint32_t bytes_to_read);
uint8_t SD_Write(char* buffer, uint32_t bytes_to_write);
uint8_t SD_Close(void);
uint8_t SD_SelfTest(void);

#endif /* SD_H_ */