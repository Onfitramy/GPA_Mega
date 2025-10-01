#ifndef SD_H_
#define SD_H_

#include "stm32f4xx_hal.h"
#include "Packets.h"

uint8_t SD_SaveBuffer(void);
uint8_t SD_AppendDataPacketToBuffer(DataPacket_t* packet);

uint8_t SD_Mount(void);
uint8_t SD_Unmount(void);
uint8_t SD_Open(char* filename, uint8_t mode);
uint8_t SD_Read(char* buffer, uint32_t bytes_to_read);
uint8_t SD_Write(char* buffer, uint32_t bytes_to_write);
uint8_t SD_Close(void);

#endif /* SD_H_ */