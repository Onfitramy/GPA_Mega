#ifndef SPI2QUEUE_H
#define SPI2QUEUE_H

#include <stdbool.h>

#include "packets.h"

typedef enum __attribute__((packed)){
    SPI2_WriteType_DataPackets,
    SPI2_WriteType_Raw,
} SPI2_WriteType_t;

typedef enum __attribute__((packed)){
    SPI2_Action_Write,
    SPI2_Action_Read,
    SPI2_Action_Erase,
    SPI2_Action_SetWriteStatus,
} SPI2_Action_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t* data;
    uint32_t size;
    SPI2_WriteType_t type;
} SPI2_Write_t;

typedef struct {
    // request data
    uint32_t start_page;
    uint8_t offset;
    uint32_t size;
    // response data
    uint8_t* data;
    bool finished;
} SPI2_Read_t;

typedef struct {
    bool write_logs;
} SPI2_WriteStatus_t;

typedef struct {
    uint32_t first_sector;
    uint32_t last_sector;
} SPI2_Erase_t;

typedef union {
    SPI2_Write_t write;
    SPI2_Read_t* read;
    SPI2_WriteStatus_t write_status;
    SPI2_Erase_t erase;
} SPI2_ActionPayload_u;

typedef struct {
    SPI2_Action_t action;
    SPI2_ActionPayload_u payload;
} SPI2_ActionPacket_t;
#pragma pack(pop)

SPI2_ActionPacket_t SPI2_CreatePacket(SPI2_Action_t action, SPI2_ActionPayload_u);
uint8_t SPI2_QueueAction(SPI2_ActionPacket_t action_packet);
uint8_t SPI2_QueueWrite(uint8_t* data_packets, uint32_t size);
uint8_t SPI2_QueueRead(SPI2_Read_t* read_data);
uint8_t SPI2_QueueErase();
uint8_t SPI2_QueueSetWriteStatus(bool write_logs);
uint8_t SPI2_QueueReadBlocking(SPI2_Read_t* read_result);
void SPI2_HandleAction(SPI2_ActionPacket_t action_packet);

#endif //SPI2QUEUE_H
