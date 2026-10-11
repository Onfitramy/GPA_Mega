#ifndef SPI2QUEUE_H
#define SPI2QUEUE_H

#include <stdbool.h>

#include "packets.h"

typedef enum __attribute__((packed)) {
    SPI2_WriteType_DataPackets,
    SPI2_WriteType_Raw,
} SPI2_W25Q_WriteType_t;

typedef enum __attribute__((packed)) {
    SPI2_CopyLogsTarget_SD,
    SPI2_CopyLogsTarget_Serial,
} SPI2_W25Q_CopyLogsTarget_t;

typedef enum __attribute__((packed)) {
    SPI2_SD_MountType_Mount,
    SPI2_SD_MountType_Unmount,
} SPI2_SD_MountType_t;

typedef enum __attribute__((packed)) {
    SPI2_Action_W25Q_Write,
    SPI2_Action_W25Q_Read,
    SPI2_Action_W25Q_Erase,
    SPI2_Action_W25Q_SetWriteStatus,
    SPI2_Action_W25Q_WriteConfig,
    SPI2_Action_W25Q_CopyLogs,
    SPI2_Action_SD_MountOperation,
    SPI2_Action_SD_SelfTest,
} SPI2_Action_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t* data;
    uint32_t size;
    SPI2_W25Q_WriteType_t type;
} SPI2_W25Q_Write_t;

typedef struct {
    // request data
    uint32_t start_page;
    uint8_t offset;
    uint32_t size;
    // response data
    uint8_t* data;
    bool finished;
} SPI2_W25Q_Read_t;

typedef struct {
    uint32_t first_sector;
    uint32_t last_sector;
} SPI2_W25Q_Erase_t;

typedef struct {
    bool write_logs;
} SPI2_W25Q_WriteStatus_t;

typedef struct {
    uint16_t max_page;
    SPI2_W25Q_CopyLogsTarget_t target;
} SPI2_W25Q_CopyLogs;

typedef struct {
    SPI2_SD_MountType_t type;
} SPI2_SD_MountOperation_t;

typedef struct {
    uint8_t result;
    bool finished;
} SPI2_SD_SelfTest_t;

typedef union {
    SPI2_W25Q_Write_t write;
    SPI2_W25Q_Read_t* read;
    SPI2_W25Q_Erase_t erase;
    SPI2_W25Q_WriteStatus_t write_status;
    SPI2_W25Q_CopyLogs copy_logs;
    SPI2_SD_MountOperation_t mount_operation;
    SPI2_SD_SelfTest_t* self_test;
} SPI2_ActionPayload_u;

typedef struct {
    SPI2_Action_t action;
    SPI2_ActionPayload_u payload;
} SPI2_ActionPacket_t;
#pragma pack(pop)

SPI2_ActionPacket_t SPI2_CreatePacket(SPI2_Action_t action, SPI2_ActionPayload_u);
uint8_t SPI2_QueueAction(SPI2_ActionPacket_t action_packet);
uint8_t SPI2_QueueWrite(uint8_t* data, uint32_t size, SPI2_W25Q_WriteType_t type);
uint8_t SPI2_QueueRead(SPI2_W25Q_Read_t* read_data);
uint8_t SPI2_QueueErase(uint32_t first_sector, uint32_t last_sector);
uint8_t SPI2_QueueSetWriteStatus(bool write_logs);
uint8_t SPI2_QueueWriteConfig();
uint8_t SPI2_QueueCopyLogs(uint16_t max_page, SPI2_W25Q_CopyLogsTarget_t target);
uint8_t SPI2_QueueMountOperation(SPI2_SD_MountType_t type);
uint8_t SPI2_QueueSelfTest(SPI2_SD_SelfTest_t* self_test_data);
uint8_t SPI2_QueueBlockingRead(SPI2_W25Q_Read_t* read_data);
void SPI2_HandleAction(SPI2_ActionPacket_t action_packet);

#endif //SPI2QUEUE_H
