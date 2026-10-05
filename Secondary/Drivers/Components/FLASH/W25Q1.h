#include <stdbool.h>

#include "packets.h"
extern SPI_HandleTypeDef hspi2;
#define W25Q1_SPI hspi2

#define numBLOCK 256 //256 Blocks in the 128Mbit FLASH
#define PAGE_SIZE 256
#define SECTOR_SIZE 4096
#define PAGES_PER_SECTOR (SECTOR_SIZE / PAGE_SIZE)

#define CONFIG_PAGE 32 //Start of the configuration page, ends at 127->32KB
#define GPS_ASSIST_PAGE 256 //Start of the GPS assist data page, ends at 767->128KB
#define LOG_PAGE 1024 //Start of the log page, runs till the end of the flash memory ~16MB

#define ACTION_QUEUE_LENGTH 32

#define FLASH_BUFFER_SIZE (SECTOR_SIZE / sizeof(DataPacket_t))
#define PACKETS_PER_PAGE (PAGE_SIZE / sizeof(DataPacket_t))

typedef struct {
    uint8_t * const buffer;
    int head;
    int tail;
    const int maxlen;
} circ_bbuf_t;

#define CIRC_BBUF_DEF(x,y)                \
    uint8_t x##_data_space[y];            \
    circ_bbuf_t x = {                     \
        .buffer = x##_data_space,         \
        .head = 0,                        \
        .tail = 0,                        \
        .maxlen = y                       \
    }

#pragma pack(push, 1)
typedef struct {
    uint8_t ID[2]; // 2 bytes for ID AP in hex
    uint32_t curr_configPage; // Current configuration page
    uint32_t curr_configOffset; // Current configuration offset
    uint32_t curr_logPage; // Current log page
    uint32_t curr_logOffset; // Current log offset
    bool write_logs;
} W25QPage0_config_t;
#pragma pack(pop)

typedef enum {
    W25Q_State_Available,
    W25Q_State_Writing,
    W25Q_State_Reading,
    W25Q_State_Erasing,
} W25Q_State_t;

typedef struct {
    DataPacket_t* packets;
} W25Q_Write_t;

typedef struct {
    DataPacket_t* packets;
    uint32_t size;
} W25Q_Read_t;

typedef struct {
    bool write_logs;
} W25Q_WriteStatus_t;

typedef union {
    W25Q_Write_t write;
    W25Q_Read_t read;
    W25Q_WriteStatus_t write_status;
} W25Q_ActionPayload_u;

typedef enum {
    W25Q_Action_Write, // data
    W25Q_Action_Read, // data*, page
    W25Q_Action_Erase,
    W25Q_Action_SetWriteStatus,
} W25Q_Action;

typedef struct {
    W25Q_Action action;
    W25Q_ActionPayload_u payload;
} W25Q_ActionPacket_t;

// Writing and erasing can be queued, while reading can should be instantaneous => separate queue for reading?
// read function waits until action is done and is then prioritized
//
// own thread for flash queue, should I use semaphores
//
// queue takes pointer to struct

void W25Q1_Reset(void);
uint32_t W25Q1_ReadID(void);
void W25Q_Erase_Sector (uint16_t numsector);
void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
void W25Q_Write_Byte (uint32_t Addr, uint8_t data);
void W25Q_Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
void W25Q_Write_NUM (uint32_t page, uint16_t offset, float data);
float W25Q_Read_NUM (uint32_t page, uint16_t offset);
void W25Q_Write_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);
void W25Q_Read_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);
void W25Q_Chip_Erase (void);
void W25Q_updateLogPosition(void);
void W25Q_Write_Cleared(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
void W25Q_SaveToLog(uint8_t *data, uint32_t size);
void W25Q_AddFlashBufferPacket(const DataPacket_t* data_packet);
void W25Q_WritePackets(DataPacket_t* packets);
void W25Q_LoadFromLog(uint8_t *data, uint32_t size, uint32_t log_page, uint32_t log_offset);
void W25Q_GetConfig();
uint8_t W25Q_LoadLastPackets(PacketType_t *packet_types, DataPacket_t *packets, uint8_t packet_count);
void W25Q_CopyLogsToSD(uint16_t max_page);
void W25Q_CopyLogsToSerial(uint16_t max_page);

W25Q_ActionPacket_t W25Q_CreatePacket(W25Q_Action action, W25Q_ActionPayload_u);
uint8_t W25Q_QueueAction(W25Q_ActionPacket_t action_packet);
uint8_t W25Q_QueueWrite(DataPacket_t* data_packets);
uint8_t W25Q_QueueSetWriteStatus(bool write_logs);
void W25Q_HandleAction(W25Q_ActionPacket_t* action_packet);
