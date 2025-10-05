#ifndef InterBoardCom_H_
#define InterBoardCom_H_

#include "stm32f4xx_hal.h"
#include "Packets.h"

#define INTERBOARD_BUFFER_SIZE 32

//The Bitfield describe what should be expected and done with the data received.
typedef enum {
    // Status Packets:
    // Operation Types (bits 0-2, lower nibble)
    INTERBOARD_OP_NONE         = 0x00,
    INTERBOARD_OP_SAVE_SEND     = 0x01,  // Save data or send it via radio
    INTERBOARD_OP_LOAD_REQUEST  = 0x02,  // Load data  or request it from another board
    INTERBOARD_OP_CMD           = 0x04,  // Command operation

    // Target Types (bits 3-6, upper nibble)
    INTERBOARD_TARGET_NONE = 0x00,
    INTERBOARD_TARGET_FLASH = 0x08,
    INTERBOARD_TARGET_SD    = 0x10,
    INTERBOARD_TARGET_RADIO = 0x20,
    INTERBOARD_TARGET_MCU   = 0x40,
    INTERBOARD_TARGET_FOLLOWING = 0x80, // Indicates more packets to follow

    // Unusual combined types
    INTERBOARD_OP_DEBUG_VIEW = INTERBOARD_OP_SAVE_SEND | INTERBOARD_TARGET_NONE, // Send for debugging to PC
} InterBoardPacketID_t;

//Wrapper for the DataPacket to be used to send via SPI1
#pragma pack(push, 1)
typedef struct {
    uint8_t InterBoardPacket_ID;
    uint8_t Data[32];
} InterBoardPacket_t;
#pragma pack(pop)

typedef struct {
    InterBoardPacket_t buffer[INTERBOARD_BUFFER_SIZE];
    volatile uint16_t head;      // Write index
    volatile uint16_t tail;      // Read index
    volatile uint16_t count;     // Number of items in buffer
} CircularBuffer_t;


void InterBoardCom_ActivateReceive(void);

InterBoardPacket_t InterBoardCom_ReceivePacket(void);

void InterBoardCom_ParsePacket(InterBoardPacket_t packet);
void InterBoardCom_EvaluateCommand(DataPacket_t *dataPacket);

void InterBoardCom_ReactivateDMAReceive(void);
void InterBoardCom_command_acknowledge(uint8_t command_target, uint8_t command_id, uint8_t status);

void InterBoardCom_SendPacket(InterBoardPacket_t *packet);
void InterBoardCom_SendTestPacket(void);
void InterBoardCom_SendDataPacket(InterBoardPacketID_t Inter_ID, DataPacket_t *packet);
uint8_t InterBoard_CheckCRC(DataPacket_t *packet);

void InterBoardCom_Init(void);
InterBoardPacket_t InterBoardCom_CreatePacket(InterBoardPacketID_t ID);
void InterBoardCom_FillRaw(InterBoardPacket_t *packet, int num, ...);
void InterBoardCom_FillData(InterBoardPacket_t *packet, DataPacket_t *data_packet);

void CircBuffer_Init(CircularBuffer_t* cb);
uint8_t CircBuffer_Push(CircularBuffer_t* cb, InterBoardPacket_t* packet);
uint8_t CircBuffer_Pop(CircularBuffer_t* cb, InterBoardPacket_t* packet);
uint8_t CircBuffer_IsEmpty(CircularBuffer_t* cb);
uint8_t CircBuffer_IsFull(CircularBuffer_t* cb);
uint16_t CircBuffer_Count(CircularBuffer_t* cb);
void CircBuffer_Clear(CircularBuffer_t* cb);

#endif /* InterBoardCom_H_ */