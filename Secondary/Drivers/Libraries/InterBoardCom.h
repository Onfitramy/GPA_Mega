#ifndef InterBoardCom_H_
#define InterBoardCom_H_

#include "stm32f4xx_hal.h"
#include "Packets.h"

#define INTERBOARD_BUFFER_SIZE 32

//The Bitfield describe what should be expected and done with the data received
typedef enum {
    // Status Packets:
    InterBoardPACKET_ID_EMPTY = 0x00,          // No packet
    InterBoardPACKET_ID_SELFTEST = 0x01,        // Used to check if the communication is working
    InterBoardPACKET_ID_DataAck = 0x02,         // Acknowledge data reception
    InterBoardPACKET_ID_Echo = 0x03,            // Echo back received data for testing

    // Command Packets:
    InterBoardPACKET_ID_DataLoadFLASH = 0x11,   // Load data from the flash memory
    InterBoardPACKET_ID_ResetFLASH = 0x12,      // Reset the flash memory ~40s
    InterBoardPACKET_ID_DataLoadSD = 0x13,      // Load data from the SD card
    InterBoardPACKET_ID_DataRequest = 0x14,     // Request data from the Groundstation

    // Data Packets:
    InterBoardPACKET_ID_DataSaveFLASH = 0x21,   // Save data to the flash memory
    InterBoardPACKET_ID_DataSaveSD = 0x22,      // Save data to the SD card
    InterBoardPACKET_ID_DataSend = 0x23,        // Send data to the Groundstation via NRF/XBee
    InterBoardPACKET_ID_DataSaveFLASHSend = 0x24,    // Save data to the flash memory and send it to the Groundstation via NRF/XBee
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

void InterBoardCom_ReactivateDMAReceive(void);

void InterBoardCom_SendPacket(InterBoardPacket_t *packet);
void InterBoardCom_SendTestPacket(void);

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