#ifndef XBEE_H_
#define XBEE_H_

#include "stm32f4xx_hal.h"
#include "Packets.h"

uint8_t XBee_Init(void);
void XBee_TestDeviceIdentifier();
void XBee_SetAPIMode(uint8_t mode);
void XBee_GetTemperature();
void XBee_Broadcast(uint8_t* data, uint16_t length);
void XBee_Transmit(uint8_t* data, uint16_t length, uint64_t destinationAddress);
uint8_t XBee_QueueDataPacket(DataPacket_t* data);
void XBee_TransmitQueue(uint64_t destinationAddress);
void XBee_Receive(uint8_t* response_buffer);
void XBee_ReceivedErrorCount();
uint8_t XBee_calcCRC(uint8_t* data, uint16_t length);
void XBee_changeBaudRate(uint32_t baudrate);
void XBee_TransmitErrorCount(void);
void XBee_ReadChannelMask(void);
void XBee_ReadDataRate(void);
void XBee_changeDataRate(uint8_t dataRate);

#define XBEE_MAX_FRAME_SIZE 64
#define XBEE_MAX_RF_PAYLOAD_SIZE 256

#pragma pack(push, 1)
typedef struct {
    uint8_t start_delimiter; // 0x7E
    uint8_t frame_length[2];
    uint8_t frame_type;
    uint8_t frame_id;
    char at_cmd[2];
    uint8_t parameters[XBEE_MAX_FRAME_SIZE - 4];
    uint8_t checksum;
} xbee_at_cmd_t;

typedef struct {
    uint8_t start_delimiter; // 0x7E
    uint8_t frame_length[2];
    uint8_t frame_type;
    uint8_t frame_id;
    uint64_t dest_addr;
    uint16_t reserved_16;
    uint8_t broadcast_radius;
    uint8_t options;
    uint8_t rf_data[XBEE_MAX_FRAME_SIZE - 13];
    uint8_t checksum;
} xbee_tx_req_t;

typedef struct {
    uint8_t start_delimiter; // 0x7E
    uint8_t frame_length[2];
    uint8_t frame_data[XBEE_MAX_RF_PAYLOAD_SIZE];
    uint8_t checksum;
}  xbee_frame_t;
#pragma pack(pop)

void XBee_parseReceivedRFFrame(xbee_frame_t* frame);

#endif /* XBEE_H_ */