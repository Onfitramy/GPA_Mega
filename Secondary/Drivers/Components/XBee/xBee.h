#ifndef XBEE_H_
#define XBEE_H_

#include "stm32f4xx_hal.h"

void XBee_TestDeviceIdentifier();
void XBee_SetAPIMode(uint8_t mode);
void XBee_GetTemperature();
void XBee_Broadcast(uint8_t* data, uint16_t length);
void XBee_Transmit(uint8_t* data, uint16_t length, uint8_t* destinationAddress);
void XBee_Receive(uint8_t* response_buffer);

#define XBEE_MAX_FRAME_SIZE 64
#define XBEE_TX_HDR_SIZE 14

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
    uint8_t dest_addr[8];
    uint8_t reserved[2];
    uint8_t broadcast_radius;
    uint8_t options;
    uint8_t rf_data[XBEE_MAX_FRAME_SIZE - 13];
    uint8_t checksum;
} xbee_tx_req_t;

typedef struct {
    uint8_t start_delimiter; // 0x7E
    uint8_t frame_length[2];
    uint8_t frame_data[XBEE_MAX_FRAME_SIZE];
    uint8_t checksum;
} xbee_frame_t;
#pragma pack(pop)

#endif /* XBEE_H_ */