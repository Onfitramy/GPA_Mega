#ifndef InterBoardCom_H_
#define InterBoardCom_H_

#include "stm32h7xx_hal.h"

typedef enum {
    PACKET_ID_GPS = 0x01,
    PACKET_ID_IMU1 = 0x02,
    PACKET_ID_IMU2 = 0x03,
    PACKET_ID_MAG = 0x04,
    PACKET_ID_BMP = 0x05,
    PACKET_ID_CALIBRATION = 0x06,
    PACKET_ID_SELFTEST = 0x07,
    PACKET_ID_GPA = 0x08,
} PacketID_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t Packet_ID;
    uint8_t Data[16];
    uint8_t crc;
} InterBoardPacket_t;
#pragma pack(pop)

void InterBoardCom_SendTestPacket(void);

#endif /* InterBoardCom_H_ */