#include "xBee.h"

#define XBEE_CS_GPIO_PIN GPIO_PIN_4
#define XBEE_CS_GPIO_PORT GPIOD

extern UART_HandleTypeDef huart1;

uint8_t XBee_checkCRC(uint8_t* data, uint16_t length, uint8_t crc);

/**
 * @brief Constructs an XBee Transmit Request frame.
 *
 * @param frame Pointer to a buffer to be populated.
 * @param frame_id The frame ID to assign to this frame (used for response tracking).
 * @param dest_addr The 64-bit destination address for the frame (set as 0 to indicate broadcast).
 * @param frame_payload Pointer to the payload data to be transmitted.
 * @param payload_length Length of the payload data in bytes.
 */
void constructTransmitFrame(uint8_t* frame, uint8_t frame_id, uint64_t dest_addr, uint8_t* frame_payload, uint16_t payload_length)
{
    if (dest_addr == 0) {
        dest_addr = 0x000000000000FFFF; // Broadcast address
    }

    frame[0] = 0x7E;
    frame[1] = (payload_length + 14) >> 8; // High byte
    frame[2] = (payload_length + 14) & 0xFF; // Low byte
    frame[3] = 0x10; // AT Command frame
    frame[4] = frame_id;
    frame[5] = (dest_addr >> 56) & 0xFF;
    frame[6] = (dest_addr >> 48) & 0xFF;
    frame[7] = (dest_addr >> 40) & 0xFF;
    frame[8] = (dest_addr >> 32) & 0xFF;
    frame[9] = (dest_addr >> 24) & 0xFF;
    frame[10] = (dest_addr >> 16) & 0xFF;
    frame[11] = (dest_addr >> 8) & 0xFF;
    frame[12] = dest_addr & 0xFF;
    frame[13] = 0xFF;
    frame[14] = 0xFE;
    frame[15] = 0x00; // Maximum hops
    frame[16] = 0x40; // Deleivery method point to multipoint
    for (uint16_t i = 0; i < payload_length; i++) {
        frame[17 + i] = frame_payload[i];
    }
    // Calculate and append checksum
    uint64_t checksum = 0;
    // Add all bytes of the packet, except the start delimiter 0x7E and the length (the second and third bytes).
    for (uint16_t i = 3; i < 17 + payload_length; i++) {
        checksum += frame[i];
    }
    //Keep only the lowest 8 bits from the result and subtract this from 0xFF.
    frame[17 + payload_length] = 0xFF - (checksum & 0xFF);

    XBee_checkCRC(&frame[3], 14 + payload_length , frame[17 + payload_length]);
}

uint8_t XBee_calcCRC(uint8_t* data, uint16_t length)
{
    uint64_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return 0xFF - (checksum & 0xFF);
}

uint8_t XBee_checkCRC(uint8_t* data, uint16_t length, uint8_t crc)
{
    uint64_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    if (((checksum + crc) & 0xFF) == 0xFF) {
        return 1; // Valid CRC
    }
    uint8_t corr_checksum = 0xFF - (checksum & 0xFF);
    return 0; // Invalid CRC
}

void constructATFrame(xbee_at_cmd_t* frame, uint8_t frame_id, uint16_t ATCommand, uint8_t *payload, uint16_t payload_length)
{

    frame->start_delimiter = 0x7E;
    frame->frame_length[0] = (payload_length + 4) >> 8; // High byte
    frame->frame_length[1] = (payload_length + 4) & 0xFF; // Low byte
    frame->frame_type = 0x08; // AT Command frame
    frame->frame_id = frame_id;
    frame->at_cmd[0] = (ATCommand >> 8) & 0xFF;
    frame->at_cmd[1] = ATCommand & 0xFF;
    for (uint8_t i = 0; i < payload_length; i++) {
        frame->parameters[i] = payload[i];
    }
    // Calculate and append checksum
    uint8_t checksum = 0;
    //Add all bytes of the packet, except the start delimiter 0x7E and the length (the second and third bytes).
    checksum += frame->frame_type;
    checksum += frame->frame_id;
    checksum += frame->at_cmd[0];
    checksum += frame->at_cmd[1];
    for (uint8_t i = 0; i < frame->frame_length[1] - 4; i++) {
        checksum += frame->parameters[i];
    }
    //Keep only the lowest 8 bits from the result and subtract this from 0xFF.
    frame->checksum = 0xFF - (checksum & 0xFF);
    //Also set the checksum in the raw frame for transmission
    frame->parameters[payload_length] = frame->checksum;
}

uint8_t debug_temp;
uint8_t UART_LOCK = 0;
uint8_t UART_LOCK_ERROR = 0;
void sendAPIFrame(uint8_t* frame, uint16_t frame_length)
{

    HAL_StatusTypeDef res = HAL_UART_Transmit_IT(&huart1, frame, frame_length + 4); //Convert frame_length to bytes
    res = 0;
}

uint8_t frame[XBEE_MAX_FRAME_SIZE];
uint8_t response[XBEE_MAX_FRAME_SIZE];

void XBee_TestDeviceIdentifier()
{
    xbee_at_cmd_t frame;
    constructATFrame(&frame, 0x01, ('D' << 8) | 'D', NULL, 0);
    sendAPIFrame((uint8_t*)&frame, frame.frame_length[0] | (frame.frame_length[1] << 8));
}

void XBee_ToCommandMode()
{
    HAL_Delay(1100); //Wait for 1 second of silence
    uint8_t command_sequence[] = "+++";
    HAL_UART_Transmit(&huart1, command_sequence, 3, HAL_MAX_DELAY);
    HAL_Delay(1100); //Wait for 1 second of silence
}

//Change API mode while in Passthrough mode
void XBee_SetAPIMode(uint8_t mode) // mode: 0 = Passthrough 1 = API without escapes, 2 = API with escapes
{
    XBee_ToCommandMode();
    uint8_t command_sequence[7];
    command_sequence[0] = 'A';
    command_sequence[1] = 'T';
    command_sequence[2] = 'A';
    command_sequence[3] = 'P';
    command_sequence[4] = '=';
    command_sequence[5] = '0' + mode; //Convert mode to ASCII
    command_sequence[6] = '\r';
    HAL_UART_Transmit(&huart1, command_sequence, 7, HAL_MAX_DELAY);
    HAL_Delay(1000);
    command_sequence[0] = 'A';
    command_sequence[1] = 'T';
    command_sequence[2] = 'W';
    command_sequence[3] = 'R';
    command_sequence[4] = '\r';
    HAL_UART_Transmit(&huart1, command_sequence, 5, HAL_MAX_DELAY);
}

xbee_at_cmd_t at_frame;
void XBee_GetTemperature()
{
    constructATFrame(&at_frame, 0x01, ('T' << 8) | 'P', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}
uint8_t xbee_frame_buffer[XBEE_MAX_FRAME_SIZE+4];
void XBee_Transmit(uint8_t* data, uint16_t length, uint64_t destinationAddress)
{
    if (UART_LOCK) {
        UART_LOCK_ERROR += 1;
        return; // UART is busy
    }
    UART_LOCK = 1;
    constructTransmitFrame(xbee_frame_buffer, 0x01, destinationAddress, data, length);
    sendAPIFrame(xbee_frame_buffer, xbee_frame_buffer[1] << 8 | xbee_frame_buffer[2]);
}

void XBee_ReceivedErrorCount()
{
    constructATFrame(&at_frame, 0x01, ('E' << 8) | 'R', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_TransmitErrorCount()
{
    constructATFrame(&at_frame, 0x01, ('T' << 8) | 'R', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_changeBaudRate(uint32_t baudrate)
{
    uint8_t baud_payload[1];
    switch(baudrate) {
        case 9600:
            baud_payload[0] = 0x03;
            break;
        case 19200:
            baud_payload[0] = 0x04;
            break;
        case 38400: 
            baud_payload[0] = 0x05;
            break;
        case 57600:
            baud_payload[0] = 0x06;
            break;
        case 115200:
            baud_payload[0] = 0x07;
            break;
        case 230400:
            baud_payload[0] = 0x08;
    }
    constructATFrame(&at_frame, 0x01, ('B' << 8) | 'D', baud_payload, 1);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
    HAL_Delay(100);
    HAL_UART_Abort(&huart1);
    huart1.Init.BaudRate = baudrate;
    HAL_UART_Init(&huart1);
}

void XBee_ReadChannelMask()
{
    constructATFrame(&at_frame, 0x01, ('C' << 8) | 'M', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_ReadDataRate()
{
    constructATFrame(&at_frame, 0x01, ('B' << 8) | 'R', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

// Data Rate: 0 = 10kb/s, 1 = 80kb/s
void XBee_changeDataRate(uint8_t dataRate)
{
    uint8_t dataRate_payload[1];
    dataRate_payload[0] = dataRate;
    constructATFrame(&at_frame, 0x01, ('B' << 8) | 'R', dataRate_payload, 1);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_changePowerLevel(uint8_t powerLevel)
{
    uint8_t powerLevel_payload[1];
    powerLevel_payload[0] = powerLevel;
    constructATFrame(&at_frame, 0x01, ('B' << 8) | 'P', powerLevel_payload, 1);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_SaveSettings()
{
    constructATFrame(&at_frame, 0x01, ('W' << 8) | 'R', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_ReadHardwareAddress()
{
    constructATFrame(&at_frame, 0x01, ('S' << 8) | 'H', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
    HAL_Delay(100);
    constructATFrame(&at_frame, 0x01, ('S' << 8) | 'L', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

//void XBee_BufferT