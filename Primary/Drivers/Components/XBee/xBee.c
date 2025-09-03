#include "xBee.h"

#define XBEE_MAX_FRAME_SIZE 100
#define XBEE_CS_GPIO_PIN GPIO_PIN_4
#define XBEE_CS_GPIO_PORT GPIOD

extern SPI_HandleTypeDef hspi6;

void constructATFrame(uint8_t* frame, uint8_t frame_type, uint8_t frame_id, uint16_t ATCommand, uint8_t *payload, uint16_t payload_length)
{
    frame[0] = 0x7e; //Start Delimiter
    frame[1] = ((payload_length + 4) >> 8) & 0xFF; // Length between start length and checksum, Frame Length != payload length
    frame[2] = (payload_length + 4) & 0xFF;
    frame[3] = frame_type; // Frame Type
    frame[4] = frame_id; 
    frame[5] = (ATCommand >> 8) & 0xFF;
    frame[6] = ATCommand & 0xFF;
    for (uint8_t i = 0; i < payload_length; i++) {
        frame[7 + i] = payload[i];
    }

    // Calculate and append checksum
    uint8_t checksum = 0;
    //Add all bytes of the packet, except the start delimiter 0x7E and the length (the second and third bytes).
    for (uint8_t i = 0; i < payload_length + 4; i++) {
        checksum += frame[i + 3];
    }
    //Keep only the lowest 8 bits from the result and subtract this from 0xFF.
    frame[payload_length + 7] = 0xFF - (checksum & 0xFF);
}

void sendReceiveATFrame(uint8_t* frame, uint16_t frame_length, uint8_t response_length)
{
    uint8_t receive_buffer[XBEE_MAX_FRAME_SIZE] = {0};
    HAL_GPIO_WritePin(XBEE_CS_GPIO_PORT, XBEE_CS_GPIO_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi6, frame, receive_buffer, frame_length + 4 + response_length, HAL_MAX_DELAY); //Convert frame_length to bytes
    //HAL_SPI_Receive(&hspi6, receive_buffer, response_length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(XBEE_CS_GPIO_PORT, XBEE_CS_GPIO_PIN, GPIO_PIN_SET);
    // Process the received data
}

void XBee_TestDeviceIdentifier()
{
    uint8_t frame[XBEE_MAX_FRAME_SIZE];
    constructATFrame(frame, 0x08, 0x01, ('D' << 8) | 'D', NULL, 0);
    sendReceiveATFrame(frame, frame[2], 12);
}