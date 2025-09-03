#include "xBee.h"

#define XBEE_MAX_FRAME_SIZE 100
#define XBEE_CS_GPIO_PIN GPIO_PIN_4
#define XBEE_CS_GPIO_PORT GPIOD

extern SPI_HandleTypeDef hspi6;

void constructAPIFrame(uint8_t* frame, uint8_t frame_type, uint8_t frame_id, uint16_t frame_length, uint8_t* frame_payload)
{
    frame[0] = 0x7e; //Start Delimiter
    frame[1] = ((frame_length + 2) >> 8) & 0xFF; // Length between start length and checksum
    frame[2] = (frame_length + 2) & 0xFF;
    frame[3] = frame_type; // Frame Type
    frame[4] = frame_id;
    for (uint8_t i = 0; i < frame_length; i++) {
        frame[5 + i] = frame_payload[i];
    }
    // Calculate and append checksum
    uint8_t checksum = 0;
    //Add all bytes of the packet, except the start delimiter 0x7E and the length (the second and third bytes).
    for (uint8_t i = 0; i < frame_length + 2; i++) {
        checksum += frame[i + 3];
    }
    //Keep only the lowest 8 bits from the result and subtract this from 0xFF.
    frame[frame_length + 5] = 0xFF - (checksum & 0xFF);
}

void constructATFrame(uint8_t* frame, uint8_t frame_type, uint8_t frame_id, uint16_t ATCommand, uint8_t *payload, uint16_t payload_length)
{
    uint8_t frame_payload[XBEE_MAX_FRAME_SIZE];
    frame_payload[0] = (ATCommand >> 8) & 0xFF;
    frame_payload[1] = ATCommand & 0xFF;
    for (uint8_t i = 0; i < payload_length; i++) {
        frame_payload[2 + i] = payload[i];
    }
    constructAPIFrame(frame, frame_type, frame_id, payload_length + 2, frame_payload);
}

uint8_t debug_temp;
void sendReceiveAPIFrame(uint8_t* frame, uint8_t* response_buffer, uint16_t frame_length, uint8_t response_length)
{
    HAL_GPIO_WritePin(XBEE_CS_GPIO_PORT, XBEE_CS_GPIO_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi6, frame, response_buffer, frame_length + 4 + response_length, HAL_MAX_DELAY); //Convert frame_length to bytes
    HAL_GPIO_WritePin(XBEE_CS_GPIO_PORT, XBEE_CS_GPIO_PIN, GPIO_PIN_SET);
}

uint8_t frame[XBEE_MAX_FRAME_SIZE];
uint8_t response[XBEE_MAX_FRAME_SIZE];

void XBee_TestDeviceIdentifier()
{
    constructATFrame(frame, 0x08, 0x01, ('D' << 8) | 'D', NULL, 0);
    sendReceiveAPIFrame(frame, response, frame[2], 12);
}

void XBee_GetTemperature()
{
    constructATFrame(frame, 0x08, 0x01, ('T' << 8) | 'P', NULL, 0);
    sendReceiveAPIFrame(frame, response, frame[2], 12);
    debug_temp = response[10]; // Example: Store the first byte of the response for debugging
}

void XBee_Broadcast(uint8_t* data, uint16_t length)
{
    uint8_t TransmitPayload[50];
    uint64_t destinationAddress = 0x000000000000FFFF; //Broadcast address
    TransmitPayload[0] = (destinationAddress >> 56) & 0xFF; //64-bit address high
    TransmitPayload[1] = (destinationAddress >> 48) & 0xFF;
    TransmitPayload[2] = (destinationAddress >> 40) & 0xFF;
    TransmitPayload[3] = (destinationAddress >> 32) & 0xFF;
    TransmitPayload[4] = (destinationAddress >> 24) & 0xFF;
    TransmitPayload[5] = (destinationAddress >> 16) & 0xFF;
    TransmitPayload[6] = (destinationAddress >> 8) & 0xFF;
    TransmitPayload[7] = destinationAddress & 0xFF; //64-bit address low
    TransmitPayload[8] = 0xFF; //Reserved 16
    TransmitPayload[9] = 0xFE; //Reserved 16
    TransmitPayload[10] = 0x00; //Broadcast radius
    TransmitPayload[11] = 0x00; //Options
    for (uint16_t i = 0; i < length; i++) {
        TransmitPayload[i + 12] = data[i];
    }
    constructAPIFrame(frame, 0x10, 0x01, length + 12, TransmitPayload);
    sendReceiveAPIFrame(frame, response, frame[2], 12);
}

void XBee_Transmit(uint8_t* data, uint16_t length, uint64_t destinationAddress)
{
    uint8_t TransmitPayload[50];
    TransmitPayload[0] = (destinationAddress >> 56) & 0xFF; //64-bit address high
    TransmitPayload[1] = (destinationAddress >> 48) & 0xFF;
    TransmitPayload[2] = (destinationAddress >> 40) & 0xFF;
    TransmitPayload[3] = (destinationAddress >> 32) & 0xFF;
    TransmitPayload[4] = (destinationAddress >> 24) & 0xFF;
    TransmitPayload[5] = (destinationAddress >> 16) & 0xFF;
    TransmitPayload[6] = (destinationAddress >> 8) & 0xFF;
    TransmitPayload[7] = destinationAddress & 0xFF; //64-bit address low
    TransmitPayload[8] = 0xFF; //Reserved 16
    TransmitPayload[9] = 0xFE; //Reserved 16
    TransmitPayload[10] = 0x00; //Broadcast radius
    TransmitPayload[11] = 0x00; //Options
    for (uint16_t i = 0; i < length; i++) {
        TransmitPayload[i + 12] = data[i];
    }
    constructAPIFrame(frame, 0x10, 0x01, length + 12, TransmitPayload);
    sendReceiveAPIFrame(frame, response, frame[2], 12);
}

void XBee_Receive(uint8_t* response_buffer)
{
    HAL_GPIO_WritePin(XBEE_CS_GPIO_PORT, XBEE_CS_GPIO_PIN, GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi6, response_buffer,  27, 5); //Convert frame_length to bytes
    HAL_GPIO_WritePin(XBEE_CS_GPIO_PORT, XBEE_CS_GPIO_PIN, GPIO_PIN_SET);
}