#include "xBee.h"

#define XBEE_CS_GPIO_PIN GPIO_PIN_4
#define XBEE_CS_GPIO_PORT GPIOD

extern UART_HandleTypeDef huart1;


/**
 * @brief Constructs an XBee Transmit Request frame.
 *
 * @param frame Pointer to the xbee_tx_req_t structure to be populated.
 * @param frame_id The frame ID to assign to this frame (used for response tracking).
 * @param dest_addr The 64-bit destination address for the frame (set as 0 to indicate broadcast).
 * @param frame_payload Pointer to the payload data to be transmitted.
 * @param payload_length Length of the payload data in bytes.
 */
void constructTransmitFrame(xbee_tx_req_t* frame, uint8_t frame_id, uint64_t dest_addr, uint8_t* frame_payload, uint16_t payload_length)
{
    if (dest_addr == 0) {
        dest_addr = 0xFFFF000000000000; // Broadcast address
    }

    frame->start_delimiter = 0x7E;
    frame->frame_length[0] = (payload_length + 14) >> 8; // High byte
    frame->frame_length[1] = (payload_length + 14) & 0xFF; // Low byte
    frame->frame_type = 0x10; // AT Command frame
    frame->frame_id = frame_id;
    frame->dest_addr = dest_addr;
    frame->reserved_16 = 0xFFFE; // Reserved
    frame->broadcast_radius = 0x00; // Maximum hops
    frame->options = 0x00; // No special options
    for (uint16_t i = 0; i < payload_length; i++) {
        frame->rf_data[i] = frame_payload[i];
    }
    // Calculate and append checksum
    uint64_t checksum = 0;
    // Add all bytes of the packet, except the start delimiter 0x7E and the length (the second and third bytes).
    checksum += frame->frame_type;
    checksum += frame->frame_id;
    // Add destination address (8 bytes)
    checksum += (frame->dest_addr >> 56) & 0xFF;
    checksum += (frame->dest_addr >> 48) & 0xFF;
    checksum += (frame->dest_addr >> 40) & 0xFF;
    checksum += (frame->dest_addr >> 32) & 0xFF;
    checksum += (frame->dest_addr >> 24) & 0xFF;
    checksum += (frame->dest_addr >> 16) & 0xFF;
    checksum += (frame->dest_addr >> 8) & 0xFF;
    checksum += (frame->dest_addr) & 0xFF;
    // Add reserved_16 (2 bytes)
    checksum += (frame->reserved_16 >> 8) & 0xFF;
    checksum += (frame->reserved_16) & 0xFF;
    checksum += frame->broadcast_radius;
    checksum += frame->options;
    for (uint16_t i = 0; i < payload_length; i++) {
        checksum += frame->rf_data[i];
    }
    //Keep only the lowest 8 bits from the result and subtract this from 0xFF.
    frame->checksum = 0xFF - (checksum & 0xFF);
    //Also set the checksum in the raw frame for transmission
    frame->rf_data[payload_length] = frame->checksum;
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

xbee_tx_req_t tx_req_frame;
void XBee_Transmit(uint8_t* data, uint16_t length, uint64_t destinationAddress)
{
    constructTransmitFrame(&tx_req_frame, 0x01, destinationAddress, data, length);
    sendAPIFrame((uint8_t*)&tx_req_frame, tx_req_frame.frame_length[0] << 8 | tx_req_frame.frame_length[1]);
}