#include "xBee.h"
#include <string.h>

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
void constructTransmitFrame(xbee_tx_req_t* frame, uint8_t frame_id, uint8_t* dest_addr, uint8_t* frame_payload, uint16_t payload_length)
{

    frame->start_delimiter = 0x7E;
    uint16_t length = XBEE_TX_HDR_SIZE + payload_length;
    frame->frame_length[0] = (length >> 8) & 0xFF; // High byte
    frame->frame_length[1] = length & 0xFF; // Low byte
    frame->frame_type = 0x10; // Transmit Request

    frame->frame_id = frame_id;

    // src: const uint8_t src[8]  (your destAddr)
    // dst: the on-wire field inside your packed struct
    // note: This is the biggest bullshit I have ever seen in my life. Deepending on optimization settings, what the content of the dest_addr array is, and probably other factors,
    // the compiler may optimize the following assignements into a series of half-word or word stores, which due to the unaligned address of the dest_addr field, will sometimes cause a hardfault on the STM32.
    // Unluckely this is the case exactly for the Broadcast address 0x000000000000FFFF which is used to do a general broadcast. I want to kill the person who thought this optimisation was a good idea. Look up UFSR.UNALIGNED in the ARM documentation and content-dependent codegen causing a misaligned half/word/doubleword store
    // I have looked quite a bit, but no solution yet. If it hardfaults, its probably this fault, if it works, its probably luck.
    uint8_t *dst = &frame->dest_addr[0];

    // prevent combining into half/word stores:
    volatile uint8_t *vdst = (volatile uint8_t *)dst;

    vdst[0] = dest_addr[0];
    vdst[1] = dest_addr[1];
    vdst[2] = dest_addr[2];
    vdst[3] = dest_addr[3];
    vdst[4] = dest_addr[4];
    vdst[5] = dest_addr[5];
    vdst[6] = dest_addr[6];
    vdst[7] = dest_addr[7];

    frame->reserved[0] = 0xFF;
    frame->reserved[1] = 0xFE;
    frame->broadcast_radius = 0x00; // Maximum hops
    frame->options = 0x00; // No special options
    
    memcpy(frame->rf_data, frame_payload, payload_length);

    // Calculate and append checksum
    uint64_t checksum = 0;
    // Add all bytes of the packet, except the start delimiter 0x7E and the length (the second and third bytes).
    const uint8_t* data = &frame->frame_type;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
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
void XBee_Transmit(uint8_t* data, uint16_t length, uint8_t* destinationAddress)
{
    constructTransmitFrame(&tx_req_frame, 0x01, destinationAddress, data, length);
    sendAPIFrame((uint8_t*)&tx_req_frame, tx_req_frame.frame_length[0] << 8 | tx_req_frame.frame_length[1]);
}