#include "InterBoardCom.h"
#include <stdarg.h>

//The H7 send data to the F4 via SPI1. The data is made up of packets which are sent immediatly after completion of the previous packet.
//The Packets are made up of 1byte of Packet_ID, 16bytes of Data and a crc. In total the packet is 18 bytes long.

extern SPI_HandleTypeDef hspi1;

/**
 * @brief Creates and initializes an InterBoardPacket_t structure.
 *
 * This function initializes a new InterBoardPacket_t packet with the specified PacketID_t ID.
 * The Data array is set to zero, and the CRC field is initialized to zero.
 *
 * @param ID The PacketID_t identifier to assign to the packet.
 * @return InterBoardPacket_t The initialized packet structure.
 */
InterBoardPacket_t InterBoardCom_CreatePacket(PacketID_t ID) {
    InterBoardPacket_t packet;
    packet.Packet_ID = ID;
    for (int i = 0; i < 16; i++) {
        packet.Data[i] = 0; // Initialize Data array to 0
    }
    packet.crc = 0; // Initialize CRC to 0
    return packet;
}

/**
 * @brief Fills the Data array of an InterBoardPacket_t structure with variable arguments.
 *
 * This function takes a pointer to an InterBoardPacket_t packet and fills its Data array
 * with up to 16 values provided as variadic arguments. Each value is cast to uint8_t before
 * being stored in the Data array.
 *
 * @param packet Pointer to the InterBoardPacket_t structure to fill.
 * @param num Number of data values to fill (maximum 16).
 * @param ... Variable arguments representing the data values to be stored in the packet.
 *
 * @note Only the first 16 values will be stored if num exceeds 16.
 */
void InterBoardCom_FillData(InterBoardPacket_t *packet, int num, ...) {
    va_list args;
    va_start(args, num);
    for (int i = 0; i < num && i < 16; i++) {
        packet->Data[i] = (uint8_t)va_arg(args, int); // 'int' is promoted type for variadic args
    }
    va_end(args);

    // Calculate CRC for the packet
    packet->crc = 0; // Reset CRC
    for (size_t i = 0; i < sizeof(packet->Data); i++) { //XOR checksum
        packet->crc ^= packet->Data[i];
    }
}

/**
 * @brief Sends an inter-board communication packet via SPI using DMA.
 *
 * This function calculates a simple XOR checksum (CRC) over the packet's data,
 * stores the result in the packet's crc field, and then transmits the entire
 * packet using the SPI peripheral in DMA mode.
 *
 * @param packet Pointer to the InterBoardPacket_t structure to be sent.
 */
void InterBoardCom_SendPacket(InterBoardPacket_t *packet) {
    
    // Send the packet via SPI
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)packet, sizeof(InterBoardPacket_t));
}

void InterBoardCom_SendTestPacket(void) {
    InterBoardPacket_t packet = InterBoardCom_CreatePacket(PACKET_ID_SELFTEST);
    InterBoardCom_FillData(&packet, 16, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
    InterBoardCom_SendPacket(&packet);
}