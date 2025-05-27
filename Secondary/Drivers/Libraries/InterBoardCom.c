#include "InterBoardCom.h"

extern SPI_HandleTypeDef hspi1;

InterBoardPacket_t receiveBuffer;

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

void InterBoardCom_ActivateReceive(void) {
    //Called to activate the SPI DMA receive
    HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)&receiveBuffer, sizeof(InterBoardPacket_t));
}


InterBoardPacket_t InterBoardCom_ReceivePacket(void) {
    //Called when a packet is received inside the SPI_DMA receive complete callback
    HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)&receiveBuffer, sizeof(InterBoardPacket_t)); // Re-activate the SPI DMA receive for the next packet
    return receiveBuffer;
}