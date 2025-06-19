#include "InterBoardCom.h"
#include "string.h"
#include "Packets.h"
#include "W25Q1.h"

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_spi1_rx;

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
InterBoardPacket_t InterBoardCom_CreatePacket(InterBoardPacketID_t ID) {
    InterBoardPacket_t packet;
    packet.InterBoardPacket_ID = ID;
    for (int i = 0; i < 16; i++) {
        packet.Data[i] = 0; // Initialize Data array to 0
    }
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

DataPacket_t InterBoardCom_UnpackPacket(InterBoardPacket_t packet) {
    //This function is used to move a InterBoardPacket_t to a DataPacket_t
    DataPacket_t dataPacket;
    memcpy(dataPacket.Header, &packet.Data[0], 2);
    memcpy(&dataPacket.Packet_ID, &packet.Data[2], 1);
    memcpy(dataPacket.Data, &packet.Data[3], 28);
    memcpy(&dataPacket.crc, &packet.Data[31], 1);
    
    return dataPacket;
}

uint8_t selftestPacketsReceived = 0;
void InterBoardCom_ParsePacket(InterBoardPacket_t packet) {
    //This function is used to parse the received packet
    switch (packet.InterBoardPacket_ID) {
        case InterBoardPACKET_ID_SELFTEST:
            //Handle self-test packet
            selftestPacketsReceived += 1;
            break;
        
        case InterBoardPACKET_ID_DataSaveFLASH:
        {
            //Handle data save to flash packet
            DataPacket_t dataPacket = InterBoardCom_UnpackPacket(packet);
            W25Q_SaveToLog((uint8_t *)&dataPacket, sizeof(dataPacket)); // Save the data to flash memory
            break;
        }

        case InterBoardPACKET_ID_ResetFLASH:
            // This is a command packet, so we might want to send an acknowledgment back
            W25Q1_Reset(); // Reset the flash memory
            break;

        default:
            //Handle unknown packet
            break;
    }
}

void InterBoardCom_ReactivateDMAReceive(void) {
    //This function is used to reactivate the SPI DMA receive when it is broken due to an error, most comonly due to a timeout or halting the program
    HAL_SPI_DMAStop(&hspi1);
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_FLAG_TEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4);
    // Clear possible SPI overrun (OVR) flag by reading SR and DR
    volatile uint32_t tmp;
    tmp = hspi1.Instance->SR;
    tmp = hspi1.Instance->DR;
    (void)tmp;
    HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)&receiveBuffer, sizeof(InterBoardPacket_t));
}