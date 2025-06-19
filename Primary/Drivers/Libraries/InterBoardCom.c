#include "InterBoardCom.h"
#include "string.h"
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
InterBoardPacket_t InterBoardCom_CreatePacket(InterBoardPacketID_t ID) {
    InterBoardPacket_t packet;
    packet.InterBoardPacket_ID = ID;
    for (int i = 0; i < 32; i++) {
        packet.Data[i] = 0; // Initialize Data array to 0
    }
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
void InterBoardCom_FillRaw(InterBoardPacket_t *packet, int num, ...) {
    va_list args;
    va_start(args, num);
    for (int i = 0; i < num && i < 32; i++) {
        packet->Data[i] = (uint8_t)va_arg(args, int); // 'int' is promoted type for variadic args
    }
    va_end(args);
}

/**
 * @brief Fills an InterBoardPacket_t structure with data from a DataPacket_t structure and calculates CRC.
 *
 * This function copies the contents of the Data field from the provided DataPacket_t structure
 * into the Data field of the InterBoardPacket_t structure. It then calculates a simple XOR-based
 * checksum (CRC) over the copied data and stores the result in the crc field of the DataPacket_t structure.
 *
 * @param packet Pointer to the InterBoardPacket_t structure to be filled.
 * @param data Pointer to the DataPacket_t structure containing the source data and where the CRC will be stored.
 */
void InterBoardCom_FillData(InterBoardPacket_t *packet, DataPacket_t *data_packet) {
    // Copy the data from the DataPacket_t structure to the InterBoardPacket_t structure
    // Calculate CRC (XOR checksum)
    data_packet->crc = 0;
    for (int i = 0; i < sizeof(data_packet->Data.raw); i++) {
        data_packet->crc ^= data_packet->Data.raw[i];
    }

    memcpy(packet->Data, data_packet, 32);
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
    InterBoardPacket_t packet = InterBoardCom_CreatePacket(InterBoardPACKET_ID_SELFTEST);
    InterBoardCom_FillRaw(&packet, 32, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
    InterBoardCom_SendPacket(&packet);
}

/**
 * @brief Sends a data packet to another board via SPI communication.
 *
 * This function creates an inter-board packet, fills it with the provided data,
 * and sends it using the SPI interface. It abstracts the process of packet creation,
 * data population, and transmission.
 *
 * @param Inter_ID   The identifier for the target inter-board communication channel.
 * @param Packet_ID  The type identifier for the packet being sent.
 * @param packet     Pointer to the data packet to be sent.
 */
void InterBoardCom_SendDataPacket(InterBoardPacketID_t Inter_ID, PacketType_t Packet_ID, PacketData_u *packet){
    // Create a new InterBoardPacket_t structure
    InterBoardPacket_t interPacket = InterBoardCom_CreatePacket(Inter_ID);
    DataPacket_t dataPacket = {
        .Header = {0x41, 0x45}, // AE header
        .Packet_ID = Packet_ID,
    };
    memcpy(&dataPacket.Data, packet, 28);

    // Fill the Data field of the InterBoardPacket_t with the provided packet data
    InterBoardCom_FillData(&interPacket, &dataPacket);
    
    // Send the filled packet via SPI
    InterBoardCom_SendPacket(&interPacket);
}