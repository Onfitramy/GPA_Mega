#include "InterBoardCom.h"
#include "string.h"
#include <stdarg.h>
#include "Packets.h"
#include "W25Q1.h"

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_spi1_rx;

InterBoardPacket_t receiveBuffer;

void InterBoardCom_ActivateReceive(void) {
    //Called to activate the SPI DMA receive
    HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)&receiveBuffer, sizeof(InterBoardPacket_t));
}


InterBoardPacket_t InterBoardCom_ReceivePacket(void) {
    //Called when a packet is received inside the SPI_DMA receive complete callback
    HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)&receiveBuffer, sizeof(InterBoardPacket_t)); // Re-activate the SPI DMA receive for the next packet
    return receiveBuffer;
}

InterBoardPacket_t InterBoardCom_CreatePacket(InterBoardPacketID_t ID) {
    InterBoardPacket_t packet;
    packet.InterBoardPacket_ID = ID;
    for (int i = 0; i < 32; i++) {
        packet.Data[i] = 0; // Initialize Data array to 0
    }
    return packet;
}

void InterBoardCom_FillRaw(InterBoardPacket_t *packet, int num, ...) {
    va_list args;
    va_start(args, num);
    for (int i = 0; i < num && i < 32; i++) {
        packet->Data[i] = (uint8_t)va_arg(args, int); // 'int' is promoted type for variadic args
    }
    va_end(args);
}

void InterBoardCom_FillData(InterBoardPacket_t *packet, DataPacket_t *data_packet) {
    // Copy the data from the DataPacket_t structure to the InterBoardPacket_t structure
    // Calculate CRC (XOR checksum)
    data_packet->crc = 0;
    for (int i = 0; i < sizeof(data_packet->Data.raw); i++) {
        data_packet->crc ^= data_packet->Data.raw[i];
    }

    memcpy(packet->Data, data_packet, 32);
}

void InterBoardCom_SendPacket(InterBoardPacket_t packet) {
    //First interrupt main to signal incomming packet
    //HAL_SPI_DMAStop(&hspi1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    //HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)&packet, sizeof(InterBoardPacket_t));

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    //InterBoardCom_ReactivateDMAReceive();
    //Called to send a packet over SPI
}

DataPacket_t InterBoardCom_UnpackPacket(InterBoardPacket_t packet) {
    //This function is used to move a InterBoardPacket_t to a DataPacket_t
    DataPacket_t dataPacket;
    memcpy(dataPacket.Header, &packet.Data[0], 2);
    memcpy(&dataPacket.Packet_ID, &packet.Data[2], 1);
    memcpy(&dataPacket.Data, &packet.Data[3], 28);
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