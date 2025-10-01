#include "InterBoardCom.h"
#include "stdio.h"
#include "string.h"
#include <stdarg.h>
#include "Packets.h"
#include "W25Q1.h"
#include "fatfs.h"
#include "SD.h"

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

volatile InterBoardPacket_t receiveBuffer;
volatile InterBoardPacket_t transmitBuffer;

CircularBuffer_t txCircBuffer; // outgoing buffer

volatile uint8_t SPI1_STATUS = 0; // 0= Idle, 1 = Busy

void InterBoardCom_ClearSPIErrors(void);

void InterBoardCom_Init(void) {
    // Initialize circular buffers
    CircBuffer_Init(&txCircBuffer);
    // Initialize SPI state
}


/* The SPI Slave (F4) is triggered by the main (H7) and automaticaly sends out its data packets while receiving from the master */

void InterBoardCom_ActivateReceive(void) {
    //Called to activate the SPI DMA receive

    if (SPI1_STATUS != 0 || hspi1.State != HAL_SPI_STATE_READY || hdma_spi1_rx.State != HAL_DMA_STATE_READY || hspi1.hdmatx->State != HAL_DMA_STATE_READY) {
        return; // Busy
    }

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
        // Master is currently transmitting - don't start DMA
        return;
    }

    if (hspi1.Instance->SR & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR)) {
        // SPI has errors - clear them before reactivating
        // They are caused by transmitions from the master when not listening
        InterBoardCom_ClearSPIErrors();
    }

    if(CircBuffer_Pop(&txCircBuffer, &transmitBuffer) == 0) { // Get next packet to transmit, if none available a packet with ID 0 will be sent
        transmitBuffer.InterBoardPacket_ID = 0; //The data will be ignored by the master
    }

    SPI1_STATUS = 1; // Busy
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&transmitBuffer, (uint8_t *)&receiveBuffer, sizeof(InterBoardPacket_t));
}

// Add this new function
void InterBoardCom_ClearSPIErrors(void) {
    // Stop any ongoing DMA
    HAL_SPI_DMAStop(&hspi1);
    
    // Clear DMA flags
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_FLAG_TEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4);
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_FLAG_TEIF3_7 | DMA_FLAG_FEIF3_7 | DMA_FLAG_DMEIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_TCIF3_7);

    // Drain any pending RX data
    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)) {
        (void)hspi1.Instance->DR;
    }
    
    // Clear SPI error flags by reading SR and DR
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_MODF)) {
    __HAL_SPI_CLEAR_MODFFLAG(&hspi1); // macro handles SR read + CR1 write
    }
    
    // Reset SPI state
    hspi1.State = HAL_SPI_STATE_READY;
    hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
    
    SPI1_STATUS = 0; // Idle
}


InterBoardPacket_t InterBoardCom_ReceivePacket(void) {
    //Called when a packet is received inside the SPI_DMA receive complete callback
    // Create a local copy to avoid race condition
    __DSB(); // Ensure memory operations complete
    InterBoardPacket_t packet;
    memcpy(&packet, (void*)&receiveBuffer, sizeof(InterBoardPacket_t));
    memset((void*)&receiveBuffer, 0, sizeof(InterBoardPacket_t)); // Clear the receive buffer for next reception
    return packet;
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

uint8_t InterBoard_CheckCRC(DataPacket_t *packet) {
    // Calculate CRC (XOR checksum)
    uint8_t crc = 0;
    for (int i = 0; i < sizeof(packet->Data.raw); i++) {
        crc ^= packet->Data.raw[i];
    }
    return (crc == packet->crc);
}

static uint8_t rx_dummy[sizeof(InterBoardPacket_t)];
void InterBoardCom_SendPacket(InterBoardPacket_t *packet) {
    //Push the packet to the transmit buffer
    CircBuffer_Push(&txCircBuffer, packet);
}

InterBoardPacket_t TestPacket;
void InterBoardCom_SendTestPacket(void) {
    TestPacket = InterBoardCom_CreatePacket(InterBoardPACKET_ID_SELFTEST);
    InterBoardCom_FillRaw(&TestPacket, 32, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
    InterBoardCom_SendPacket(&TestPacket);
}

DataPacket_t InterBoardCom_UnpackPacket(InterBoardPacket_t packet) {
    //This function is used to move a InterBoardPacket_t to a DataPacket_t
    DataPacket_t dataPacket;
    memcpy(&dataPacket.Packet_ID, &packet.Data[0], 1);
    memcpy(&dataPacket.timestamp, &packet.Data[1], 4);
    memcpy(&dataPacket.Data, &packet.Data[5], 26);
    memcpy(&dataPacket.crc, &packet.Data[31], 1);

    return dataPacket;
}

uint32_t valid_packets;
uint32_t invalid_packets;

void InterBoardCom_ParsePacket(InterBoardPacket_t packet) {
    //This function is used to parse the received packet

    //Remove the top bit on the ID to ignore the more data incoming bit(already handled by the DMA)
    packet.InterBoardPacket_ID &= 0x7F;

    DataPacket_t dataPacket = InterBoardCom_UnpackPacket(packet);

    uint8_t valid_crc = InterBoard_CheckCRC(&dataPacket); //Check CRC

    if (valid_crc || packet.InterBoardPacket_ID == InterBoardPACKET_ID_Echo || packet.InterBoardPacket_ID == InterBoardPACKET_ID_SELFTEST) {
        valid_packets++;
    } else {
        invalid_packets++;
        return; // Invalid packet, ignore
    }

    switch (packet.InterBoardPacket_ID) {
        case InterBoardPACKET_ID_SELFTEST:
            //Handle self-test packet
            break;
        
        case InterBoardPACKET_ID_Echo:
            //Handle echo packet
            packet.InterBoardPacket_ID = InterBoardPACKET_ID_DataAck; // Change ID to acknowledge
            InterBoardCom_SendPacket(&packet); // Echo back the received packet
            break;

        case InterBoardPACKET_ID_DataSaveFLASH:
        {
            //Handle data save to flash packet
            char LogMessage[100];
            DataPacket_t dataPacket = InterBoardCom_UnpackPacket(packet);
            SD_AppendDataPacketToBuffer(&dataPacket); // Save the data to SD card buffer
            //W25Q_SaveToLog((uint8_t *)&dataPacket, sizeof(dataPacket)); // Save the data to flash memory
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
    SPI1_STATUS = 1;
    HAL_SPI_TransmitReceive_DMA(&hspi1,  (uint8_t *)&transmitBuffer, (uint8_t *)&receiveBuffer, sizeof(InterBoardPacket_t));
}

/**
 * @brief Initializes the circular buffer
 * @param cb Pointer to the circular buffer structure
 */
void CircBuffer_Init(CircularBuffer_t* cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

/**
 * @brief Pushes a packet into the circular buffer
 * @param cb Pointer to the circular buffer structure
 * @param packet Pointer to the packet to be added
 * @return 1 if successful, 0 if buffer is full
 */
uint8_t CircBuffer_Push(CircularBuffer_t* cb, InterBoardPacket_t* packet) {
    if (cb->count >= INTERBOARD_BUFFER_SIZE) {
        return 0; // Buffer is full
    }
    
    // Disable interrupts to ensure atomic operation
    __disable_irq();
    
    // Copy packet to buffer
    memcpy(&cb->buffer[cb->head], packet, sizeof(InterBoardPacket_t));
    
    // Update head pointer
    cb->head = (cb->head + 1) % INTERBOARD_BUFFER_SIZE;
    cb->count++;
    
    __enable_irq();
    
    return 1; // Success
}

/**
 * @brief Pops a packet from the circular buffer
 * @param cb Pointer to the circular buffer structure
 * @param packet Pointer to store the popped packet
 * @return 1 if successful, 0 if buffer is empty
 */
uint8_t CircBuffer_Pop(CircularBuffer_t* cb, InterBoardPacket_t* packet) {
    if (cb->count == 0) {
        return 0; // Buffer is empty
    }
    
    // Disable interrupts to ensure atomic operation
    __disable_irq();
    
    // Copy packet from buffer
    memcpy(packet, &cb->buffer[cb->tail], sizeof(InterBoardPacket_t));
    
    // Update tail pointer
    cb->tail = (cb->tail + 1) % INTERBOARD_BUFFER_SIZE;
    cb->count--;
    
    __enable_irq();
    
    return 1; // Success
}

/**
 * @brief Checks if the circular buffer is empty
 * @param cb Pointer to the circular buffer structure
 * @return 1 if empty, 0 if not empty
 */
uint8_t CircBuffer_IsEmpty(CircularBuffer_t* cb) {
    return (cb->count == 0);
}

/**
 * @brief Checks if the circular buffer is full
 * @param cb Pointer to the circular buffer structure
 * @return 1 if full, 0 if not full
 */
uint8_t CircBuffer_IsFull(CircularBuffer_t* cb) {
    return (cb->count >= INTERBOARD_BUFFER_SIZE);
}

/**
 * @brief Gets the current count of items in the buffer
 * @param cb Pointer to the circular buffer structure
 * @return Number of items in buffer
 */
uint16_t CircBuffer_Count(CircularBuffer_t* cb) {
    return cb->count;
}

/**
 * @brief Clears the circular buffer
 * @param cb Pointer to the circular buffer structure
 */
void CircBuffer_Clear(CircularBuffer_t* cb) {
    __disable_irq();
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
    __enable_irq();
}