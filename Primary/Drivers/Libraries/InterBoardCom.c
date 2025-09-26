#include "InterBoardCom.h"
#include "string.h"
#include <stdarg.h>

//The H7 send data to the F4 via SPI1. The data is made up of packets which are sent immediatly after completion of the previous packet.
//The Packets are made up of 1byte of Packet_ID and 32bytes of Data 33 bytes long.
//Data is sent and received from the H7 via DMA, a CS is used to signal the F4 when data is ready to be read.
//The CS is controlled by the H7's SPI peripheral, it is manually pulled low when a transmission starts and high when it ends.

#define SPI1_RX_SIZE (sizeof(InterBoardPacket_t))

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_spi1_rx;

InterBoardCircularBuffer_t txCircBuffer; // outgoing buffer

InterBoardPacket_t receiveBuffer;
uint8_t SPI1_DMA_Rx_Buffer[SPI1_RX_SIZE];

uint8_t SPI1_State = 0; //0: Ready, 1: Busy

void InterBoardCom_Init(void) {
    // Initialize circular buffers
    InterBoardBuffer_Init(&txCircBuffer);
    // Initialize SPI state
    SPI1_State = 0;
}

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
 * @brief Enhanced packet sending with buffering support
 * @param packet Pointer to the packet to send
 * @return 1 if queued successfully, 0 if failed
 */
uint8_t InterBoardCom_QueuePacket(InterBoardPacket_t *packet) {
    // Try to queue the packet
    if (InterBoardBuffer_Push(&txCircBuffer, packet)) {
        // Try to start transmission if SPI is ready
        InterBoardCom_ProcessTxBuffer();
        return 1;
    }
    return 0; // Buffer full
}

/**
 * @brief Processes the transmission buffer
 */
void InterBoardCom_ProcessTxBuffer(void) {
    InterBoardPacket_t packet;
    
    // If SPI is busy or buffer is empty, return
    if (SPI1_State != 0 || InterBoardBuffer_IsEmpty(&txCircBuffer)) {
        return;
    }
    
    // Get next packet from buffer
    if (InterBoardBuffer_Pop(&txCircBuffer, &packet)) {
        // Send the packet
        SPI1_State = 1; // Mark SPI as busy
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Pull CS low
        HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&packet, SPI1_DMA_Rx_Buffer, sizeof(InterBoardPacket_t));
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
    if (SPI1_State != 0) {
        // SPI is busy, cannot send now
        return;
    }
    SPI1_State = 1; // Mark SPI as busy
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Manually pull CS low to start transmission (pulled high in DMA complete callback)
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)packet, SPI1_DMA_Rx_Buffer, sizeof(InterBoardPacket_t));
}

InterBoardPacket_t InterBoardCom_ReceivePacket() {
    // Receive the packet via SPI
    InterBoardPacket_t receivedPacket;
    receivedPacket.InterBoardPacket_ID = SPI1_DMA_Rx_Buffer[0]; // First byte after detection time
    memcpy(receivedPacket.Data, &SPI1_DMA_Rx_Buffer[1], 32);
    return receivedPacket;
}

void InterBoardCom_ProcessReceivedPacket(InterBoardPacket_t *packet) {
    // Process the received packet based on its ID
    switch (packet->InterBoardPacket_ID) {
        case InterBoardPACKET_ID_SELFTEST:
            // Handle self-test packet
            break;
        // Add cases for other packet IDs as needed
        default:
            break;
    }
}

InterBoardPacket_t TestPacket;
void InterBoardCom_SendTestPacket(void) {
    TestPacket = InterBoardCom_CreatePacket(InterBoardPACKET_ID_SELFTEST);
    InterBoardCom_FillRaw(&TestPacket, 32, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
    InterBoardCom_QueuePacket(&TestPacket);
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
void InterBoardCom_SendDataPacket(InterBoardPacketID_t Inter_ID, PacketType_t Packet_ID, DataPacket_t *packet){
    // Create a new InterBoardPacket_t structure
    //Implement later
}

/**
 * @brief Initializes the circular buffer
 * @param cb Pointer to the circular buffer structure
 */
void InterBoardBuffer_Init(InterBoardCircularBuffer_t* cb) {
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
uint8_t InterBoardBuffer_Push(InterBoardCircularBuffer_t* cb, InterBoardPacket_t* packet) {
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
uint8_t InterBoardBuffer_Pop(InterBoardCircularBuffer_t* cb, InterBoardPacket_t* packet) {
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
uint8_t InterBoardBuffer_IsEmpty(InterBoardCircularBuffer_t* cb) {
    return (cb->count == 0);
}

/**
 * @brief Checks if the circular buffer is full
 * @param cb Pointer to the circular buffer structure
 * @return 1 if full, 0 if not full
 */
uint8_t InterBoardBuffer_IsFull(InterBoardCircularBuffer_t* cb) {
    return (cb->count >= INTERBOARD_BUFFER_SIZE);
}

/**
 * @brief Gets the current count of items in the buffer
 * @param cb Pointer to the circular buffer structure
 * @return Number of items in buffer
 */
uint16_t InterBoardBuffer_Count(InterBoardCircularBuffer_t* cb) {
    return cb->count;
}

/**
 * @brief Clears the circular buffer
 * @param cb Pointer to the circular buffer structure
 */
void InterBoardBuffer_Clear(InterBoardCircularBuffer_t* cb) {
    __disable_irq();
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
    __enable_irq();
}

