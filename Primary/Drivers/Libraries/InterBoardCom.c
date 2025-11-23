#include "InterBoardCom.h"
#include "string.h"
#include <stdarg.h>
#include <stdio.h>
#include "main.h"
#include "spark.h"
#include "usbd_cdc_if.h"
#include "statemachine.h"

//The H7 send data to the F4 via SPI1. The data is made up of packets which are sent immediatly after completion of the previous packet.
//The Packets are made up of 1byte of Packet_ID and 32bytes of Data 33 bytes long.
//Data is sent and received from the H7 via DMA, a CS is used to signal the F4 when data is ready to be read.
//The CS is controlled by the H7's SPI peripheral, it is manually pulled low when a transmission starts and high when it ends.

#define SPI1_RX_SIZE (sizeof(InterBoardPacket_t))

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_spi1_rx;

extern DataPacket_t powerData; //For receiving power data from secondary

InterBoardCircularBuffer_t txCircBuffer; // outgoing buffer

InterBoardPacket_t receiveBuffer;
InterBoardPacket_t transmitBuffer;
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
    calcCRC(data_packet);

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
        //Transmission is requested in the main loop
        return 1;
    }
    return 0; // Buffer full
}

/**
 * @brief Processes the transmission buffer, called once from 100hz loop, then from SPI DMA complete callback till empty
 */
void InterBoardCom_ProcessTxBuffer(void) {
    
    // If SPI is busy or buffer is empty, return
    if (InterBoardBuffer_IsEmpty(&txCircBuffer)) {
        return;
    }
    
    if (SPI1_State != 0) {
        return; // SPI is busy
    }
    // Get next packet from buffer
    if (InterBoardBuffer_Pop(&txCircBuffer, &transmitBuffer)) {
        // Send the packet
        SPI1_State = 1; // Mark SPI as busy
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Aktivate Interrupt
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Deactivate Interrupt
        // Add small delay (10-100 microseconds) to let slave prepare
        delay_us(3); //10: 0.3% dropped 50: 0.3% dropped
        HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&transmitBuffer, SPI1_DMA_Rx_Buffer, sizeof(InterBoardPacket_t));
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
    memset(SPI1_DMA_Rx_Buffer, 0, SPI1_RX_SIZE); // Clear buffer after reading
    return receivedPacket;
}

void InterBoardCom_ProcessReceivedPacket(InterBoardPacket_t *packet) {
    // Process the received packet based on its ID
    switch (packet->InterBoardPacket_ID) {
        case INTERBOARD_OP_ECHO:
            // Handle echo packet
            break;
        // Add cases for other packet IDs as needed
        default:
            break;
    }
}
uint8_t return_string[80];
void InterBoardCom_EvaluateCommand(DataPacket_t *dataPacket){
    switch (dataPacket->Data.command.command_target) {
        case COMMAND_TARGET_NONE:
            // No target specified, possibly log or ignore
            break;
        case COMMAND_TARGET_SPECIAL:
            if (dataPacket->Data.command.command_id == COMMAND_ID_PRIMARY_RESET) {
                // Special command 0x00: Reset the primary board
                HAL_NVIC_SystemReset();
                // No Acknowledgment possible, as we reset immediately
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_SECONDARY_RESET) {
                // Special command 0x01: Reset the secondary board
                // We never get here
            }
            break;
        case COMMAND_TARGET_STATE:
            // Handle state-related commands
            if (dataPacket->Data.command.command_id == COMMAND_ID_STATE_FORCE) {
                // State command 0x04: Force State
                StateMachine_ForceState(&flight_sm, (flight_state_t)dataPacket->Data.command.params[0]);
                InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_STATE_SIMULATE_EVENT) {
                // State command 0x05: Simulate Event
                StateMachine_Dispatch(&flight_sm, (flight_event_t)dataPacket->Data.command.params[0]);
                InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
            }
            break;
        case COMMAND_TARGET_SPARK:
            if (dataPacket->Data.command.command_id == COMMAND_ID_SPARK_SET_ACS_ANGLE) {
                uint8_t *angle_address = dataPacket->Data.command.params;
                float angle_target;
                memcpy(&angle_target, angle_address, sizeof(float));
                ACS_SetAngle(angle_target);
            } else {
                SPARK_sendCommand(dataPacket);
            }
            break;
        case COMMAND_TARGET_TESTING:
            // Handle testing commands
            break;
        case COMMAND_TARGET_STORAGE:
            // Should never receive this command from secondary board as it should already have handled it
            break;
        case COMMAND_TARGET_CAMERA:
            // Should never receive this command from secondary board as it should already have handled it
            break;
        case COMMAND_TARGET_GROUNDSTATION:
            // Groundstation should never receive commands from other boards, only via USB from PC
            break;
        case COMMAND_TARGET_LOGGING:
            if (dataPacket->Data.command.command_id == 0x00) {
                // Logging command 0x00: FlightDataOut
                // StartLogging();
            }
            break;
        case COMMAND_TARGET_ACK:
            // Acknowledgment packet received, possibly log or process
            snprintf((char *)return_string, sizeof(return_string), "Received ACK for target %d, command %d with status %d\r\n",
                     dataPacket->Data.command.params[0], dataPacket->Data.command.params[1], dataPacket->Data.command.command_id);
            CDC_Transmit_HS(return_string, strlen((char *)return_string));
            break;
        default:
            // Unknown command target
            InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 2); // Invalid command
            break;
    }
}

// Acknowledge a command with status of its execution as its ID (0=Success, 1=Failed, 2=Invalid)
void InterBoardCom_command_acknowledge(uint8_t command_target, uint8_t command_id, uint8_t status) {
    uint8_t params[2];
    params[0] = command_target;
    params[1] = command_id;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_ACK, status, params, sizeof(params));
    InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_RADIO, &packet);
}

extern uint8_t is_groundstation;
void InterBoardCom_ParsePacket(InterBoardPacket_t *packet) {
    // Process the received packet based on its ID
    switch (packet->InterBoardPacket_ID) {
        case INTERBOARD_OP_DEBUG_VIEW: //Send for debugging
            if (is_groundstation) {
                //PlotDataPacket((DataPacket_t *)packet->Data);
                USB_QueueDataPacket((DataPacket_t *)packet->Data);
            }
            break;
        // Add cases for other packet IDs as needed

        case (INTERBOARD_OP_SAVE_SEND | INTERBOARD_TARGET_MCU): { //Receive Data Packet for use on MCU
            if (((DataPacket_t *)packet->Data)->Packet_ID == PACKET_ID_POWER) {
                powerData = *((DataPacket_t *)packet->Data);
            }
            break;
        }

        case (INTERBOARD_OP_CMD | INTERBOARD_TARGET_MCU): {
            DataPacket_t *cmd = (DataPacket_t *)packet->Data;
            if (cmd->Packet_ID != PACKET_ID_COMMAND) {
                // Invalid command ID
                return;
            } else {
                InterBoardCom_EvaluateCommand(cmd);
            }
        }
        default:
            break;
    }
}

uint8_t usb_packet_buffer[128];
extern QueueHandle_t USB_Tx_Queue;
uint8_t USB_QueueDataPacket(DataPacket_t *packet) {
    if (packet == NULL) {
        return 0;
    }
    if (xQueueSend(USB_Tx_Queue, packet, 0) == pdTRUE) {
        return 1; // Successfully queued
    }
    return 0; // Queue full or error
}

uint8_t USB_OutputDataPacket(DataPacket_t *packet) {
    if (packet == NULL) {
        return 0;
    }

    switch(packet->Packet_ID) {
        case PACKET_ID_IMU:
            sprintf((char *)usb_packet_buffer, "ID%d,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d\n",
                    packet->Packet_ID , packet->timestamp, packet->Data.imu.accelX, packet->Data.imu.accelY, packet->Data.imu.accelZ,
                    packet->Data.imu.gyroX, packet->Data.imu.gyroY, packet->Data.imu.gyroZ, packet->Data.imu.magX, packet->Data.imu.magY, packet->Data.imu.magZ);
            break;
        case PACKET_ID_ATTITUDE:
            sprintf((char *)usb_packet_buffer, "ID%d,%ld,%f,%f,%f\n",
                    packet->Packet_ID , packet->timestamp, packet->Data.attitude.phi, packet->Data.attitude.theta, packet->Data.attitude.psi);
            break;
        case PACKET_ID_GPS:
            sprintf((char *)usb_packet_buffer, "ID%d,%ld,%ld,%ld,%ld,%d,%d\n",
                    packet->Packet_ID , packet->timestamp, packet->Data.gps.latitude, packet->Data.gps.longitude, packet->Data.gps.altitude, packet->Data.gps.course, packet->Data.gps.speed);
            break;
        case PACKET_ID_POSITION:
            sprintf((char *)usb_packet_buffer, "ID%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
                    packet->Packet_ID , packet->timestamp, packet->Data.position.posX, packet->Data.position.posY, packet->Data.position.posZ,
                    packet->Data.position.velX, packet->Data.position.velY, packet->Data.position.velZ);
            break;
        case PACKET_ID_POWER:
            sprintf((char *)usb_packet_buffer, "ID%d,%ld,%ld,%ld,%d,%d,%d,%d,%d\n",
                    packet->Packet_ID , packet->timestamp, packet->Data.power.M1_5V_bus, packet->Data.power.M1_BAT_bus_volt,
                    packet->Data.power.M2_bus_5V, packet->Data.power.M2_bus_GPA_bat_volt, packet->Data.power.PU_bat_bus_volt,
                    packet->Data.power.PU_curr, packet->Data.power.PU_pow);
            break;
        case PACKET_ID_TEMPERATURE:
            sprintf((char *)usb_packet_buffer, "ID%d,%ld,%d,%d,%d,%d,%d,%d,%d\n",
                    packet->Packet_ID , packet->timestamp, packet->Data.temperature.M1_ADC, packet->Data.temperature.M1_BMP,
                    packet->Data.temperature.M1_DTS, packet->Data.temperature.M1_IMU1, packet->Data.temperature.M1_MAG,
                    packet->Data.temperature.M2_3V3, packet->Data.temperature.M2_XBee);
            break;
        case PACKET_ID_STATUS:
            sprintf((char *)usb_packet_buffer, "ID%d,%ld,%ld,%ld,%ld,%d\n",
                    packet->Packet_ID , packet->timestamp, packet->Data.status.status_flags, packet->Data.status.sensor_status_flags, packet->Data.status.error_flags, packet->Data.status.State);
            break;
        default:
            break;
            // return; // Unsupported packet type
    }
    return CDC_Transmit_HS(usb_packet_buffer, strlen((char *)usb_packet_buffer));
}

InterBoardPacket_t TestPacket;
void InterBoardCom_SendTestPacket(void) {
    TestPacket = InterBoardCom_CreatePacket(INTERBOARD_OP_ECHO);
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
void InterBoardCom_SendDataPacket(InterBoardPacketID_t Inter_ID, DataPacket_t *packet) {
    // Create a new InterBoardPacket_t structure
    InterBoardPacket_t newPacket = InterBoardCom_CreatePacket(Inter_ID);
    InterBoardCom_FillData(&newPacket, packet);

    // Send the packet
    int result = InterBoardCom_QueuePacket(&newPacket);
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

