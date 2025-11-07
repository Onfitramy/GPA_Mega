#include "InterBoardCom.h"
#include "stdio.h"
#include "string.h"
#include <stdarg.h>
#include <stdbool.h>

#include "packets.h"
#include "W25Q1.h"
#include "fatfs.h"
#include "SD.h"
//#include "xBee.h"
#include "radio.h"
#include "PowerUnit.h"

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

volatile InterBoardPacket_t receiveBuffer;
volatile InterBoardPacket_t transmitBuffer;

CircularBuffer_t txCircBuffer; // outgoing buffer

volatile uint8_t SPI1_STATUS = 0; // 0= Idle, 1 = Busy

uint32_t packets_sent_to_self = 0;
uint32_t valid_packets;
uint32_t invalid_packets;
uint32_t packets_dropped;
float packets_dropped_rate; //% of packets dropped

extern volatile bool saving_to_SD;
extern W25QPage0_config_t W25Q_FLASH_CONFIG;

void InterBoardCom_ClearSPIErrors(void);

void InterBoardCom_Init(void) {
    // Initialize circular buffers
    CircBuffer_Init(&txCircBuffer);

    HAL_NVIC_EnableIRQ(EXTI4_IRQn); // Enable EXTI4 interrupt for NSS pin (PA4)
    // Initialize SPI state
}


/* The SPI Slave (F4) is triggered by the main (H7) and automaticaly sends out its data packets while receiving from the master */
//Right now a dropped packet rate of 3 in 1000 is observed
void InterBoardCom_ActivateReceive(void) {
    //Called to activate the SPI DMA receive
    packets_sent_to_self++;
    packets_dropped = packets_sent_to_self - valid_packets - invalid_packets;

    if (SPI1_STATUS != 0 || hspi1.State != HAL_SPI_STATE_READY || hdma_spi1_rx.State != HAL_DMA_STATE_READY || hspi1.hdmatx->State != HAL_DMA_STATE_READY) {
        //return; // Busy
        //HAL_SPI_DMAStop(&hspi1); // Stop any ongoing DMA
        //SPI1_STATUS = 0; // Idle
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
    TestPacket = InterBoardCom_CreatePacket(INTERBOARD_OP_NONE);
    InterBoardCom_FillRaw(&TestPacket, 32, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
    InterBoardCom_SendPacket(&TestPacket);
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
    InterBoardCom_SendPacket(&newPacket);
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

DataPacket_t attitudeTransmitPacket;
void InterBoardCom_ParsePacket(InterBoardPacket_t packet) {
    //This function is used to parse the received packet
    packet.InterBoardPacket_ID &= 0x7F;

    DataPacket_t dataPacket = InterBoardCom_UnpackPacket(packet);

    bool valid_crc = InterBoard_CheckCRC(&dataPacket); //Check CRC

    packets_dropped_rate = (1 - ((float)(valid_packets + invalid_packets) / (float)packets_sent_to_self)) * 100.0f;

    if (valid_crc) {
        valid_packets++;
    } else {
        invalid_packets++;
        return; // Invalid packet, ignore
    }

    uint8_t Interboard_OP = packet.InterBoardPacket_ID & 0x07; //Extract operation type
    uint8_t Interboard_Target = packet.InterBoardPacket_ID & 0xF8; //Extract Target type
    switch (Interboard_OP) {
        case INTERBOARD_OP_SAVE_SEND:
        {
            //Handle data save to flash packet
            char LogMessage[100];
            DataPacket_t dataPacket = InterBoardCom_UnpackPacket(packet);

            if ((Interboard_Target & INTERBOARD_TARGET_SD) == INTERBOARD_TARGET_SD) {
                SD_AppendDataPacketToBuffer(&dataPacket); // Save the data to SD card buffer
            }

            if ((Interboard_Target & INTERBOARD_TARGET_FLASH) == INTERBOARD_TARGET_FLASH) {
                W25Q_AddFlashBufferPacket(&dataPacket);
            }

            if ((Interboard_Target & INTERBOARD_TARGET_RADIO) == INTERBOARD_TARGET_RADIO) {
                radioSend(&dataPacket);
            }
            break;
        }

        //Target MCU mean act on the commands or send them to main if needed
        case (INTERBOARD_OP_CMD): {
            //Target Radio means send the command via radio to the flight computer
            if(Interboard_Target == INTERBOARD_TARGET_RADIO) {
                radioSend(&dataPacket);
                break;
            } else if (Interboard_Target == INTERBOARD_TARGET_MCU) {
                if (dataPacket.Packet_ID != PACKET_ID_COMMAND) {
                    // Invalid command ID
                    return;
                } else {
                    InterBoardCom_EvaluateCommand(&dataPacket);
                }

            }
            break;
        }

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

void InterBoardCom_EvaluateCommand(DataPacket_t *dataPacket){
    switch (dataPacket->Data.command.command_target) {
        case COMMAND_TARGET_NONE:
            // No target specified, possibly log or ignore
            break;
        case COMMAND_TARGET_SPECIAL:
            if (dataPacket->Data.command.command_id == COMMAND_ID_PRIMARY_RESET) {
                // Special command 0x00: Reset the primary board
                InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_MCU, dataPacket); // Forward command to main board
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_SECONDARY_RESET) {
                // Special command 0x01: Reset the secondary board
                NVIC_SystemReset();
                // No ACK possible as the board resets immediately
            }
            break;
        case COMMAND_TARGET_STATE:
            // State machine is on the main board, forward command
            InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_MCU, dataPacket);
            break;
        case COMMAND_TARGET_SPARK:
            // Forward power unit commands to the main board
            InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_MCU, dataPacket);
            break;
        case COMMAND_TARGET_TESTING:
            // Testing commands are on the main board, forward command
            InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_MCU, dataPacket);
            break;
        case COMMAND_TARGET_STORAGE:
            // Handle storage commands
            if (dataPacket->Data.command.command_id == 0x00) {
                // Storage command 0x00: FlashToSD
                W25Q_FLASH_CONFIG.write_logs = false;
                saving_to_SD = true; // Trigger saving flash to SD in main loop
            } else if (dataPacket->Data.command.command_id == 0x01) {
                // Storage command 0x01: FlashReset
                W25Q_FLASH_CONFIG.write_logs = false;
                W25Q_Chip_Erase();
                InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
            } else if (dataPacket->Data.command.command_id == 0x02) {
                // Storage command 0x02: FLASH saving enable/disable
                if (dataPacket->Data.command.params[0] == 0x01) {
                    W25Q_FLASH_CONFIG.write_logs = true;
                } else if (dataPacket->Data.command.params[0] == 0x00) {
                    W25Q_FLASH_CONFIG.write_logs = false;
                }
                InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
            }
            break;
        case COMMAND_TARGET_CAMERA:
            if (dataPacket->Data.command.command_id == COMMAND_ID_CAMERA_POWER) {
                // Camera command 0x00: Power On/Off
                if (dataPacket->Data.command.params[0] == 0x01) {
                    Camera_SwitchOn(); // Power On
                    InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
                } else if (dataPacket->Data.command.params[0] == 0x00) {
                    Camera_SwitchOff(); // Power Off
                    InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
                }
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_CAMERA_RECORD) {
                // Camera command 0x01: Start/Stop recording
                if (dataPacket->Data.command.params[0] == 0x01) {
                    Camera_StartRecording();
                    InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
                } else if (dataPacket->Data.command.params[0] == 0x00) {
                    Camera_StopRecording();
                    InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
                }
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_CAMERA_SKIPDATE) {
                // Camera command 0x02: Skip Date
                Camera_SkipDate();
                InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_CAMERA_WIFI) {
                // Camera command 0x03: Wifi On/Off
                if (dataPacket->Data.command.params[0] == 0x01) {
                    Camera_WifiOn(); // Wifi On
                    InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
                } else if (dataPacket->Data.command.params[0] == 0x00) {
                    Camera_WifiOff(); // Wifi Off
                    InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
                }
            }
            break;
        case COMMAND_TARGET_RADIO:
            if (dataPacket->Data.command.command_id == COMMAND_ID_RADIO_SWITCH) {
                // Radio command 0x01: Switch radio to xbee(1) or nrf(2)
                if(dataPacket->Data.command.params[0] == 0x01) {
                    radioSet(XBEE_ACTIVE);
                    InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
                } else if (dataPacket->Data.command.params[0] == 0x02) {
                    radioSet(NRF_24_ACTIVE);
                    InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
                }
                InterBoardCom_command_acknowledge(dataPacket->Data.command.command_target, dataPacket->Data.command.command_id, 0);
            }
            break;
        case COMMAND_TARGET_GROUNDSTATION:
            // Groundstation should never receive commands from other boards, only via USB from PC
            break;
        case COMMAND_TARGET_LOGGING:
            // Logging commands are on the main board, forward command
            InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_MCU, dataPacket);
            break;
        case COMMAND_TARGET_SECONDARY:
            if (dataPacket->Data.command.command_id == COMMAND_ID_PU_POWER_CAM) {
                HAL_GPIO_WritePin(CAMS_GPIO_Port, CAMS_Pin, dataPacket->Data.command.params[0]);
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_PU_POWER_RECOVERY) {
                HAL_GPIO_WritePin(Recovery_GPIO_Port, Recovery_Pin, dataPacket->Data.command.params[0]);
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_PU_POWER_ACS) {
                HAL_GPIO_WritePin(ACS_GPIO_Port, ACS_Pin, dataPacket->Data.command.params[0]);
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_BUZZER_PLAYNOTE) {
                uint16_t playtime_ms = 0;
                playtime_ms = dataPacket->Data.command.params[0] << 8;
                playtime_ms |= dataPacket->Data.command.params[1];
                uint8_t length = dataPacket->Data.command.params[2];
                char note[length+1];
                for (uint8_t i = 0; i < length; i++) {
                    note[i] = dataPacket->Data.command.params[3+i];
                }
                note[length] = '\0';
                buzzerPlayNote(note, playtime_ms);
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_BUZZER_PLAYSONG) {
                buzzerPlayPattern(dataPacket->Data.command.params[0]);
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_BUZZER_PLAYSONGREPEAT) {
                uint16_t repeattime_ms = 0;
                repeattime_ms = dataPacket->Data.command.params[0] << 8;
                repeattime_ms |= dataPacket->Data.command.params[1];
                uint8_t song_pattern = dataPacket->Data.command.params[2];
                buzzerPlayPattern(song_pattern);
                buzzerEnablePeriodicMode2(song_pattern, repeattime_ms);
            } else if (dataPacket->Data.command.command_id == COMMAND_ID_BUZZER_STOPALL) {
                buzzerStopPeriodic();
                HAL_TIM_Base_Stop_IT(&htim11);
            }
            break;
        case COMMAND_TARGET_ACK:
            // ACK commands are on the main board, forward command
            InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_MCU, dataPacket);
            break;
        default:
            // Unknown command target
            break;
    }
}

bool cmd_ack_activated = 0;
// Acknowledge a command with status of its execution as its ID (0=Success, 1=Failed, 2=Invalid)
void InterBoardCom_command_acknowledge(uint8_t command_target, uint8_t command_id, uint8_t status) {
    if (!cmd_ack_activated) {
        return; // Ack not activated
    }

    uint8_t params[2];
    params[0] = command_target;
    params[1] = command_id;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_ACK, status, params, sizeof(params));
    radioSend(&packet);
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