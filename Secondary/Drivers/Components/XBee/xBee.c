#include "xBee.h"
#include "InterBoardCom.h"
#include "string.h"

#define XBEE_CS_GPIO_PIN GPIO_PIN_4
#define XBEE_CS_GPIO_PORT GPIOD

extern UART_HandleTypeDef huart1;

DataCircularBuffer_t xBeeRxCircBuffer; // incoming buffer for outgoing packets

volatile int8_t XBee_Status = -1; // -1 = Initializing, 0 = Ready, 1 = Error
static void XBee_StartupBegin(void);

uint8_t XBee_Init(void)
{
    // Initialize circular buffer
    if (XBee_Status == 0) {
        return 0; // Already initialized
    }
    DataCircBuffer_Init(&xBeeRxCircBuffer);

    XBee_UART_RestartReceive();
    XBee_changeBaudRate(115200); // Change XBee baudrate to 115200
    HAL_Delay(100);
    XBee_StartupBegin();
    return 0;
}

uint8_t XBee_checkCRC(uint8_t* data, uint16_t length, uint8_t crc);
void XBee_Transmit(uint8_t* data, uint16_t length, uint64_t destinationAddress);

/**
 * @brief Constructs an XBee Transmit Request frame.
 *
 * @param frame Pointer to a buffer to be populated.
 * @param frame_id The frame ID to assign to this frame (used for response tracking).
 * @param dest_addr The 64-bit destination address for the frame (set as 0 to indicate broadcast).
 * @param frame_payload Pointer to the payload data to be transmitted.
 * @param payload_length Length of the payload data in bytes.
 */
void constructTransmitFrame(uint8_t* frame, uint8_t frame_id, uint64_t dest_addr, uint8_t* frame_payload, uint16_t payload_length)
{
    if (dest_addr == 0) {
        dest_addr = 0x000000000000FFFF; // Broadcast address
    }

    frame[0] = 0x7E;
    frame[1] = (payload_length + 14) >> 8; // High byte
    frame[2] = (payload_length + 14) & 0xFF; // Low byte
    frame[3] = 0x10; // AT Command frame
    frame[4] = frame_id;
    frame[5] = (dest_addr >> 56) & 0xFF;
    frame[6] = (dest_addr >> 48) & 0xFF;
    frame[7] = (dest_addr >> 40) & 0xFF;
    frame[8] = (dest_addr >> 32) & 0xFF;
    frame[9] = (dest_addr >> 24) & 0xFF;
    frame[10] = (dest_addr >> 16) & 0xFF;
    frame[11] = (dest_addr >> 8) & 0xFF;
    frame[12] = dest_addr & 0xFF;
    frame[13] = 0xFF;
    frame[14] = 0xFE;
    frame[15] = 0x00; // Maximum hops
    frame[16] = 0x40; // Deleivery method point to multipoint
    for (uint16_t i = 0; i < payload_length; i++) {
        frame[17 + i] = frame_payload[i];
    }
    // Calculate and append checksum
    uint64_t checksum = 0;
    // Add all bytes of the packet, except the start delimiter 0x7E and the length (the second and third bytes).
    for (uint16_t i = 3; i < 17 + payload_length; i++) {
        checksum += frame[i];
    }
    //Keep only the lowest 8 bits from the result and subtract this from 0xFF.
    frame[17 + payload_length] = 0xFF - (checksum & 0xFF);

    XBee_checkCRC(&frame[3], 14 + payload_length , frame[17 + payload_length]);
}

uint8_t XBee_calcCRC(uint8_t* data, uint16_t length)
{
    uint64_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return 0xFF - (checksum & 0xFF);
}

uint8_t XBee_checkCRC(uint8_t* data, uint16_t length, uint8_t crc)
{
    uint64_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    if (((checksum + crc) & 0xFF) == 0xFF) {
        return 1; // Valid CRC
    }
    uint8_t corr_checksum = 0xFF - (checksum & 0xFF);
    return 0; // Invalid CRC
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
uint8_t UART_LOCK = 0;
uint8_t UART_LOCK_ERROR = 0;
volatile uint32_t XBEE_UART_TxStartOK = 0;
volatile uint32_t XBEE_UART_TxStartBusy = 0;
volatile uint32_t XBEE_UART_TxStartErrors = 0;
HAL_StatusTypeDef sendAPIFrame(uint8_t* frame, uint16_t frame_length)
{
    /*if (XBee_Status != 0) {
        return; // XBee not initialized or in error state
    }*/
    HAL_StatusTypeDef res = HAL_UART_Transmit_IT(&huart1, frame, frame_length + 4); //Convert frame_length to bytes
    if (res == HAL_OK) {
        XBEE_UART_TxStartOK++;
    } else if (res == HAL_BUSY) {
        XBEE_UART_TxStartBusy++;
    } else {
        XBEE_UART_TxStartErrors++;
    }
    return res;
}

void xBee_changeState(uint8_t state)
{
    XBee_Status = state;
}

uint8_t frame[XBEE_MAX_RF_PAYLOAD_SIZE]; // 256 bytes max payload + overhead
uint8_t response[XBEE_MAX_FRAME_SIZE];

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
    HAL_Delay(1100);
    command_sequence[0] = 'A';
    command_sequence[1] = 'T';
    command_sequence[2] = 'W';
    command_sequence[3] = 'R';
    command_sequence[4] = '\r';
    HAL_UART_Transmit(&huart1, command_sequence, 5, HAL_MAX_DELAY);
}

xbee_at_cmd_t at_frame;

typedef struct {
    char command[2];
    uint8_t length;
    uint8_t value[4];
} XBee_StartupSetting;

static const XBee_StartupSetting startup_settings[] = {
    {{'B', 'R'}, 1, {1}},
    {{'P', 'L'}, 1, {4}},
    {{'D', 'R'}, 1, {1}},
    {{'C', 'M'}, 4, {0x3E, 0xFF, 0xFD, 0xFF}},
    {{'R', 'R'}, 1, {0x0A}},
    {{'S', 'M'}, 1, {0}},
    // Preserve the existing network ID on both boards.
    {{'H', 'P'}, 1, {0}},
    {{'A', 'P'}, 1, {1}},
    {{'A', 'O'}, 1, {0}},
    {{'A', 'C'}, 0, {0}}, // Apply changes; deliberately do not persist with WR.
};
static xbee_at_cmd_t startup_frame;
static bool startup_active;
static bool startup_waiting;
static uint32_t startup_sent_at;
volatile uint8_t XBEE_StartupStep;
volatile uint8_t XBEE_StartupAttempts;
volatile uint16_t XBEE_StartupCommand;
volatile uint8_t XBEE_StartupLastStatus; // 0xFF = no response, 0xFE = UART failure

static void XBee_StartupBegin(void)
{
    XBee_Status = -1;
    XBEE_StartupStep = 0;
    XBEE_StartupAttempts = 0;
    XBEE_StartupLastStatus = 0xFF;
    startup_waiting = false;
    startup_active = true;
}

// Called by the interrupt task without blocking its response processing.
void XBee_StartupPoll(void)
{
    if (!startup_active) return;
    if (startup_waiting && (uint32_t)(HAL_GetTick() - startup_sent_at) < 500U) return;
    if (XBEE_StartupAttempts >= 3) {
        XBee_Status = 1;
        startup_active = false;
        return;
    }
    if (huart1.gState != HAL_UART_STATE_READY) return;

    const XBee_StartupSetting *setting = &startup_settings[XBEE_StartupStep];
    XBEE_StartupCommand = ((uint16_t)setting->command[0] << 8) | setting->command[1];
    constructATFrame(&startup_frame, 0x80 + XBEE_StartupStep,
                     XBEE_StartupCommand, (uint8_t *)setting->value, setting->length);
    HAL_StatusTypeDef result = sendAPIFrame((uint8_t *)&startup_frame, 4 + setting->length);
    XBEE_StartupAttempts++;
    startup_sent_at = HAL_GetTick();
    startup_waiting = true;
    XBEE_StartupLastStatus = result == HAL_OK ? 0xFF : 0xFE;
}

void XBee_StartupHandleResponse(const xbee_frame_t *packet)
{
    uint16_t length = ((uint16_t)packet->frame_length[0] << 8) | packet->frame_length[1];
    // Write/AC responses contain exactly five frame-data bytes. The queue
    // copies raw UART bytes, so the checksum follows that variable-length data.
    if (!startup_active || !startup_waiting || length != 5 ||
        packet->frame_data[0] != 0x88 ||
        packet->frame_data[1] != 0x80 + XBEE_StartupStep ||
        (((uint16_t)packet->frame_data[2] << 8) | packet->frame_data[3]) != XBEE_StartupCommand)
        return;
    if (XBee_calcCRC((uint8_t *)packet->frame_data, length) != ((const uint8_t *)packet)[3 + length])
        return;

    XBEE_StartupLastStatus = packet->frame_data[4];
    if (XBEE_StartupLastStatus != 0) {
        XBee_Status = 1;
        startup_active = false;
        return;
    }
    startup_waiting = false;
    XBEE_StartupAttempts = 0;
    XBEE_StartupStep++;
    if (XBEE_StartupStep == sizeof(startup_settings) / sizeof(startup_settings[0])) {
        startup_active = false;
        XBee_Status = 0;
    }
}

void XBee_TestDeviceIdentifier()
{
    constructATFrame(&at_frame, 0x01, ('D' << 8) | 'D', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] | (at_frame.frame_length[1] << 8));
}

void XBee_GetTemperature()
{
    constructATFrame(&at_frame, 0x01, ('T' << 8) | 'P', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}
uint8_t xbee_frame_buffer[350]; // 256 bytes max payload + overhead

uint8_t XBee_QueueDataPacket(DataPacket_t* data)
{
    return DataCircBuffer_Push(&xBeeRxCircBuffer, data);
}

void XBee_TransmitQueue(uint64_t destinationAddress)
{
    if (XBee_Status != 0 || UART_LOCK || huart1.gState != HAL_UART_STATE_READY) {
        return;
    }
    const uint16_t max_payload_bytes = sizeof(frame);
    uint16_t curr_payload_bytes = 0;

    if (xBeeRxCircBuffer.count == 0) {
        return; // No data to send
    }

    while (xBeeRxCircBuffer.count > 0 && (curr_payload_bytes + sizeof(DataPacket_t)) <= max_payload_bytes) {
        DataCircBuffer_Pop(&xBeeRxCircBuffer, (DataPacket_t*)&frame[curr_payload_bytes]);
        curr_payload_bytes += sizeof(DataPacket_t);
    }
    XBee_Transmit((uint8_t*)&frame, curr_payload_bytes, destinationAddress);
}

void XBee_Transmit(uint8_t* data, uint16_t length, uint64_t destinationAddress)
{
    if (XBee_Status != 0) return;
    if (UART_LOCK) {
        UART_LOCK_ERROR += 1;
        return; // UART is busy
    }
    UART_LOCK = 1;
    constructTransmitFrame(xbee_frame_buffer, 0x01, destinationAddress, data, length);
    sendAPIFrame(xbee_frame_buffer, xbee_frame_buffer[1] << 8 | xbee_frame_buffer[2]);
}

void XBee_ReceivedErrorCount()
{
    constructATFrame(&at_frame, 0x01, ('E' << 8) | 'R', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_TransmitErrorCount()
{
    constructATFrame(&at_frame, 0x01, ('T' << 8) | 'R', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_changeBaudRate(uint32_t baudrate)
{
    uint8_t baud_payload[1];
    switch(baudrate) {
        case 9600:
            baud_payload[0] = 0x03;
            break;
        case 19200:
            baud_payload[0] = 0x04;
            break;
        case 38400: 
            baud_payload[0] = 0x05;
            break;
        case 57600:
            baud_payload[0] = 0x06;
            break;
        case 115200:
            baud_payload[0] = 0x07;
            break;
        case 230400:
            baud_payload[0] = 0x08;
    }
    constructATFrame(&at_frame, 0x01, ('B' << 8) | 'D', baud_payload, 1);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
    HAL_Delay(100);
    HAL_UART_Abort(&huart1);
    huart1.Init.BaudRate = baudrate;
    HAL_UART_Init(&huart1);
    XBee_UART_RestartReceive(); // Discard any partial frame from the old baud rate
}

void XBee_ReadChannelMask()
{
    constructATFrame(&at_frame, 0x01, ('C' << 8) | 'M', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_ReadDataRate()
{
    constructATFrame(&at_frame, 0x01, ('B' << 8) | 'R', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

// Data Rate: 0 = 10kb/s, 1 = 80kb/s
void XBee_changeDataRate(uint8_t dataRate)
{
    uint8_t dataRate_payload[1];
    dataRate_payload[0] = dataRate;
    constructATFrame(&at_frame, 0x01, ('B' << 8) | 'R', dataRate_payload, 1);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_changePowerLevel(uint8_t powerLevel)
{
    uint8_t powerLevel_payload[1];
    powerLevel_payload[0] = powerLevel;
    constructATFrame(&at_frame, 0x01, ('P' << 8) | 'L', powerLevel_payload, 1);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_SaveSettings()
{
    constructATFrame(&at_frame, 0x01, ('W' << 8) | 'R', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_ReadHardwareAddressHigh(void)
{
    constructATFrame(&at_frame, 0x01, ('S' << 8) | 'H', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

void XBee_ReadHardwareAddressLow(void)
{
    constructATFrame(&at_frame, 0x01, ('S' << 8) | 'L', NULL, 0);
    sendAPIFrame((uint8_t*)&at_frame, at_frame.frame_length[0] << 8 | at_frame.frame_length[1]);
}

//Parses a received RF frame and splits it into the different packets inside
void XBee_parseReceivedRFFrame(xbee_frame_t* frame)
{
    // Process received RF data
    uint8_t* packetData = &frame->frame_data[12]; // RF data starts at byte 12
    uint16_t packetDataLength = (frame->frame_length[0] << 8 | frame->frame_length[1]) - 12; // Length of RF data

    uint8_t numPackets = packetDataLength / sizeof(DataPacket_t);
    while (numPackets > 0) {
        uint8_t packet_id =  packetData[0];
        DataPacket_t receivedData = CreateDataPacket(packet_id);
        memcpy(&receivedData.timestamp, &packetData[1], sizeof(uint32_t));
        memcpy(&receivedData.Data, &packetData[5], sizeof(receivedData.Data));
        receivedData.crc = packetData[31];

        //Check CRC
        uint8_t valid_crc = InterBoard_CheckCRC(&receivedData);

        if (!valid_crc) {
            // Handle invalid CRC
            return;
        }

        // Handle command packet from the XBee
        if (receivedData.Packet_ID == PACKET_ID_COMMAND) {
            InterBoardCom_EvaluateCommand(&receivedData, PACKET_SOURCE_EXTERNAL);
            // For example, you can check the command type and execute corresponding actions
        } else {
            //Send data packets to main board for display/logging
            InterBoardCom_SendDataPacket(INTERBOARD_OP_DEBUG_VIEW, &receivedData);
        }

        packetData += sizeof(DataPacket_t); // Move to the next packet
        numPackets--;
    }
}

//void XBee_BufferT
