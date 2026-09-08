#ifndef InterBoardCom_H_
#define InterBoardCom_H_

#include "_components.h"
#include "packets.h"

#define INTERBOARD_BUFFER_SIZE 32U
#define INTERBOARD_DIAG_STALL_THRESHOLD_MS 50U

//The ID describe what should be expected and done with the data received
//The top bit of the ID describes if there are more packets to follow or if it is the last packet (for 10ms till next scheduled packets)
typedef enum __attribute__((packed)){
    // Status Packets:
    // Operation Types (bits 0-2, lower nibble)
    INTERBOARD_OP_NONE         = 0x00,
    INTERBOARD_OP_SAVE_SEND     = 0x01,  // Save data or send it via radio
    INTERBOARD_OP_LOAD_REQUEST  = 0x02,  // Load data  or request it from another board
    INTERBOARD_OP_CMD           = 0x04,  // Command operation

    // Target Types (bits 3-6, upper nibble)
    INTERBOARD_TARGET_NONE = 0x00,
    INTERBOARD_TARGET_FLASH = 0x08,
    INTERBOARD_TARGET_SD    = 0x10,
    INTERBOARD_TARGET_RADIO = 0x20,
    INTERBOARD_TARGET_MCU   = 0x40,
    INTERBOARD_TARGET_FOLLOWING = 0x80, // Indicates more packets to follow

    // Unusual combined types
    INTERBOARD_OP_ECHO = INTERBOARD_OP_CMD | INTERBOARD_TARGET_NONE, // Echo command
    INTERBOARD_OP_DEBUG_VIEW = INTERBOARD_OP_SAVE_SEND | INTERBOARD_TARGET_NONE, // Send for debugging to PC
} InterBoardPacketID_t;

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_spi1_rx;

extern DataPacket_t powerData; //For receiving power data from secondary

//Wrapper for the DataPacket to be used to send via SPI1
#pragma pack(push, 1)
typedef struct {
    uint8_t InterBoardPacket_ID;
    uint8_t Data[32];
} InterBoardPacket_t;
#pragma pack(pop)

typedef struct {
    InterBoardPacket_t buffer[INTERBOARD_BUFFER_SIZE];
    volatile uint16_t head;      // Write index
    volatile uint16_t tail;      // Read index
    volatile uint16_t count;     // Number of items in buffer
} InterBoardCircularBuffer_t;

/*
 * Master-side link diagnostics.  These counters are observational only: a bad
 * CRC is counted but the packet is still passed to the existing parser.
 * InterBoardCom_Diagnostics is intentionally global so it can also be watched
 * directly in a debugger when the CLI is unavailable.
 */
typedef struct {
    uint32_t reset_at_ms;

    uint32_t tx_enqueue_attempts;
    uint32_t tx_enqueued;
    uint32_t tx_queue_full;
    uint32_t tx_queue_high_water;
    uint32_t tx_queue_depth;
    uint32_t tx_dequeued;

    uint32_t tx_start_attempts;
    uint32_t tx_started;
    uint32_t tx_start_busy;
    uint32_t tx_start_error;
    uint32_t tx_completed;
    uint32_t tx_stall_observations;
    uint32_t last_transfer_us;
    uint32_t max_transfer_us;
    uint32_t last_tx_start_ms;
    uint32_t last_tx_complete_ms;
    uint32_t last_tx_id;
    uint32_t last_hal_status;

    uint32_t spi_error_callbacks;
    uint32_t last_spi_error;

    uint32_t rx_frames;
    uint32_t rx_none;
    uint32_t rx_echo;
    uint32_t rx_data_frames;
    uint32_t rx_crc_ok;
    uint32_t rx_crc_bad;
    uint32_t rx_unknown_id;
    uint32_t rx_queue_enqueued;
    uint32_t rx_queue_full;
    uint32_t rx_processed;
    uint32_t last_rx_ms;
    uint32_t last_rx_id;
    uint32_t last_rx_crc_calculated;
    uint32_t last_rx_crc_received;

    uint32_t task_wait_returns;
    uint32_t task_wait_zero_returns;
    uint32_t app_spi_state;
    uint32_t hal_spi_state;
} InterBoardComDiagnostics_t;

extern volatile InterBoardComDiagnostics_t InterBoardCom_Diagnostics;

// Function prototypes
void InterBoardBuffer_Init(InterBoardCircularBuffer_t* cb);
uint8_t InterBoardBuffer_Push(InterBoardCircularBuffer_t* cb, InterBoardPacket_t* packet);
uint8_t InterBoardBuffer_Pop(InterBoardCircularBuffer_t* cb, InterBoardPacket_t* packet);
uint8_t InterBoardBuffer_IsEmpty(InterBoardCircularBuffer_t* cb);
uint8_t InterBoardBuffer_IsFull(InterBoardCircularBuffer_t* cb);
uint16_t InterBoardBuffer_Count(InterBoardCircularBuffer_t* cb);
void InterBoardBuffer_Clear(InterBoardCircularBuffer_t* cb);

uint8_t USB_QueueDataPacket(DataPacket_t *packet);
uint8_t USB_OutputDataPacket(DataPacket_t *packet);

void InterBoardCom_Init(void);
uint8_t InterBoardCom_QueuePacket(InterBoardPacket_t *packet);
void InterBoardCom_ProcessTxBuffer(void);
void InterBoardCom_SendTestPacket(void);
void InterBoardCom_SendDataPacket(InterBoardPacketID_t Inter_ID, DataPacket_t *packet);
InterBoardPacket_t InterBoardCom_ReceivePacket(void);
void InterBoardCom_ProcessReceivedPacket(InterBoardPacket_t *packet);
void InterBoardCom_ParsePacket(InterBoardPacket_t *packet);
void InterBoardCom_ActivateReceive(void);
void InterBoardCom_command_acknowledge(uint8_t command_target, uint8_t command_id, uint8_t status);

InterBoardPacket_t InterBoardCom_CreatePacket(InterBoardPacketID_t ID);
void InterBoardCom_FillRaw(InterBoardPacket_t *packet, int num, ...);
void InterBoardCom_FillData(InterBoardPacket_t *packet, DataPacket_t *data_packet);

void InterBoardCom_ResetDiagnostics(void);
void InterBoardCom_GetDiagnostics(InterBoardComDiagnostics_t *snapshot);
void InterBoardCom_DiagnosticsRecordTransferComplete(void);
void InterBoardCom_DiagnosticsRecordSpiError(uint32_t spi_error);
void InterBoardCom_DiagnosticsRecordRx(const InterBoardPacket_t *packet);
void InterBoardCom_DiagnosticsRecordRxQueueResult(uint8_t queued);
void InterBoardCom_DiagnosticsRecordRxProcessed(void);
void InterBoardCom_DiagnosticsRecordTaskWait(uint32_t notification_value);

#endif /* InterBoardCom_H_ */
