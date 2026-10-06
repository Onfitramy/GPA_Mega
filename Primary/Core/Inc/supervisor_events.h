#ifndef SUPERVISOR_EVENTS_H
#define SUPERVISOR_EVENTS_H

enum {
    SUP_COMPONENT_INTERBOARD = 1U
};

typedef enum {
    IBC_EVENT_TX_ENQUEUED = 0,
    IBC_EVENT_TX_QUEUE_FULL,
    IBC_EVENT_TX_STARTED,
    IBC_EVENT_TX_START_BUSY,
    IBC_EVENT_TX_START_FAILED,
    IBC_EVENT_TRANSFER_COMPLETE,
    IBC_EVENT_SPI_ERROR,
    IBC_EVENT_RX_CHECKSUM_BAD,
    IBC_EVENT_RX_UNKNOWN_ID,
    IBC_EVENT_RX_QUEUE_FULL,
    IBC_EVENT_TX_STALL,

    IBC_EVENT_COUNT
} InterBoardSupervisorEvent_t;

/* First-occurrence context (duplicates update only count and last timestamp):
 * TX_ENQUEUED / TX_QUEUE_FULL: argument = packet ID; metric[0] = TX depth.
 * TX_STARTED / TX_START_BUSY / TX_START_FAILED: argument = packet ID;
 *   metric[0] = HAL status.
 * TRANSFER_COMPLETE: argument = TX packet ID; metric[0] = duration in us.
 * SPI_ERROR: argument = HAL SPI error flags; no metrics.
 * RX_CHECKSUM_BAD: argument = RX packet ID; metric[0] = calculated checksum,
 *   metric[1] = received checksum.
 * RX_UNKNOWN_ID / RX_QUEUE_FULL: argument = RX packet ID; no metrics.
 * TX_STALL: argument = TX packet ID; metric[0] = observed elapsed ms.
 */

#endif
