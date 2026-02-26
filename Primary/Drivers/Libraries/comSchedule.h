#ifndef COMSCHEDULE_H
#define COMSCHEDULE_H
// This library handles routing of data, decides which data should be sent when and with what freq

#include "main.h"
#include "packets.h"

#define MAX_NUM_MESSAGES 16

// Describes the central mode of communication
typedef enum {
    COMM_MODE_FORWARDING, // Data is received via Radio and forwarded to the PC via USB (to be used on groundstation)
    COMM_MODE_REMOTE_TRANSMIT, // Data is sent via Radio from the secondary board, but not forwarded to the PC via USB (to be used on flight)
    COMM_MODE_LOCAL, // Data is not sent via Radio, only forwarded to the PC via USB (to be used for testing with only one board)
} CommunicationMode_t;

typedef struct {
    uint32_t period; // in ms, how often the packet should be sent, max x. Zero means it should never be sent
    uint32_t last_sent_tick; // xTaskGetTickCount() value of the last time this packet was sent
    DataPacket_t *packet; //Pointer to the DataPacket_t packet that holds the data to be sent, this allows the update function to directly modify the packet data
}message_info_t;

void InitializeComSchedule();
void UpdateComSchedule(uint32_t* new_frequencies);
void ProcessComSchedule(uint32_t current_tick);

#endif // COMSCHEDULE_H