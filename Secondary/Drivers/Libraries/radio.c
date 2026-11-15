#include "main.h"
#include "packets.h"
#include "xBee.h"
#include "radio.h"
#include "NRF24L01P.h"

SemaphoreHandle_t nrfMutex = NULL;

radio_info_t radio_info;

uint8_t radio_rx_data[NRF24L01P_PAYLOAD_LENGTH];



//Activate needed Radio 0: NRF, 1:XBEE
void radioSet(radio_status_t radio){
    radio_info.status = radio;
}

radio_send_result_t  radioSend(DataPacket_t* dataPacket){
    if(radio_info.status == NRF_24_ACTIVE){
        xSemaphoreTake(nrfMutex, portMAX_DELAY); // Take the mutex to ensure exclusive access to the NRF24L01+ module
        if (radio_info.mode != RADIO_MODE_RECEIVER){
            if (nrf24l01p_get_fifo_status() & 0x20) { // FIFO_STATUS.TX_FULL
                xSemaphoreGive(nrfMutex);
                return RADIO_SEND_BUSY;
            }
            //Set the packet type and determine if acknowledgment is required
            uint8_t packet_type = ((uint8_t)dataPacket->Packet_ID) & 0x1F;
            bool require_ack = (packet_type == PACKET_ID_COMMAND);
            uint8_t status = nrf24l01p_write_tx_fifo((uint8_t*)dataPacket, require_ack);
            if(radio_info.mode == RADIO_MODE_TRANSCEIVER){
                nrf24l01p_txMode();
            }
        }
        xSemaphoreGive(nrfMutex); // Release the mutex
        return RADIO_SEND_QUEUED;
    } else if(radio_info.status == XBEE_ACTIVE){
        XBee_QueueDataPacket(dataPacket);
        return RADIO_SEND_QUEUED;
    }
}

void radioSetMode(radio_mode_t mode){
    if (radio_info.mode == mode) return;
    radio_info.mode = mode;
    if(radio_info.status == NRF_24_ACTIVE){
        if(mode == RADIO_MODE_TRANSMITTER){
            nrf24l01p_txMode();
        } else if(mode == RADIO_MODE_RECEIVER || mode == RADIO_MODE_TRANSCEIVER){
            nrf24l01p_rxMode();
        }
    }
}