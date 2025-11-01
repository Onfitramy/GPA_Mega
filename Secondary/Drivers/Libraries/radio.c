#include "main.h"
#include "packets.h"
#include "XBee.h"
#include "radio.h"
#include "NRF24L01P.h"


radio_info_t radio_info;

uint8_t radio_rx_data[NRF24L01P_PAYLOAD_LENGTH];

//Activate needed Radio 0: NRF, 1:XBEE
void radioSet(radio_status_t radio){
    radio_info.status = radio;
}

void radioSend(DataPacket_t* dataPacket){
    if(radio_info.status == NRF_24_ACTIVE){
        if (radio_info.mode == RADIO_MODE_RECEIVER){
            return; // Cannot send in receiver mode
        }
        uint8_t status = nrf24l01p_write_tx_fifo((uint8_t*)dataPacket);
        nrf24l01p_clear_known_irqs(status);

        if(radio_info.mode == RADIO_MODE_TRANSCEIVER){
            nrf24l01p_txMode();
        }
    } else if(radio_info.status == XBEE_ACTIVE){
        XBee_QueueDataPacket(dataPacket);
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