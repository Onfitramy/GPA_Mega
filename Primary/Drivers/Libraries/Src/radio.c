#include "radio.h"

radio_info_t radio_info;

uint8_t radio_rx_data[NRF24L01P_PAYLOAD_LENGTH];

//Activate needed Radio 0: NRF, 1:XBEE
void radioSet(radio_status_t radio){
    radio_info.status = radio;
}

void radioSend(uint8_t *tx_buf){
    if(radio_info.status == NRF_24_ACTIVE){
        nrf24l01p_write_tx_fifo(tx_buf);

        if(radio_info.mode != RADIO_MODE_TRANSMITTER){
            nrf24l01p_txMode();
            delay_us(500);
            nrf24l01p_rxMode();
        }
    }
}

void radioSetMode(radio_mode_t mode){
    radio_info.mode = mode;
    if(radio_info.status == NRF_24_ACTIVE){
        if(mode == RADIO_MODE_TRANSMITTER){
            nrf24l01p_txMode();
        } else if(mode == RADIO_MODE_RECEIVER || mode == RADIO_MODE_TRANSCEIVER){
            nrf24l01p_rxMode();
        }
    }
}