#include "radio.h"
#include "NRF24L01P.h"


radio_info_t radio_info;

uint8_t radio_rx_data[NRF24L01P_PAYLOAD_LENGTH];

//Activate needed Radio 0: NRF, 1:XBEE
void radioSet(radio_status_t radio){
    radio_info.status = radio;
}

void radioSend(uint8_t *tx_buf){
    if(radio_info.status == NRF_24_ACTIVE){
        if (radio_info.mode == RADIO_MODE_RECEIVER){
            return; // Cannot send in receiver mode
        }
        uint8_t status = nrf24l01p_write_tx_fifo(tx_buf);
        nrf24l01p_clear_known_irqs(status);

        if(radio_info.mode == RADIO_MODE_TRANSCEIVER){
            nrf24l01p_txMode();
        }
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

/*  Due to a unknown error the radio data is right shifted by 1 bit(maybe CRC activation missmatch)
    This function unshuffles this. In the process the whole 1. bit and some of the 2. bit as well as parts of the last bit get lost, they are not used for now
*/
void radioDecode(uint8_t *original_data, uint8_t *fixed_data, uint8_t lenght){

    for (size_t i = 0; i < lenght; ++i) {
        uint8_t next_bit = (uint8_t)((original_data[i+1] & 0x80) >> 7);
        fixed_data[i] = (uint8_t)((original_data[i] << 1) | next_bit);
    }
}