#ifndef RADIO_H
#define RADIO_H

#include "main.h"
#include "stdbool.h"

#define NRF24L01P_PAYLOAD_LENGTH 32

typedef enum
{
    NRF_24_ACTIVE,
    XBEE_ACTIVE
}radio_status_t;

extern bool nrf_mode;

extern uint8_t radio_rx_data[NRF24L01P_PAYLOAD_LENGTH];

typedef enum
{
    RADIO_MODE_NONE,
    RADIO_MODE_TRANSMITTER,
    RADIO_MODE_RECEIVER,
    RADIO_MODE_TRANSCEIVER
}radio_mode_t;

typedef struct {
    radio_status_t status;
    radio_mode_t mode;
}radio_info_t;

void radioSet(radio_status_t radio);
void radioSend(uint8_t *tx_buf);
void radioSetMode(radio_mode_t mode);
void radioDecode(uint8_t *original_data, uint8_t *fixed_data, uint8_t lenght);

//Exportet Functions from nrf24xx.c
void nrf24l01p_rx_receive(uint8_t* rx_payload);
void nrf24l01p_tx_irq();


#endif // RADIO_H