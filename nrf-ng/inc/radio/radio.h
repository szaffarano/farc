/*
 * radio.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <stdint.h>
#include <hal/nrf_reg.h>

#define RF_CHANNEL 			40
#define RF_POWER_UP_DELAY 	2
#define RF_PAYLOAD_LENGTH 	10
#define RF_RETRANSMITS 		15
#if (RF_PAYLOAD_LENGTH <= 18)
#define RF_RETRANS_DELAY 	250
#else
#define RF_RETRANS_DELAY 	500
#endif

typedef enum {
	RF_IDLE, RF_MAX_RT, RF_TX_DS, RF_RX_DR, RF_TX_AP, RF_BUSY
} radio_status_t;

typedef enum {
	HAL_NRF_MAX_RT = 4, HAL_NRF_TX_DS, HAL_NRF_RX_DR
} hal_nrf_irq_source_t;

void radio_init(const uint8_t *address,
		hal_nrf_operation_mode_t operational_mode);

radio_status_t radio_get_status(void);

void radio_set_status(radio_status_t new_status);

uint8_t radio_get_pload_byte(uint8_t byte_index);

void radio_send_packet(uint8_t *packet, uint8_t length);

void radio_irq(void);

#endif /* RADIO_H_ */
