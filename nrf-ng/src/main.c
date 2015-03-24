/*
 * nrf-ng.c
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#include <nrf-ng.h>
#include <stdint.h>
#include <radio/radio.h>
#include <hal/nrf.h>
#include <hw/nrf24l01.h>
#include <avr/interrupt.h>

static uint8_t pload_esb[RF_PAYLOAD_LENGTH];

int main(void) {
	debounce_event event;
	radio_status_t status;

	nrf_ng_init();

	nrf_ng_boot_msg();

	sei();

	pload_esb[0] = 0xBE;
	while (true) {
		status = radio_get_status();

		switch (status) {
		case RF_IDLE: /* radio IDLE */
			break;
		case RF_MAX_RT: /* máxima cantidad de reintentos */
			hal_nrf_set_operation_mode(HAL_NRF_PRX);
			radio_set_status(RF_IDLE);
			LEDB_ON();
			break;
		case RF_TX_DS: /* se envio el paquete */
			hal_nrf_set_operation_mode(HAL_NRF_PRX);
			radio_set_status(RF_IDLE);
			LEDB_TGL();
			break;
		case RF_RX_DR: /* se recibió un paquete */
			if (radio_get_pload_byte(0) == 0xBE) {
				LEDA_TGL();
			}
			radio_set_status(RF_IDLE);
			break;
		case RF_TX_AP: /* se recibió un paquete ACK */
			radio_set_status(RF_IDLE);
			break;
		case RF_BUSY: /* radio BUSY */
			break;
		default: /* no debería... */
			break;
		};

		event = farc_debounce();
		if (event == FELL) {
			hal_nrf_set_operation_mode(HAL_NRF_PTX);
			radio_send_packet(pload_esb, RF_PAYLOAD_LENGTH);
		}

		_delay_ms(1);
	}
}
