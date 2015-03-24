/*
 * radio.c
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#include <radio/radio.h>
#include <hal/nrf.h>
#include <hw/nrf24l01.h>
#include <util/delay.h>
#include <avr/interrupt.h>

static volatile radio_status_t status;
static uint8_t pload[RF_PAYLOAD_LENGTH];

void radio_init(const uint8_t *address,
		hal_nrf_operation_mode_t operational_mode) {
	hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);        // Power down device

	hal_nrf_close_pipe(HAL_NRF_ALL);              // First close all radio pipes
												  // Pipe 0 and 1 open by default
	hal_nrf_open_pipe(HAL_NRF_PIPE0, true);       // Then open pipe0, w/autoack

	hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);      // Operates in 16bits CRC mode
	hal_nrf_set_auto_retr(RF_RETRANSMITS, RF_RETRANS_DELAY);
	// Enables auto retransmit.
	// 3 retrans with 250ms delay

	// sebas: casteo
	hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);  // 5 bytes address width
	hal_nrf_set_address(HAL_NRF_TX, (uint8_t*) address); // Set device's addresses
	hal_nrf_set_address(HAL_NRF_PIPE0, (uint8_t*) address); // Sets recieving address on
	// pipe0

	/*****************************************************************************
	 * Changed from esb/radio_esb.c                                              *
	 * Enables:                                                                  *
	 *  - ACK payload                                                            *
	 *  - Dynamic payload width                                                  *
	 *  - Dynamic ACK                                                            *
	 *****************************************************************************/
	hal_nrf_enable_ack_pl();                       // Try to enable ack payload

	// When the features are locked, the FEATURE and DYNPD are read out 0x00
	// even after we have tried to enable ack payload. This mean that we need to
	// activate the features.
	if (hal_nrf_read_reg(FEATURE) == 0x00
			&& (hal_nrf_read_reg(DYNPD) == 0x00)) {
		hal_nrf_lock_unlock();                      // Activate features
		hal_nrf_enable_ack_pl();                     // Enables payload in ack
	}

	hal_nrf_enable_dynamic_pl();                   // Enables dynamic payload
	hal_nrf_setup_dyn_pl(ALL_PIPES);               // Sets up dynamic payload on
												   // all data pipes.
	/*****************************************************************************
	 * End changes from esb/radio_esb.c                                          *
	 *****************************************************************************/

	if (operational_mode == HAL_NRF_PTX)            // Mode depentant settings
			{
		hal_nrf_set_operation_mode(HAL_NRF_PTX);     // Enter TX mode
		CE_LOW();
	} else {
		hal_nrf_set_operation_mode(HAL_NRF_PRX);     // Enter RX mode
		hal_nrf_set_rx_pload_width((uint8_t) HAL_NRF_PIPE0, RF_PAYLOAD_LENGTH);
		// Pipe0 expect
		// PAYLOAD_LENGTH byte payload
		// PAYLOAD_LENGTH in radio.h
		CE_HIGH();
	}

	hal_nrf_set_rf_channel(RF_CHANNEL);           // Operating on static channel
												  // Defined in radio.h.
												  // Frequenzy =
												  //        2400 + RF_CHANNEL
	hal_nrf_set_power_mode(HAL_NRF_PWR_UP);        // Power up device

	_delay_ms(RF_POWER_UP_DELAY);                // Wait for the radio to

	radio_set_status(RF_IDLE);                    // Radio now ready

	// PCINT para NRF_IRQ (PCINT2)
	clear_bit(NRF_IRQ_DDR, NRF_IRQ);
	GIMSK |= _BV(PCIE); /* pin change interrupt enable ... */
	PCMSK |= _BV(PCINT2); /* ... para pcint2 solamente */
}

radio_status_t radio_get_status(void) {
	return status;
}

void radio_set_status(radio_status_t new_status) {
	status = new_status;
}

uint8_t radio_get_pload_byte(uint8_t byte_index) {
	return pload[byte_index];
}

void radio_send_packet(uint8_t *packet, uint8_t length) {
	hal_nrf_write_tx_pload(packet, length);
	CE_PULSE()
	;
	radio_set_status(RF_BUSY);
}

ISR(PCINT_vect) {
	radio_irq();
}

void radio_irq(void) {
	switch (hal_nrf_get_clear_irq_flags()) {
	case (1 << HAL_NRF_MAX_RT):
		// El PTX alcanzó la máxima cantidad de reintentos al enviar un paquete
		hal_nrf_flush_tx();
		radio_set_status(RF_MAX_RT);
		break;
	case (1 << HAL_NRF_TX_DS):
		// El PTX recibió una respuesta ACK ( = el paquete se envió OK)
		radio_set_status(RF_TX_DS);
		break;
	case (1 << HAL_NRF_RX_DR):
		// El PRX recibió un paquete nuevo
		while (!hal_nrf_rx_fifo_empty()) {
			hal_nrf_read_rx_pload(pload);
		}
		radio_set_status(RF_RX_DR);
		break;
	case ((1 << HAL_NRF_RX_DR) | (1 << HAL_NRF_TX_DS)):
		// 1. El PTX recibió una respuesta ACK con payload
		// 2. El PRX recibió el ACK del <ACK con payload> que envió
		while (!hal_nrf_rx_fifo_empty()) {
			hal_nrf_read_rx_pload(pload);
		}
		radio_set_status(RF_TX_AP);
		break;
	default:
		break;
	}
}
