/*
 * radio.c
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#include <radio/radio.h>
#include <hal/nrf.h>
#include <hw/nrf24l01.h>
#include <utils.h>

static volatile radio_status_t status;
static uint8_t pload[RF_PAYLOAD_LENGTH];

void radio_init(const uint8_t *address,
		hal_nrf_operation_mode_t operational_mode) {
	uint8_t config = mask(EN_CRC);

	// Pongo device en power down (registro CONFIG con su valor de reset)
	hal_nrf_write_reg(CONFIG, config);

	// Cierro todos los pipes
	hal_nrf_write_reg(EN_RXADDR, 0);
	hal_nrf_write_reg(EN_AA, 0);

	// Abro pipe0 con auto ack
	hal_nrf_write_reg(EN_RXADDR, mask(0));
	hal_nrf_write_reg(EN_AA, mask(0));

	// auto retransmit delay = SETUP_RETR[7:4]: 0b0001 = 500 uSec
	// auto retransmit count = SETUP_RETR[3:0]: 0b1111 = 15
	hal_nrf_write_reg(SETUP_RETR, 0b00011111);

	// address width: 0b11 = 5bytes
	hal_nrf_write_reg(SETUP_AW, 0b11);

	// Seteo dirección para TX y RX en pipe0
	hal_nrf_write_multibyte_reg(HAL_NRF_TX, (uint8_t*) address, 0);
	hal_nrf_write_multibyte_reg(HAL_NRF_PIPE0, (uint8_t*) address, 0);

	// Seteo payload length en pipe0 (solo aplicable en modo PRX)
	hal_nrf_set_rx_pload_width((uint8_t) HAL_NRF_PIPE0, RF_PAYLOAD_LENGTH);

	// FEATURE[1]: enable payload with ack
	// FEATURE[2]: enable dinamic payload length
	hal_nrf_write_reg(FEATURE, 0b110);

	// Habilito payload de longitud dinamico en pipe0
	hal_nrf_write_reg(DYNPD, mask(0));

	// Seteo frecuencia de operación: 2400 + ${RF_CHANNEL}
	hal_nrf_set_rf_channel(RF_CHANNEL);

	// Habilito CRC con encoding de 16 bits y pongo en power up
	config |= mask(EN_CRC) | mask(CRCO) | mask(PWR_UP);
	if (operational_mode == HAL_NRF_PRX) {
		config |= mask(PRIM_RX);
		ce_high();
	}
	hal_nrf_write_reg(CONFIG, config);

	// Espero para que se estabilice el power up
	delay_ms(RF_POWER_UP_DELAY);

	// Fin de inicialización, radio idle
	radio_set_status(RF_IDLE);
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
	radio_set_status(RF_BUSY);
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
