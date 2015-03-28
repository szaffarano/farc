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
#include <string.h>

static uint8_t pload[RF_PAYLOAD_LENGTH];
static const uint8_t address[5] = RF_ADDRESS;
static uint8_t rf_control;

void radio_init(hal_nrf_operation_mode_t operational_mode) {
	// Habilito CRC con encoding de 16 bits y pongo en power up.
	uint8_t config = mask(EN_CRC) | mask(CRCO) | mask(PWR_UP);

	hal_nrf_write_reg(CONFIG, mask(CRCO));
	delay_ms(RF_POWER_UP_DELAY);

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

	if (operational_mode == HAL_NRF_PRX) {
		// Agrego config para PRX
		config |= mask(PRIM_RX);
		ce_high();
	}
	hal_nrf_write_reg(CONFIG, config);

	// Espero para que se estabilice el power up
	delay_ms(RF_POWER_UP_DELAY);

	// Fin de inicialización, radio idle
	rf_control = ~(mask(RF_DATA_AVAILABLE) | mask(RF_BUSY) | mask(RF_MAX_RT));
}

uint8_t radio_get_control_register(void) {
	return rf_control;
}

bool radio_data_available(void) {
	return (rf_control & mask(RF_DATA_AVAILABLE));
}

void radio_get_pload(uint8_t* buffer) {
	clear_bit(rf_control, RF_DATA_AVAILABLE);
	memcpy(buffer, pload, RF_PAYLOAD_LENGTH);
}

bool radio_send_packet(uint8_t *packet, uint8_t length) {
	if (!(rf_control & mask(RF_BUSY))) {
		hal_nrf_write_tx_pload(packet, length);
		set_bit(rf_control, RF_BUSY);
		return true;
	}
	return false;
}

void radio_irq(void) {
	uint8_t status =
			hal_nrf_read_reg(STATUS)
					& (mask(HAL_NRF_MAX_RT) | mask(HAL_NRF_TX_DS)
							| mask(HAL_NRF_RX_DR));

	switch (status) {
	case (mask(HAL_NRF_MAX_RT)):
		// El PTX alcanzó la máxima cantidad de reintentos al enviar un paquete
		hal_nrf_flush_tx();
		set_bit(rf_control, RF_MAX_RT);
		clear_bit(rf_control, RF_BUSY);
		break;
	case (mask(HAL_NRF_TX_DS)):
		// El PTX recibió una respuesta ACK ( = el paquete se envió OK)
		clear_bit(rf_control, RF_BUSY);
		break;
	case (mask(HAL_NRF_RX_DR)):
		// El PRX recibió un paquete nuevo
		while (!hal_nrf_rx_fifo_empty()) {
			hal_nrf_read_rx_pload(pload);
		}
		set_bit(rf_control, RF_DATA_AVAILABLE);
		clear_bit(rf_control, RF_BUSY);
		break;
	case ((mask(HAL_NRF_RX_DR)) | (mask(HAL_NRF_TX_DS))):
		// 1. El PTX recibió una respuesta ACK con payload
		// 2. El PRX recibió el ACK del <ACK con payload> que envió
		while (!hal_nrf_rx_fifo_empty()) {
			hal_nrf_read_rx_pload(pload);
		}
		set_bit(rf_control, RF_DATA_AVAILABLE);
		clear_bit(rf_control, RF_BUSY);
		break;
	default:
		break;
	}

	hal_nrf_write_reg(STATUS,
			(mask(HAL_NRF_MAX_RT) | mask(HAL_NRF_TX_DS) | mask(HAL_NRF_RX_DR)));
}
