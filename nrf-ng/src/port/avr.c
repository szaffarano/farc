/*
 * avr.c
 *
 *  Created on: 24/03/2015
 *      Author: sebas
 */

#include <port/avr.h>
#include <util/delay.h>
#include <spi/spi.h>
#include <utils.h>

void delay_ms(uint16_t count) {
	while (count--) {
		_delay_ms(1);

	}
}

void delay_us(uint16_t count) {
	while (count--) {
		_delay_us(1);

	}
}

void csn_low(void) {
	spi_start_trade();
}

void csn_high(void) {
	spi_stop_trade();
}

void ce_low(void) {
	clear_bit(NRF_CE_PORT, NRF_CE);
}

void ce_high(void) {
	set_bit(NRF_CE_PORT, NRF_CE);
}

void ce_pulse(void) {
	ce_high();
	delay_us(20);
	ce_low();
}
