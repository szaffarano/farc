/*
 * main.c
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#include <spi.h>
#include <nrf.h>
#include <farc.h>
#include <fsm-master.h>

#include <util/delay.h>
#include <avr/interrupt.h>

void systick(void);

static uint8_t address[5] = { 0x22, 0x33, 0x44, 0x55, 0x01 };

int main(void) {
	farc_gpio_init();

	farc_systick_init(systick);

	spi_init();
	nrf_init(NRF_RX, address);

	for (int i = 0; i < 5; i++) {
		set_bit(LEDRX_PORT, LEDRX);
		_delay_ms(70);
		clear_bit(LEDRX_PORT, LEDRX);
		_delay_ms(100);
	}
	clear_bit(LEDRX_PORT, LEDRX);

	sei();
	while (1) {
		_delay_ms(50);
	}

	return 0;
}

void systick(void) {
	fsm_master();
}
