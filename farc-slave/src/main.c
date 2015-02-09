/*
 * main.c
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#include <spi.h>
#include <nrf.h>
#include <farc.h>

#include <util/delay.h>
#include <avr/interrupt.h>

void systick(void);

int main(void) {
	uint8_t address[5] = { 0x22, 0x33, 0x44, 0x55, 0x01 };
	uint8_t data;

	farc_gpio_init();

	farc_systick_init(systick);

	spi_init();

	nrf_init(NRF_RX, address);

	sei();
	while (1) {
		if (nrf_available() > 0) {
			data = nrf_receive() & 0xFF;
			switch (data) {
			case 'a':
				tgl_bit(LEDRX_PORT, LEDRX);
				nrf_send('b');
				break;
			case 'b':
				tgl_bit(LEDRX_PORT, LEDRX);
				break;
			}
		}

		_delay_ms(200);
	}

	return 0;
}

void systick(void) {
	if (farc_debounce() == ROSE) {
		tgl_bit(LEDTX_PORT, LEDTX);
		nrf_send('a');
	}
}
