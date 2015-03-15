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

#define	START	0x10
#define	STOP	0x20
#define STATUS	0x30

#define	ON		0x01
#define	OFF		0x02

#define TIMEOUT	(1000 * 30)

void systick(void);

static uint8_t address[5] = { 0x22, 0x33, 0x44, 0x55, 0x01 };
uint8_t running = 0;
int main(void) {
	farc_gpio_init();

	farc_systick_init(systick);

	spi_init();

	nrf_init(NRF_RX, address);

	sei();
	while (1) {
		_delay_ms(50);
		if (nrf_available() > 0) {
			switch (nrf_receive()) {
			case ON:
				running = 1;
				break;
			case OFF:
				running = 0;
				break;
			}
		}
		if (running) {
			set_bit(LEDTX_PORT, LEDTX);
		} else {
			clear_bit(LEDTX_PORT, LEDTX);
		}
	}

	return 0;
}

void systick(void) {
	if (farc_debounce() == ROSE) {
		nrf_send(running ? STOP : START);
	}
}
