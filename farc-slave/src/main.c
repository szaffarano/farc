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

#define TIMEOUT	2000

void systick(void);

static uint8_t address[5] = { 0x22, 0x33, 0x44, 0x55, 0x01 };
static uint8_t running;
static int16_t start_ms = 0;

int main(void) {

	farc_gpio_init();

	farc_systick_init(systick);

	spi_init();

	nrf_init(NRF_RX, address);

	sei();
	while (1) {
		if (nrf_available() > 0) {
			switch (nrf_receive()) {
			case STOP:
				running = 0;
				break;
			case START:
				if (!running) {
					start_ms = 0;
					set_bit(LEDTX_PORT, LEDRX);
				}
				running = 1;
				break;
			case STATUS:
				nrf_send(running ? ON : OFF);
				break;
			}
		}

		if (running) {
			if (start_ms >= TIMEOUT) {
				running = 0;
			}
		} else {
			clear_bit(LEDRX_PORT, LEDRX);
		}
		nrf_send(running ? ON : OFF);
		_delay_ms(50);
	}

	return 0;
}

void systick(void) {
	start_ms++;
}
