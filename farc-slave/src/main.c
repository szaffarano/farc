/*
 * main.c
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#include <spi.h>
#include <nrf.h>
#include <farc.h>
#include <fsm.h>

#include <util/delay.h>
#include <avr/interrupt.h>

void systick(void);

static uint8_t relay_status(void);
static void on(void);
static void off(void);

static uint8_t address[5] = RADIO_ADDRESS;
static fsm_relay_t fsm;
int main(void) {

	farc_gpio_init();

	farc_systick_init(systick);

	spi_init();

	nrf_init(NRF_RX, address);

	fsm.off = off;
	fsm.on = on;
	fsm.status_cb = relay_status;
	fsm.timeout = TIMEOUT;

	sei();
	while (1) {
		if (nrf_available() > 0) {
			switch (nrf_receive()) {
			case CMD_STOP:
				off();
				break;
			case CMD_START:
				on();
				break;
			case CMD_STATUS:
				nrf_send(relay_status() ? RELAY_RUNNING : RELAY_IDDLE);
				break;
			}
		}
		_delay_ms(50);
	}

	return 0;
}

void systick(void) {
	fsm_status(&fsm);
}

static uint8_t relay_status(void) {
	return bit_is_set(LEDTX_PORT, LEDTX);
}

static void on(void) {
	set_bit(LEDTX_PORT, LEDTX);
}

static void off(void) {
	clear_bit(LEDTX_PORT, LEDTX);
}
