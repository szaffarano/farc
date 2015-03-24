/*
 * nrf-ng.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#include <spi/spi.h>
#include <port/avr.h>
#include <nrf-ng.h>
#include <hal/nrf_reg.h>
#include <radio/radio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* private stuff */
static void nrf_ng_uc_init(void);
static const uint8_t address[HAL_NRF_AW_5BYTES] =
		{ 0x22, 0x33, 0x44, 0x55, 0x01 };
typedef enum {
	WAITING_PUSH, NOISE_PUSH, PUSHING, NOISE_RELEASE
} debounce_state;
#define DEBOUNCE_DELAY 20
/* end private stuff */

void nrf_ng_init(void) {
	nrf_ng_uc_init();
	spi_init();
	radio_init(address, HAL_NRF_PRX);
}

void nrf_ng_boot_msg(void) {
	LEDA_OFF();
	for (int i = 0; i < 3; i++) {
		LEDA_ON();
		_delay_ms(50);
		LEDA_OFF();
		_delay_ms(70);
	}
}

ISR(PCINT_vect) {
	radio_irq();
}

debounce_event farc_debounce(void) {
	static debounce_state state = WAITING_PUSH;
	static int8_t ticks = DEBOUNCE_DELAY;

	debounce_event event = RELEASED;

	if (ticks-- <= 0) {
		switch (state) {
		case WAITING_PUSH:
			if (bit_is_clear(SW1_PIN, SW1)) {
				state = NOISE_PUSH;
				ticks = DEBOUNCE_DELAY;
			}
			break;
		case NOISE_PUSH:
			event = FELL;
			if (bit_is_clear(SW1_PIN, SW1)) {
				state = PUSHING;
				event = FELL;
			} else {
				state = WAITING_PUSH;
			}
			break;
		case PUSHING:
			if (bit_is_set(SW1_PIN, SW1)) {
				ticks = DEBOUNCE_DELAY;
				state = NOISE_RELEASE;
			} else {
				event = PUSHED;
			}
			break;
		case NOISE_RELEASE:
			if (bit_is_clear(SW1_PIN, SW1)) {
				state = PUSHING;
			} else {
				// liberaron el boton
				state = WAITING_PUSH;

				event = ROSE;
			}
			break;
		}
	}
	return event;
}

static void nrf_ng_uc_init(void) {
	/* pulsador como input y con pullup */
	set_bit(SW1_PORT, SW1);

	/* leds como output */
	set_bit(LEDB_DDR, LEDB);
	set_bit(LEDA_DDR, LEDA);

	/* CE como output */
	set_bit(NRF_CE_DDR, NRF_CE);

	clear_bit(NRF_IRQ_DDR, NRF_IRQ);

	// PCINT para NRF_IRQ (PCINT2)
	clear_bit(NRF_IRQ_DDR, NRF_IRQ);
	GIMSK |= _BV(PCIE); /* pin change interrupt enable ... */
	PCMSK |= _BV(PCINT2); /* ... para pcint2 solamente */
}
