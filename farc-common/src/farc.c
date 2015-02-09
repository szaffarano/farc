/*
 * farc.c
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#include <farc.h>
#include <avr/interrupt.h>

typedef enum {
	WAITING_PUSH, NOISE_PUSH, PUSHING, NOISE_RELEASE
} debounce_state;

static systick_callback cb;

void farc_gpio_init(void) {
	/* pulsador como input y con pullup */
	set_bit(SW1_PORT, SW1);

	/* leds como output */
	set_bit(LEDRX_DDR, LEDRX);
	set_bit(LEDTX_DDR, LEDTX);
}

/**
 * Ejecutando a 1MHZ con prescaler de 8:
 * 	1000000 / 8 = 125000 hz = 125 ticks por ms
 *
 * @param c
 */
void farc_systick_init(systick_callback c) {
	cb = c;
	TCCR0A |= _BV(WGM01); /* modo ctc */
	TCCR0B |= _BV(CS01); /* prescaler de 8 */
	OCR0A = 125; /* cuenta hasta 125*/
	TIMSK |= _BV(OCIE0A);
	TIFR |= _BV(OCF0A);

}

debounce_event farc_debounce(void) {
	static debounce_state state = WAITING_PUSH;
	static int8_t ticks = DEBOUNCE_DELAY;

	debounce_event event = UNCHANGED;

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
				event = UNCHANGED;
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

ISR(TIMER0_COMPA_vect) {
	if (cb) {
		cb();
	}
}
