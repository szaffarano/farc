/*
 * debounce.c
 *
 *  Created on: 24/03/2015
 *      Author: sebas
 */

#include <debounce/debounce.h>
#include <nrf-ng.h>

typedef enum {
	WAITING_PUSH, NOISE_PUSH, PUSHING, NOISE_RELEASE
} debounce_state;

debounce_event farc_debounce(void) {
	static debounce_state state = WAITING_PUSH;
	static int8_t ticks = DEBOUNCE_DELAY;

	static debounce_event event = RELEASED;

	if (ticks-- <= 0) {
		switch (state) {
		case WAITING_PUSH:
			if (bit_is_clear(SW1_PIN, SW1)) {
				state = NOISE_PUSH;
				ticks = DEBOUNCE_DELAY;
			}
			event = RELEASED;
			break;
		case NOISE_PUSH:
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
			}
			event = PUSHED;
			break;
		case NOISE_RELEASE:
			if (bit_is_clear(SW1_PIN, SW1)) {
				state = PUSHING;
				event = PUSHED;
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
