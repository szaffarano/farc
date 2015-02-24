/*
 * fsm.c
 *
 *  Created on: 23/2/2015
 *      Author: sebas
 */

#include <fsm.h>

void fsm_status(fsm_relay_t* relay) {
	switch (relay->state) {
	case IDLE:
		if (relay->status_cb()) {
			relay->start = 0;
			relay->state = RUNNING;
			relay->on();
		}
		break;
	case RUNNING:
		if (relay->start++ >= relay->timeout) {
			relay->state = IDLE;
			relay->off();
		}
		break;
	default:
		break;
	}
}
