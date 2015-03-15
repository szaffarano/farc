/*
 * fsm.c
 *
 *  Created on: 23/02/2015
 *      Author: sebas
 */

#include <fsm-master.h>
#include <farc.h>
#include <nrf.h>

void fsm_master() {
	static states_fsm_master state = WAITING_ON;
	static int16_t timeout;

	uint8_t status = UNDEFINED;

	debounce_event e = farc_debounce();
	if (nrf_available() > 0) {
		status = nrf_receive();
	} else {
		status = UNDEFINED;
	}

	switch (state) {
	case WAITING_ON:
		if (e == FELL) {
			nrf_send(CMD_START);
			state = POLLING;
			nrf_send(CMD_STATUS);
			timeout = 0;
		}
		break;
	case POLLING:
		if (status == RELAY_RUNNING) {
			set_bit(LEDTX_PORT, LEDTX);
			nrf_send(CMD_STATUS);
			timeout = 0;
		} else if (status == RELAY_IDDLE) {
			state = WAITING_ON;
			clear_bit(LEDTX_PORT, LEDTX);
			timeout = 0;
		} else if (status == UNDEFINED) {
			// Timeout: 1 segundo sin respuestas.
			// Enciendo LED de errores.
			if (timeout++ >= 1000) {
				set_bit(LEDRX_PORT, LEDRX);
			}
		}
		if (e == FELL) {
			nrf_send(CMD_STOP);
			state = WAITING_OFF;
			timeout = 0;
			nrf_send(CMD_STATUS);
		}
		break;
	case WAITING_OFF:
		if (status == RELAY_RUNNING) {
			set_bit(LEDTX_PORT, LEDTX);
		} else if (status == RELAY_IDDLE) {
			state = WAITING_ON;
			clear_bit(LEDTX_PORT, LEDTX);
		} else if (timeout++ >= 2000) {
			// Timeout, no me respondió el slave.
			// Asumo apagado y enciendo LED de errores.
			set_bit(LEDRX_PORT, LEDRX);
			clear_bit(LEDTX_PORT, LEDTX);
			state = WAITING_ON;
		}
		break;
	}
}
