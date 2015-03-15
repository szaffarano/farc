/*
 * fsm.h
 *
 *  Created on: 23/2/2015
 *      Author: sebas
 */

#ifndef FSM_H_
#define FSM_H_

#include <stdint.h>

typedef uint8_t (*relay_status_cb)(void);
typedef void (*drive_relay_cb)(void);

typedef enum _relay_state {
	IDLE, RUNNING
} relay_state;

typedef struct _fsm_relay_t {
	relay_state state;
	uint16_t start;
	uint16_t timeout;
	relay_status_cb status_cb;
	drive_relay_cb on;
	drive_relay_cb off;
} fsm_relay_t;

void fsm_status(fsm_relay_t* relay);

#endif /* FSM_H_ */
