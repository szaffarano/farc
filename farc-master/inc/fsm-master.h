/*
 * fsm.h
 *
 *  Created on: 23/02/2015
 *      Author: sebas
 */

#ifndef FSM_H_
#define FSM_H_

#include <stdint.h>

typedef enum _states_fsm_master {
	WAITING_ON, POLLING, WAITING_OFF
} states_fsm_master;

void fsm_master();

#endif /* FSM_H_ */
