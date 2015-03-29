/*
 * nrf-ng.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef NRF_NG_H_
#define NRF_NG_H_

#include <utils.h>
#include <avr/io.h>

typedef enum _debounce_event {
	ROSE, FELL, PUSHED, RELEASED
} debounce_event;

#ifndef SLAVE
#define SLAVE
#endif

/* =========================== [application configuration] =========================== */

#define RADIO_PROGRAMMING_TIMEOUT	500

/* ============================= [port specific macros] ============================= */

#define	SW1					PD3
#define	SW1_PORT			PORTD
#define SW1_PIN				PIND

#define	LEDA				PD4
#define	LEDA_PORT			PORTD
#define LEDA_DDR			DDRD

#define	LEDB				PD5
#define	LEDB_PORT			PORTD
#define LEDB_DDR			DDRD

#define	LEDA_ON()			set_bit(LEDA_PORT, LEDA)
#define	LEDA_OFF()			clear_bit(LEDA_PORT, LEDA)
#define LEDA_TGL()			tgl_bit(LEDA_PORT, LEDA)

#define	LEDB_ON()			set_bit(LEDB_PORT, LEDB)
#define	LEDB_OFF()			clear_bit(LEDB_PORT, LEDB)
#define LEDB_TGL()			tgl_bit(LEDB_PORT, LEDB)

#define	RELAY_ON()			set_bit(LEDB_PORT, LEDB)
#define	RELAY_OFF()			clear_bit(LEDB_PORT, LEDB)
#define RELAY_TGL()			tgl_bit(LEDB_PORT, LEDB)

#define RELAY_RUNNING()		bit_is_set(LEDB_PORT, LEDB)

/* ============================= [application layer API] ============================= */

void nrf_ng_init(void);

void nrf_ng_boot_msg(void);

void nrf_ng_loop(void);

debounce_event farc_debounce(void);

uint32_t get_systicks(void);

#endif /* NRF_NG_H_ */
