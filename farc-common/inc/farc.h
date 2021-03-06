/*
 * farc.h
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#ifndef FARC_H_
#define FARC_H_

#include <avr/io.h>

#define tgl_bit(port, pin)	port ^= _BV(pin)
#define set_bit(port, pin)	port |= _BV(pin)
#define clear_bit(port, pin)	port &= ~_BV(pin)

#define RADIO_ADDRESS		{ 0x22, 0x33, 0x44, 0x55, 0x01 }

#define	SW1					PD3
#define	SW1_PORT			PORTD
#define SW1_PIN				PIND

#define	LEDTX				PD4
#define	LEDTX_PORT			PORTD
#define LEDTX_DDR			DDRD

#define	LEDRX				PD5
#define	LEDRX_PORT			PORTD
#define LEDRX_DDR			DDRD

#define	DEBOUNCE_DELAY		20

/* relay commands */
#define	CMD_START			0x10
#define	CMD_STOP			0x20
#define CMD_STATUS			0x30

/* relay status */
#define	RELAY_RUNNING		0x01
#define	RELAY_IDDLE			0x02
#define UNDEFINED			0xFF

/* relay timeout */
#define TIMEOUT				(1000 * 30)

typedef void ((*systick_callback)(void));

typedef enum _debounce_event {
	ROSE, FELL, UNCHANGED
} debounce_event;

void farc_gpio_init(void);
void farc_systick_init(systick_callback c);
debounce_event farc_debounce(void);

#endif /* FARC_H_ */
