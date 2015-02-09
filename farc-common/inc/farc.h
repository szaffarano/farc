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

typedef void ((*systick_callback)(void));

typedef enum _debounce_event {
	ROSE, FELL, UNCHANGED
} debounce_event;

void farc_gpio_init(void);
void farc_systick_init(systick_callback c);
debounce_event farc_debounce(void);

#endif /* FARC_H_ */
