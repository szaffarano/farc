/*
 * utils.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#define 	bool	_Bool
#define 	true	1
#define 	false	0

#define		mask(bit)				(1 << (bit))
#define 	tgl_bit(port, pin)		port ^= mask(pin)
#define 	set_bit(port, pin)		port |= mask(pin)
#define 	clear_bit(port, pin)	port &= ~mask(pin)

void delay_ms(uint16_t ms);

void delay_us(uint16_t us);

#endif /* UTILS_H_ */
