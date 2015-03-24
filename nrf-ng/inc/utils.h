/*
 * utils.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef UTILS_H_
#define UTILS_H_

#define 	bool	_Bool
#define 	true	1
#define 	false	0

#define tgl_bit(port, pin)		port ^= _BV(pin)
#define set_bit(port, pin)		port |= _BV(pin)
#define clear_bit(port, pin)	port &= ~_BV(pin)

#define BIT_0 0x01
#define BIT_1 0x02
#define BIT_2 0x04
#define BIT_3 0x08
#define BIT_4 0x10
#define BIT_5 0x20
#define BIT_6 0x40
#define BIT_7 0x80

#endif /* UTILS_H_ */
