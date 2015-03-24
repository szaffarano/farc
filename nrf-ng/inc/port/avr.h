/*
 * nrf24l01.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef AVR_H_
#define AVR_H_

#include <avr/io.h>

#define NRF_CE			PB3
#define NRF_CE_PORT		PORTB
#define NRF_CE_PIN		PINB
#define NRF_CE_DDR		DDRB

#define NRF_IRQ			PB2
#define NRF_IRQ_PORT	PORTB
#define NRF_IRQ_PIN		PINB
#define NRF_IRQ_DDR		DDRB

#endif /* AVR_H_ */
