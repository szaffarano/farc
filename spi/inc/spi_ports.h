/*
 * lcd_ports.h
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#ifndef LCD_PORTS_H_
#define LCD_PORTS_H_

#include <avr/io.h>

#if defined(__AVR_ATtiny2313__)

#define SPI_SCK			PB7
#define SPI_SCK_PORT	PORTB
#define SPI_SCK_PIN		PINB
#define SPI_SCK_DDR		DDRB

#define SPI_DO			PB6
#define SPI_DO_PORT		PORTB
#define SPI_DO_PIN		PINB
#define SPI_DO_DDR		DDRB

#define SPI_DI			PB5
#define SPI_DI_PORT		PORTB
#define SPI_DI_PIN		PINB
#define SPI_DI_DDR		DDRB

#define SPI_SS			PB4
#define SPI_SS_PORT		PORTB
#define SPI_SS_PIN		PINB
#define SPI_SS_DDR		DDRB

#else

#error "Unsupported device!"

#endif

#endif /* LCD_PORTS_H_ */
