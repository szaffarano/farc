/*
 * spi.h
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <spi_ports.h>

#define	spi_start_trade()	SPI_SS_PORT &= ~(1 << SPI_SS)
#define	spi_stop_trade()	SPI_SS_PORT |= (1 << SPI_SS)

void spi_init();
uint8_t spi_trade_byte(uint8_t data);

#endif /* SPI_H_ */
