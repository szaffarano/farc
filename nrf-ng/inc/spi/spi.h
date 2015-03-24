/*
 * spi.h
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <spi/spi_ports.h>
#include <utils.h>

#define	spi_start_trade()	(clear_bit(SPI_SS_PORT, SPI_SS))
#define	spi_stop_trade()	(set_bit(SPI_SS_PORT, SPI_SS))

void spi_init(void);
uint8_t spi_trade_byte(uint8_t data);

#endif /* SPI_H_ */
