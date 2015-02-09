/*
 * spi.c
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#include <spi.h>
#include <spi_ports.h>

void spi_init() {
	SPI_SS_DDR |= (1 << SPI_SS); /* SS como output */

	SPI_SCK_DDR |= (1 << SPI_SCK); /* SCK como output */

	SPI_DO_DDR |= (1 << SPI_DO); /* DO como output */

	SPI_DI_DDR &= ~(1 << SPI_DI); /* DI como input */
	SPI_DI_PORT |= (1 << SPI_DI); /* habilitar pullup en DI */

	spi_stop_trade();
}

uint8_t spi_trade_byte(uint8_t data) {
	USIDR = data; /* dato a transferir */

	/* limpiar el USI counter overflow flag  y resetear el contador */
	USISR = (1 << USIOIF);

	while (!(USISR & (1 << USIOIF))) {
		/* setear en modo 3-wire, positive edge, incrementa USITC y clockea USCK */
		USICR = (1 << USIWM0) | (1 << USICS1) | (1 << USICLK) | (1 << USITC);
	}

	data = USIDR;

	return data;
}
