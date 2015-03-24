/*
 * nrf24l01.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <util/delay.h>
#include <spi/spi.h>
#include <avr/io.h>
#include <utils.h>

#define NRF_CE			PB3
#define NRF_CE_PORT		PORTB
#define NRF_CE_PIN		PINB
#define NRF_CE_DDR		DDRB

#define NRF_IRQ			PB2
#define NRF_IRQ_PORT	PORTB
#define NRF_IRQ_PIN		PINB
#define NRF_IRQ_DDR		DDRB

#define CSN_LOW() 		do { spi_start_trade();					} while(0)
#define CSN_HIGH()		do { spi_stop_trade();					} while(0)

#define CE_LOW()		do { clear_bit(NRF_CE_PORT, NRF_CE);	} while(0)
#define CE_HIGH()		do { set_bit(NRF_CE_PORT, NRF_CE);		} while(0)
#define CE_PULSE() 		do {\
		CE_HIGH(); 			\
		_delay_us(20); 		\
		CE_LOW(); 			\
} while(0)

#define	RADIO_ACTIVITY()		(bit_is_set(NRF_IRQ_PORT, NRF_IRQ))
#define	RESET_RADIO_ACTIVITY()	(clear_bit(NRF_IRQ_PORT, NRF_IRQ))

#endif /* NRF24L01_H_ */
