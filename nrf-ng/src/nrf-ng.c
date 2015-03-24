/*
 * nrf-ng.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#include <spi/spi.h>
#include <port/avr.h>
#include <nrf-ng.h>
#include <hal/nrf_reg.h>
#include <radio/radio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

static void nrf_ng_uc_init(void);

void nrf_ng_init(void) {
	nrf_ng_uc_init();
	spi_init();
	radio_init(HAL_NRF_PRX);
}

void nrf_ng_boot_msg(void) {
	LEDA_OFF();
	for (int i = 0; i < 3; i++) {
		LEDA_ON();
		delay_ms(50);
		LEDA_OFF();
		delay_ms(70);
	}
}

ISR(PCINT_vect) {
	radio_irq();
}

static void nrf_ng_uc_init(void) {
	/* pulsador como input y con pullup */
	set_bit(SW1_PORT, SW1);

	/* leds como output */
	set_bit(LEDB_DDR, LEDB);
	set_bit(LEDA_DDR, LEDA);

	/* CE como output */
	set_bit(NRF_CE_DDR, NRF_CE);

	clear_bit(NRF_IRQ_DDR, NRF_IRQ);

	// PCINT para NRF_IRQ (PCINT2)
	clear_bit(NRF_IRQ_DDR, NRF_IRQ);
	GIMSK |= _BV(PCIE); /* pin change interrupt enable ... */
	PCMSK |= _BV(PCINT2); /* ... para pcint2 solamente */
}
