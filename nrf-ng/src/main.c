/*
 * nrf-ng.c
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#include <nrf-ng.h>
#include <stdint.h>
#include <radio/radio.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main(void) {
	nrf_ng_init();

	nrf_ng_boot_msg();

	sei();

	nrf_ng_loop();
}
