/*
 * nrf.h
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#ifndef NRF_H_
#define NRF_H_

#include <stdint.h>

typedef enum {
	NRF_TX, NRF_RX
} nrf_mode_t;

/** @brief
 * Inicializa el dispositivo  nrf24l01+
 */
void nrf_init(nrf_mode_t mode, uint8_t address[5]);

/** @brief
 * Envía un byte a la dirección que se le configuró al dispositivo.
 */
void nrf_send(uint8_t c);

/** @brief
 * Devuelve un word con:
 *      LSB: byte recibido por el dispositivo
 *      MSB: código de error en el MSB.
 */
uint8_t nrf_receive();

/** @brief
 * Devuelve la cantidad de bytes disponibles para lectura.
 */
uint8_t nrf_available();

#endif /* NRF_H_ */
