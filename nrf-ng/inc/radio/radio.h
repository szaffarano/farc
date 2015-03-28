/*
 * radio.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <stdint.h>
#include <hal/nrf_reg.h>
#include <utils.h>

/* =============================== [RF configuration] =============================== */
#define RF_CHANNEL 			40
#define RF_POWER_UP_DELAY 	2
#define RF_PAYLOAD_LENGTH 	5
#define RF_ADDRESS			{ 0x22, 0x33, 0x44, 0x55, 0x01 }

/* ================================ [RF status bits] ================================ */
typedef struct {
	uint8_t data_available :1;
	uint8_t busy :1;
	uint8_t max_rt :1;
	uint8_t undefined :5;
} rf_status_t;

/* ================================ [API functions] ================================ */

void radio_init(hal_nrf_operation_mode_t operational_mode);

rf_status_t radio_get_status(void);

bool radio_data_available(void);

void radio_get_packet(uint8_t* buffer);

bool radio_send_packet(uint8_t *packet, uint8_t length);

void radio_irq(void);

#endif /* RADIO_H_ */
