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

#define RF_CHANNEL 			40
#define RF_POWER_UP_DELAY 	2
#define RF_PAYLOAD_LENGTH 	1
#define RF_ADDRESS	{ 0x22, 0x33, 0x44, 0x55, 0x01 }


typedef enum {
	HAL_NRF_MAX_RT = 4, HAL_NRF_TX_DS, HAL_NRF_RX_DR
} hal_nrf_irq_source_t;

#define	RF_DATA_AVAILABLE	1
#define	RF_BUSY				2
#define	RF_MAX_RT			3

void radio_init(hal_nrf_operation_mode_t operational_mode);

uint8_t radio_get_control_register(void);

void radio_get_pload(uint8_t* buffer);

bool radio_data_available(void);

bool radio_send_packet(uint8_t *packet, uint8_t length);

void radio_irq(void);

#endif /* RADIO_H_ */
