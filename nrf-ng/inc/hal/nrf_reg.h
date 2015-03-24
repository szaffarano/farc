/*
 * nrf_reg.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef NRF_REG_H_
#define NRF_REG_H_

typedef enum {
	HAL_NRF_CRC_OFF, HAL_NRF_CRC_8BIT = 2, HAL_NRF_CRC_16BIT
} hal_nrf_crc_mode_t;

typedef enum {
	HAL_NRF_PIPE0,
	HAL_NRF_PIPE1,
	HAL_NRF_PIPE2,
	HAL_NRF_PIPE3,
	HAL_NRF_PIPE4,
	HAL_NRF_PIPE5,
	HAL_NRF_TX,
	HAL_NRF_ALL = 0xFF
} hal_nrf_address_t;

typedef enum {
	HAL_NRF_AW_3BYTES = 3, HAL_NRF_AW_4BYTES, HAL_NRF_AW_5BYTES
} hal_nrf_address_width_t;

typedef enum {
	HAL_NRF_PTX, HAL_NRF_PRX
} hal_nrf_operation_mode_t;

typedef enum {
	HAL_NRF_PWR_DOWN, HAL_NRF_PWR_UP
} hal_nrf_pwr_mode_t;

typedef enum {
	HAL_NRF_TX_PLOAD = 7, HAL_NRF_RX_PLOAD, HAL_NRF_ACK_PLOAD
} hal_nrf_pload_command_t;

#define CONFIG				0x00
#define EN_AA				0x01
#define EN_RXADDR			0x02
#define SETUP_AW			0x03
#define SETUP_RETR			0x04
#define RF_CH				0x05
#define RF_SETUP			0x06
#define STATUS				0x07
#define OBSERVE_TX			0x08
#define CD					0x09
#define RX_ADDR_P0			0x0A
#define RX_ADDR_P1			0x0B
#define RX_ADDR_P2			0x0C
#define RX_ADDR_P3			0x0D
#define RX_ADDR_P4			0x0E
#define RX_ADDR_P5			0x0F
#define TX_ADDR				0x10
#define RX_PW_P0			0x11
#define RX_PW_P1			0x12
#define RX_PW_P2			0x13
#define RX_PW_P3			0x14
#define RX_PW_P4			0x15
#define RX_PW_P5			0x16
#define FIFO_STATUS			0x17
#define DYNPD				0x1C
#define FEATURE				0x1D

#define WRITE_REG			0x20
#define RD_RX_PLOAD_W		0x60
#define RD_RX_PLOAD   		0x61
#define WR_TX_PLOAD   		0xA0
#define WR_ACK_PLOAD  		0xA8
#define WR_NAC_TX_PLOAD		0xB0
#define FLUSH_TX			0xE1
#define FLUSH_RX			0xE2
#define REUSE_TX_PL			0xE3
#define LOCK_UNLOCK			0x50
#define NOP					0xFF

#define MASK_RX_DR			6
#define MASK_TX_DS			5
#define MASK_MAX_RT			4
#define EN_CRC				3
#define CRCO				2
#define PWR_UP				1
#define PRIM_RX				0

#define ALL_PIPES			(0x3F)

#endif /* NRF_REG_H_ */
