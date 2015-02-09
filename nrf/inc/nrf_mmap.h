/*
 * nrf_mmap.h
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#ifndef NRF_MMAP_H_
#define NRF_MMAP_H_

#define CONFIG			0x00
#define EN_AA			0x01
#define EN_RXADDR		0x02
#define SETUP_AW		0x03
#define SETUP_RETR		0x04
#define RF_CH			0x05
#define RF_SETUP		0x06
#define STATUS			0x07
#define OBSERVE_TX		0x08
#define RPD				0x09
#define RX_ADDR_P0		0x0A
#define RX_ADDR_P1		0x0B
#define RX_ADDR_P2		0x0C
#define RX_ADDR_P3		0x0D
#define RX_ADDR_P4		0x0E
#define RX_ADDR_P5		0x0F
#define TX_ADDR			0x10
#define RX_PW_P0		0x11
#define RX_PW_P1		0x12
#define RX_PW_P2		0x13
#define RX_PW_P3		0x14
#define RX_PW_P4		0x15
#define RX_PW_P5		0x16
#define FIFO_STATUS		0x17
#define DYNPD			0x1C
#define FEATURE			0x1D

/* bits posicionales dentro de los registros en memoria */
// CONFIG register
#define MASK_RX_DR		6
#define MASK_TX_DS		5
#define MASK_MAX_RT		4
#define EN_CRC			3
#define CRCO			2
#define PWR_UP			1
#define PRIM_RX			0

// STATUS register
#define RX_DR			6
#define TX_DS			5
#define MAX_RT			4
#define	RX_P_NO			1
#define	TX_FULL			0
#define	MASK_RX_P_NO	0x07

#define	NRF_IRQS_MASK	(_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT))

// EN_AA register
#define ENAA_P5			5
#define ENAA_P4			4
#define ENAA_P3			3
#define ENAA_P2			2
#define ENAA_P1			1
#define ENAA_P0			0

// EN_RXADDR register
#define ENRX_P5			5
#define ENRX_P4			4
#define ENRX_P3			3
#define ENRX_P2			2
#define ENRX_P1			1
#define ENRX_P0			0

// DYNPD register
#define	DPL_P5			5
#define	DPL_P4			4
#define	DPL_P3			3
#define	DPL_P2			2
#define	DPL_P1			1
#define	DPL_P0			0

// FEATURE register
#define	EN_DPL			2
#define	EN_ACK_PAY		1
#define	EN_DYN_ACK		0

#endif /* NRF_MMAP_H_ */
