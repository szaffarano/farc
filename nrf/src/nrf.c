/*
 * nrf.c
 *
 *  Created on: 05/02/2015
 *      Author: sebas
 */

#include <nrf.h>
#include <spi.h>
#include <nrf_commands.h>
#include <nrf_mmap.h>
#include <nrf_ports.h>
#include <farc.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#define	ce_low()			clear_bit(NRF_CE_PORT, NRF_CE)
#define	ce_high()			set_bit(NRF_CE_PORT, NRF_CE)

/* #### manejo de cola circular para los datos recibidos #### */
#define NRF_BUFFER_SIZE			8
#define NRF_BUFFER_MASK			(NRF_BUFFER_SIZE - 1)
#define	nrf_next_id(idx)		((idx+1) & NRF_BUFFER_MASK)
typedef struct _buffer_t {
	volatile unsigned char buffer[NRF_BUFFER_SIZE];
	volatile unsigned char push_idx;
	volatile unsigned char pop_idx;
} buffer_t;
static buffer_t rx = { .push_idx = 0, .pop_idx = 0 };
static void nrf_push(uint8_t data);
/* ##### manejo de cola circular para los datos recibidos #### */

/* encabezado de funciones helper */

static uint8_t nrf_read_reg(uint8_t reg);
//static void nrf_read_multibyte_reg(uint8_t reg, uint8_t* buffer);
static uint8_t nrf_write_reg(uint8_t reg, uint8_t value);
static uint8_t nrf_write_multibyte_reg(uint8_t reg, uint8_t *pbuf,
		uint8_t length);
static void nrf_set_mode(nrf_mode_t mode);
static void nrf_ce_pulse(void);
static uint8_t nrf_rx_fifo_empty(void);
uint8_t nrf_execute(uint8_t cmd);

void nrf_init(nrf_mode_t mode, uint8_t address[5]) {
	// dejo bajo CE para no generar actividad
	ce_low();

	// CE como output
	set_bit(NRF_CE_DDR, NRF_CE);

	// PCINT para NRF_IRQ (PCINT2)
	clear_bit(NRF_IRQ_DDR, NRF_IRQ);
	GIMSK |= _BV(PCIE); /* pin change interrupt enable ... */
	PCMSK |= _BV(PCINT2); /* ... para pcint2 solamente */

	// cierro todos los pipes...
	nrf_write_reg(EN_RXADDR, 0);
	nrf_write_reg(EN_AA, 0);

	// ... y abro el pipe0 con autoack
	nrf_write_reg(EN_RXADDR, _BV(ENRX_P0));
	nrf_write_reg(EN_AA, _BV(ENAA_P0));

	// habilito power up, crc de 2 bytes, habilitar las máscaras de interrupción y ...
	if (mode == NRF_RX) {
		// ... modo PRX y...
		nrf_write_reg(CONFIG,
		_BV(PWR_UP) | _BV(EN_CRC) | _BV(CRCO) | _BV(PRIM_RX));
	} else {
		// ... modo PTX
		nrf_write_reg(CONFIG, (_BV(PWR_UP) | _BV(EN_CRC) | _BV(CRCO)));
	}

	// setear el tamaño del payload (en bytes) de pipe0
	// hardcoded el payload en 1 byte
	nrf_write_reg(RX_PW_P0, 1);

	// bits[7:4]: tiempo de espera para retransmitir, valores:
	// 		0000 = 250 us, 0001 = 500 us, 0010 = 750 us ... 1111 = 4000 us
	// bits[4:0]: cantidad de retransmisiones
	//
	// 0011: espera 1000 uSec para retransmitir
	// 1111: retransmite hasta 15 veces antes de dar error
	// @TODO va hardcoded, analizar recibirlo como parámetro.
	nrf_write_reg(SETUP_RETR, (0b0010 << 4) | (0b1111));

	// 11: address de 5 bytes (recibo 5 bytes configurando address -> hardcoded)
	nrf_write_reg(SETUP_AW, 0b11);

	// setear la dirección a usar para tx y rx
	nrf_write_multibyte_reg(RX_ADDR_P0, address, 5);
	nrf_write_multibyte_reg(TX_ADDR, address, 5);

	// setear canal: 2400 + rf_channel
	nrf_write_reg(RF_CH, 0x01);

	// esperar 2 ms hasta que se estabiliza el power up
	_delay_ms(2);

	// si es receptor, dejo arriba CE para comenzar a escuchar
	if (mode == NRF_RX) {
		ce_high();
	}
}

void nrf_send(uint8_t c) {
	nrf_set_mode(NRF_TX);
	nrf_write_reg(W_TX_PAYLOAD, c);
	nrf_ce_pulse();
	nrf_set_mode(NRF_RX);
}

uint8_t nrf_receive() {
	uint8_t next_idx;
	uint8_t r;

	if (rx.pop_idx == rx.push_idx) {
		return 0;
	}
	next_idx = nrf_next_id(rx.pop_idx);
	r = rx.buffer[next_idx];
	rx.pop_idx = next_idx;

	return r;
}

uint8_t nrf_available() {
	return (rx.push_idx > rx.pop_idx) ? (rx.push_idx - rx.pop_idx) : 0;
}

/* funciones helper */
static uint8_t nrf_read_reg(uint8_t reg) {
	uint8_t value;
	spi_start_trade();

	value = spi_trade_byte(R_REGISTER + reg);
	value = spi_trade_byte(NOP);

	spi_stop_trade();

	return value;
}

//static void nrf_read_multibyte_reg(uint8_t reg, uint8_t* buffer) {
//	uint8_t length = 10;
//	spi_start_trade();
//
//	spi_trade_byte(R_REGISTER + reg);
//	while (length--) {
//		*buffer++ = spi_trade_byte(NOP);
//	}
//
//	spi_stop_trade();
//}

static uint8_t nrf_write_reg(uint8_t reg, uint8_t value) {
	uint8_t status;
	spi_start_trade();
	status = spi_trade_byte(W_REGISTER | reg);
	spi_trade_byte(value);
	spi_stop_trade();

	return status;
}

static uint8_t nrf_write_multibyte_reg(uint8_t reg, uint8_t *pbuf,
		uint8_t length) {
	uint8_t status;

	spi_start_trade();
	status = spi_trade_byte(W_REGISTER | reg);
	while (length--) {
		spi_trade_byte(*pbuf++);
	}

	spi_stop_trade();

	return status;
}

void nrf_set_mode(nrf_mode_t mode) {
	uint8_t config = nrf_read_reg(CONFIG);

	if (mode == NRF_RX) {
		nrf_write_reg(CONFIG, (config | _BV(PRIM_RX)));
		ce_high();
	} else {
		ce_low();
		nrf_write_reg(CONFIG, (config & ~_BV(PRIM_RX)));
	}
}

void nrf_ce_pulse(void) {
	ce_high();
	_delay_ms(1);
	ce_low();
}

static uint8_t nrf_rx_fifo_empty(void) {
	return ((nrf_read_reg(STATUS) >> RX_P_NO) & MASK_RX_P_NO) == MASK_RX_P_NO;
}

uint8_t nrf_execute(uint8_t cmd) {
	uint8_t value;
	spi_start_trade();
	value = spi_trade_byte(cmd);
	spi_stop_trade();
	return value;
}

ISR(PCINT_vect) {
	if (bit_is_clear(NRF_IRQ_PIN, NRF_IRQ)) {
		uint8_t status = nrf_write_reg(STATUS, NRF_IRQS_MASK) & NRF_IRQS_MASK;
		switch (status) {
		case (_BV(TX_DS)): /* se envió un paquete */
			break;
		case (_BV(TX_DS) | _BV(RX_DR)): /* se envió un paquete y se recibió ack con payload */
			// leer el payload
			while (!nrf_rx_fifo_empty()) {
				//	nrf_read_multibyte_reg(R_RX_PAYLOAD, ack_payload);
				nrf_push(nrf_read_reg(R_RX_PAYLOAD));
			}
			break;
		case (_BV(RX_DR)): /* se recibió un paquete */
			// leer el payload
			while (!nrf_rx_fifo_empty()) {
				//	nrf_read_multibyte_reg(R_RX_PAYLOAD, rx_payload);
				nrf_push(nrf_read_reg(R_RX_PAYLOAD));
			}
			break;
		case (_BV(MAX_RT)): /* máxima cantidad de reintentos */
			// flush de tx
			nrf_execute(FLUSH_TX);
			break;
		};
	}
}

/* implementacion funciones manejo de cola */
static void nrf_push(uint8_t data) {
	uint8_t next_idx = nrf_next_id(rx.push_idx);

	if (next_idx == rx.pop_idx) {
		// overflow
	} else {
		rx.push_idx = next_idx;
		rx.buffer[next_idx] = data;
	}
}
