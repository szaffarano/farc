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
#include <hal/nrf.h>
#include <avr/eeprom.h>
#include <util/atomic.h>

#define CMD_START		0x10
#define CMD_STOP		0x20
#define CMD_STATUS		0x30
#define CMD_UNDEFINED	0xFF

#define	RUNNING			0xC4
#define IDLE			0x57

static uint8_t pload[RF_PAYLOAD_LENGTH];
volatile static uint32_t ticks;

static uint32_t EEMEM ee_relay_timeout;
static uint32_t relay_timeout;

void fsm_master(void);
void fsm_slave(void);

typedef enum {
	MASTER_IDLE, MASTER_WAITING_START, MASTER_RUNNING
} fsm_master_state;

typedef enum {
	SLAVE_IDLE, SLAVE_RUNNING
} fsm_slave_state;

static void nrf_ng_uc_init(void);
static void nrf_ng_systick_init(void);

void nrf_ng_init(void) {
	nrf_ng_uc_init();
	nrf_ng_systick_init();
	spi_init();

	relay_timeout = eeprom_read_dword(&ee_relay_timeout);
#if defined (MASTER)
	radio_init(HAL_NRF_PTX);
#elif defined (SLAVE)
	radio_init(HAL_NRF_PRX);
#endif
}

void nrf_ng_boot_msg(void) {

#if defined (MASTER)
	uint8_t loop = 5;
#elif defined (SLAVE)
	uint8_t loop = 3;
#endif

	LEDA_OFF();
	for (; loop > 0; loop--) {
		LEDA_ON();
		delay_ms(50);
		LEDA_OFF();
		delay_ms(70);
	}
}

void nrf_ng_loop() {
	while (true) {

#if defined (MASTER)
		fsm_master();
#elif defined (SLAVE)
		fsm_slave();
#endif
		delay_ms(1);
	}
}

uint32_t get_systicks(void) {
	return ticks;
}

ISR(PCINT_vect) {
	radio_irq();
}

ISR(TIMER0_COMPA_vect) {
	ticks++;
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

static void nrf_ng_systick_init(void) {
	TCCR0A |= _BV(WGM01); /* modo ctc */
	TCCR0B |= _BV(CS01); /* prescaler de 8 */
	OCR0A = 125; /* cuenta hasta 125*/
	TIMSK |= _BV(OCIE0A);
	TIFR |= _BV(OCF0A);
}

void fsm_master(void) {
	static fsm_master_state state = MASTER_IDLE;
	radio_status_t status = radio_get_status();
	debounce_event event = farc_debounce();
	uint8_t ack = CMD_UNDEFINED;

	switch (status) {
	case RF_TX_AP:
		ack = radio_get_pload_byte(0);
		radio_set_status(RF_IDLE);
		break;
	case RF_TX_DS:
		radio_set_status(RF_IDLE);
		break;
	case RF_MAX_RT:
		LEDB_ON();
		radio_set_status(RF_IDLE);
		break;
	default:
		break;
	}

	switch (state) {
	case MASTER_IDLE:
		if (event == FELL) {
			pload[0] = CMD_START;
			if (radio_send_packet(pload, RF_PAYLOAD_LENGTH) > 0) {
				state = MASTER_WAITING_START;
				_delay_ms(2);
			}
		}
		break;
	case MASTER_WAITING_START:
		if (ack == RUNNING) {
			LEDA_ON();
			state = MASTER_RUNNING;
		} else {
			pload[0] = CMD_STATUS;
			radio_send_packet(pload, RF_PAYLOAD_LENGTH);
		}
		break;
	case MASTER_RUNNING:
		if (event == FELL) {
			pload[0] = CMD_STOP;
			radio_send_packet(pload, RF_PAYLOAD_LENGTH);
		} else if (ack == IDLE) {
			state = MASTER_IDLE;
			LEDA_OFF();
		} else if (ack == RUNNING) {
			LEDA_ON();
		} else {
			pload[0] = CMD_STATUS;
			radio_send_packet(pload, RF_PAYLOAD_LENGTH);
		}
		break;
	}
}

void fsm_slave(void) {
	static fsm_slave_state state = SLAVE_IDLE;
	static uint32_t start;
	static uint32_t enlapsed;

	radio_status_t status = radio_get_status();
	uint8_t cmd = CMD_UNDEFINED;

	switch (status) {
	case RF_TX_AP:
	case RF_RX_DR:
		pload[0] = RELAY_RUNNING() ? RUNNING : IDLE;
		cmd = radio_get_pload_byte(0);
		hal_nrf_write_ack_pload(0, pload, RF_PAYLOAD_LENGTH);
		radio_set_status(RF_IDLE);
		break;
	case RF_MAX_RT:
		LEDA_ON();
		radio_set_status(RF_IDLE);
		break;
	default:
		break;
	}

	switch (state) {
	case SLAVE_IDLE:
		if (cmd == CMD_START) {
			RELAY_ON();
			LEDA_ON();
			state = SLAVE_RUNNING;
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				start = get_systicks();
			}
		}
		break;
	case SLAVE_RUNNING:
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			enlapsed = get_systicks() - start;
		}
		if (cmd == CMD_STOP || enlapsed > relay_timeout) {
			RELAY_OFF();
			LEDA_OFF();
			state = SLAVE_IDLE;
		}
		break;
	}
}
