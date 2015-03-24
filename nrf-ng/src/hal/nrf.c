/*
 * nrf.c
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#include <hal/nrf.h>
#include <spi/spi.h>
#include <hw/nrf24l01.h>

void hal_nrf_enable_ack_pl(void) {
	hal_nrf_write_reg(FEATURE, (hal_nrf_read_reg(FEATURE) | 0x02));
}

void hal_nrf_disable_ack_pl(void) {
	hal_nrf_write_reg(FEATURE, (hal_nrf_read_reg(FEATURE) & ~0x02));
}

void hal_nrf_enable_dynamic_pl(void) {
	hal_nrf_write_reg(FEATURE, (hal_nrf_read_reg(FEATURE) | 0x04));
}

void hal_nrf_disable_dynamic_pl(void) {
	hal_nrf_write_reg(FEATURE, (hal_nrf_read_reg(FEATURE) & ~0x04));
}

void hal_nrf_setup_dyn_pl(uint8_t setup) {
	hal_nrf_write_reg(DYNPD, setup & ~0xC0);
}

uint8_t hal_nrf_read_rx_pl_w(void) {

	uint8_t temp;

	csn_low();

	hal_nrf_rw(RD_RX_PLOAD_W);
	temp = hal_nrf_rw(0);
	csn_high();

	return temp;

}

void hal_nrf_lock_unlock() {
	csn_low();

	hal_nrf_rw(LOCK_UNLOCK);
	hal_nrf_rw(0x73);

	csn_high();
}

void hal_nrf_write_ack_pload(uint8_t pipe, uint8_t *tx_pload, uint8_t length) {
	csn_low();

	hal_nrf_rw(WR_ACK_PLOAD | pipe);
	while (length--) {
		hal_nrf_rw(*tx_pload++);
	}

	csn_high();
}

uint8_t hal_nrf_get_clear_irq_flags(void) {
	return hal_nrf_write_reg(STATUS, (mask(6) | mask(5) | mask(4)))
			& (mask(6) | mask(5) | mask(4));
}

void hal_nrf_set_crc_mode(hal_nrf_crc_mode_t crc_mode) {
	hal_nrf_write_reg(CONFIG,
			(hal_nrf_read_reg(CONFIG) & ~(mask(3) | mask(2)))
					| ((uint8_t) (crc_mode) << 2));
}

void hal_nrf_open_pipe(hal_nrf_address_t pipe_num, bool auto_ack) {

	switch (pipe_num) {
	case HAL_NRF_PIPE0:
	case HAL_NRF_PIPE1:
	case HAL_NRF_PIPE2:
	case HAL_NRF_PIPE3:
	case HAL_NRF_PIPE4:
	case HAL_NRF_PIPE5:
		hal_nrf_write_reg(EN_RXADDR,
				hal_nrf_read_reg(EN_RXADDR) | _BV(pipe_num));

		if (auto_ack)
			hal_nrf_write_reg(EN_AA, hal_nrf_read_reg(EN_AA) | _BV(pipe_num));
		else
			hal_nrf_write_reg(EN_AA, hal_nrf_read_reg(EN_AA) & ~_BV(pipe_num));
		break;

	case HAL_NRF_ALL:
		// sebas: casteo
		hal_nrf_write_reg(EN_RXADDR, (uint8_t) ~(mask(7) | mask(6)));

		if (auto_ack)
			hal_nrf_write_reg(EN_AA, (uint8_t) ~(mask(7) | mask(6)));
		else
			hal_nrf_write_reg(EN_AA, 0);
		break;

	default:
		break;
	}

}

void hal_nrf_close_pipe(hal_nrf_address_t pipe_num) {
	switch (pipe_num) {
	case HAL_NRF_PIPE0:
	case HAL_NRF_PIPE1:
	case HAL_NRF_PIPE2:
	case HAL_NRF_PIPE3:
	case HAL_NRF_PIPE4:
	case HAL_NRF_PIPE5:
		hal_nrf_write_reg(EN_RXADDR,
				hal_nrf_read_reg(EN_RXADDR) & ~_BV(pipe_num));
		hal_nrf_write_reg(EN_AA, hal_nrf_read_reg(EN_AA) & ~_BV(pipe_num));
		break;

	case HAL_NRF_ALL:
		hal_nrf_write_reg(EN_RXADDR, 0);
		hal_nrf_write_reg(EN_AA, 0);
		break;

	default:
		break;
	}
}

void hal_nrf_set_address(hal_nrf_address_t address, uint8_t *addr) {
	switch (address) {
	case HAL_NRF_TX:
	case HAL_NRF_PIPE0:
	case HAL_NRF_PIPE1:
		hal_nrf_write_multibyte_reg((uint8_t) address, addr, 0);
		break;

	case HAL_NRF_PIPE2:
	case HAL_NRF_PIPE3:
	case HAL_NRF_PIPE4:
	case HAL_NRF_PIPE5:
		hal_nrf_write_reg(RX_ADDR_P0 + (uint8_t) address, *addr);
		break;

	default:
		break;
	}

}

void hal_nrf_set_auto_retr(uint8_t retr, uint16_t delay) {
	hal_nrf_write_reg(SETUP_RETR, (((delay / 250) - 1) << 4) | retr);
}

void hal_nrf_set_address_width(hal_nrf_address_width_t address_width) {
	hal_nrf_write_reg(SETUP_AW, ((uint8_t) (address_width) - 2));
}

void hal_nrf_set_rx_pload_width(uint8_t pipe_num, uint8_t pload_width) {
	hal_nrf_write_reg(RX_PW_P0 + pipe_num, pload_width);
}

uint8_t hal_nrf_get_address_width(void) {
	return (hal_nrf_read_reg(SETUP_AW) + 2);
}

void hal_nrf_set_operation_mode(hal_nrf_operation_mode_t op_mode) {
	if (op_mode == HAL_NRF_PRX) {
		hal_nrf_write_reg(CONFIG, (hal_nrf_read_reg(CONFIG) | mask(PRIM_RX)));
		ce_high();
	} else {
		hal_nrf_write_reg(CONFIG, (hal_nrf_read_reg(CONFIG) & ~mask(PRIM_RX)));
		ce_low();
	}
}

hal_nrf_operation_mode_t hal_nrf_get_operation_mode(void) {
	return (hal_nrf_read_reg(CONFIG) & _BV(PRIM_RX)) ? HAL_NRF_PRX : HAL_NRF_PTX;
}

void hal_nrf_set_power_mode(hal_nrf_pwr_mode_t pwr_mode) {
	if (pwr_mode == HAL_NRF_PWR_UP) {
		hal_nrf_write_reg(CONFIG, (hal_nrf_read_reg(CONFIG) | mask(PWR_UP)));
	} else {
		hal_nrf_write_reg(CONFIG, (hal_nrf_read_reg(CONFIG) & ~mask(PWR_UP)));
	}
}

void hal_nrf_set_rf_channel(uint8_t channel) {
	hal_nrf_write_reg(RF_CH, channel);
}

bool hal_nrf_rx_fifo_empty(void) {
	if (hal_nrf_get_rx_data_source() == 7) {
		return true;
	} else {
		return false;
	}
}

uint8_t hal_nrf_get_rx_data_source(void) {
	return ((hal_nrf_nop() & (mask(3) | mask(2) | mask(1))) >> 1);
}

uint16_t hal_nrf_read_rx_pload(uint8_t *rx_pload) {
	return hal_nrf_read_multibyte_reg((uint8_t) (HAL_NRF_RX_PLOAD), rx_pload);
}

void hal_nrf_write_tx_pload(uint8_t *tx_pload, uint8_t length) {
	hal_nrf_write_multibyte_reg((uint8_t) (HAL_NRF_TX_PLOAD), tx_pload, length);
	ce_pulse();
}

void hal_nrf_flush_tx(void) {
	hal_nrf_write_reg(FLUSH_TX, 0);
}

uint8_t hal_nrf_nop(void) {
	return hal_nrf_write_reg(NOP, 0);
}

uint8_t hal_nrf_read_reg(uint8_t reg) {
	uint8_t temp;
	csn_low();
	hal_nrf_rw(reg);
	temp = hal_nrf_rw(0);
	csn_high();

	return temp;
}

uint8_t hal_nrf_write_reg(uint8_t reg, uint8_t value) {
	uint8_t retval;
	csn_low();
	if (reg < WRITE_REG) {
		retval = hal_nrf_rw(WRITE_REG + reg);
		hal_nrf_rw(value);
	} else {
		if (!(reg == FLUSH_TX) && !(reg == FLUSH_RX) && !(reg == REUSE_TX_PL)
				&& !(reg == NOP)) {
			retval = hal_nrf_rw(reg);
			hal_nrf_rw(value);
		} else {
			retval = hal_nrf_rw(reg);
		}
	}
	csn_high();

	return retval;
}

uint16_t hal_nrf_read_multibyte_reg(uint8_t reg, uint8_t *pbuf) {
	uint8_t ctr, length;
	switch (reg) {
	case HAL_NRF_PIPE0:
	case HAL_NRF_PIPE1:
	case HAL_NRF_TX:
		length = ctr = hal_nrf_get_address_width();
		csn_low();
		hal_nrf_rw(RX_ADDR_P0 + reg);
		break;

	case HAL_NRF_RX_PLOAD:
		if ((reg = hal_nrf_get_rx_data_source()) < 7) {
			length = ctr = hal_nrf_read_rx_pl_w();

			csn_low();
			hal_nrf_rw(RD_RX_PLOAD);
		} else {
			ctr = length = 0;
		}
		break;

	default:
		ctr = length = 0;
		break;
	}

	while (ctr--) {
		*pbuf++ = hal_nrf_rw(0);
	}

	csn_high();

	return (((uint16_t) reg << 8) | length);
}

void hal_nrf_write_multibyte_reg(uint8_t reg, uint8_t *pbuf, uint8_t length) {
	switch (reg) {
	case HAL_NRF_PIPE0:
	case HAL_NRF_PIPE1:
	case HAL_NRF_TX:
		length = hal_nrf_get_address_width();
		csn_low();
		hal_nrf_rw(WRITE_REG + RX_ADDR_P0 + reg);
		break;

	case HAL_NRF_TX_PLOAD:
		csn_low();
		hal_nrf_rw(WR_TX_PLOAD);
		break;
	default:
		break;
	}

	while (length--) {
		hal_nrf_rw(*pbuf++);
	}

	csn_high();
}

uint8_t hal_nrf_rw(uint8_t value) {
	return spi_trade_byte(value);
}

