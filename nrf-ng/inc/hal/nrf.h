/*
 * nrf.h
 *
 *  Created on: 20/03/2015
 *      Author: sebas
 */

#ifndef NRF_H_
#define NRF_H_

#include <hal/nrf_reg.h>
#include <utils.h>
#include <stdint.h>

void hal_nrf_enable_ack_pl(void);

void hal_nrf_disable_ack_pl(void);

void hal_nrf_enable_dynamic_pl(void);

void hal_nrf_disable_dynamic_pl(void);

void hal_nrf_setup_dyn_pl(uint8_t setup);

uint8_t hal_nrf_read_rx_pl_w(void);

void hal_nrf_lock_unlock(void);

void hal_nrf_write_ack_pload(uint8_t pipe, uint8_t *tx_pload, uint8_t length);

uint8_t hal_nrf_get_clear_irq_flags(void);

void hal_nrf_set_crc_mode(hal_nrf_crc_mode_t crc_mode);

void hal_nrf_open_pipe(hal_nrf_address_t pipe_num, bool auto_ack);

void hal_nrf_close_pipe(hal_nrf_address_t pipe_num);

void hal_nrf_set_address(hal_nrf_address_t address, uint8_t *addr);

void hal_nrf_set_auto_retr(uint8_t retr, uint16_t delay);

void hal_nrf_set_address_width(hal_nrf_address_width_t address_width);

void hal_nrf_set_rx_pload_width(uint8_t pipe_num, uint8_t pload_width);

uint8_t hal_nrf_get_address_width(void);

void hal_nrf_set_operation_mode(hal_nrf_operation_mode_t op_mode);

hal_nrf_operation_mode_t hal_nrf_get_operation_mode(void);

void hal_nrf_set_power_mode(hal_nrf_pwr_mode_t pwr_mode);

void hal_nrf_set_rf_channel(uint8_t channel);

bool hal_nrf_rx_fifo_empty(void);

uint8_t hal_nrf_get_rx_data_source(void);

uint16_t hal_nrf_read_rx_pload(uint8_t *rx_pload);

void hal_nrf_write_tx_pload(uint8_t *tx_pload, uint8_t length);

void hal_nrf_flush_tx(void);

uint8_t hal_nrf_nop(void);

uint8_t hal_nrf_read_reg(uint8_t reg);

uint8_t hal_nrf_write_reg(uint8_t reg, uint8_t value);

uint16_t hal_nrf_read_multibyte_reg(uint8_t reg, uint8_t *pbuf);

void hal_nrf_write_multibyte_reg(uint8_t reg, uint8_t *pbuf, uint8_t length);

uint8_t hal_nrf_rw(uint8_t value);

#endif /* NRF_H_ */
