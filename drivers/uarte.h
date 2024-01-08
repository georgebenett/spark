#pragma once

#include <nrf_uarte.h>

#include "error.h"

#define EUARTE_NO_INIT			(EUARTE_BASE + 0x00)
#define EUARTE_INVALID_BAUD		(EUARTE_BASE + 0x01)
#define EUARTE_INVALID_ARG		(EUARTE_BASE + 0x02)
#define EUARTE_NUM_INSTANCE		(EUARTE_BASE + 0x03)
#define EUARTE_INVALID_INSTANCE	(EUARTE_BASE + 0x04)
#define EUARTE_FLASH_MEM		(EUARTE_BASE + 0x05)
#define EUARTE_TX_TMO			(EUARTE_BASE + 0x06)
#define EUARTE_TX_DISABLED		(EUARTE_BASE + 0x07)
#define EUARTE_MUTEX			(EUARTE_BASE + 0x08)
#define EUARTE_RX_BUSY			(EUARTE_BASE + 0x09)
#define EUARTE_RX_DISABLED		(EUARTE_BASE + 0x0C)
#define EUARTE_WRONG_STATE		(EUARTE_BASE + 0x0D)
#define EUARTE_PPI_CH_EXHAUST	(EUARTE_BASE + 0x0E)
#define EUARTE_READ_MUTEX		(EUARTE_BASE + 0x0F)
#define EUARTE_NEED_FASTRX		(EUARTE_BASE + 0x10)

struct uarte_pins {
	uint8_t pin_tx;
	uint8_t pin_rx;
} __packed;

struct uarte_instance {
	NRF_UARTE_Type			*p_nrf;
	nrf_uarte_baudrate_t	baud;
	struct uarte_pins		uarte_pins;
	bool					use_fastrx;
};

struct uarte_board {
	const struct uarte_instance	*p_ins;
	uint8_t						num_instances;
};

err_code uarte_set_baudrate(const NRF_UARTE_Type * const p_nrf,
	const nrf_uarte_baudrate_t baudrate);
err_code uarte_read(const NRF_UARTE_Type * const p_nrf, uint8_t *p_data,
	uint16_t len, uint16_t * const p_numread, uint32_t tmo_ticks);
err_code uarte_write(const NRF_UARTE_Type * const p_nrf,
	const uint8_t *p_data, uint16_t len);
err_code uarte_rx_flush_buffer(const NRF_UARTE_Type * const p_nrf);
err_code uarte_rx_disable(const NRF_UARTE_Type * const p_nrf);
err_code uarte_rx_enable(const NRF_UARTE_Type * const p_nrf);
err_code uarte_tx_disable(const NRF_UARTE_Type * const p_nrf);
err_code uarte_tx_enable(const NRF_UARTE_Type * const p_nrf);
err_code uarte_init(const struct uarte_board * const p_board);
