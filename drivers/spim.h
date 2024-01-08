#pragma once

#include <nrf_spim.h>
#include <stdbool.h>

#include "error.h"

#define ESPIM_NO_INIT				(ESPIM_BASE + 0x00)
#define ESPIM_INVALID_ARG			(ESPIM_BASE + 0x01)
#define ESPIM_TMO					(ESPIM_BASE + 0x02)
#define ESPIM_NO_RESOURCES			(ESPIM_BASE + 0x03)
#define ESPIM_MAX_NBR_CLNTS			(ESPIM_BASE + 0x04)
#define ESPIM_CLNT_REG_NOT_FOUND	(ESPIM_BASE + 0x05)
#define ESPIM_CLNT_INST_NOT_FOUND	(ESPIM_BASE + 0x06)
#define ESPIM_MUTEX					(ESPIM_BASE + 0x07)
#define ESPIM_ARG_NOT_RAM			(ESPIM_BASE + 0x08)

struct spim_bus_pins {
	int8_t clk;
	int8_t miso;
	int8_t mosi;
} __packed;

struct spim_bus {
	bool					high_speed;
	struct spim_bus_pins	pins;
};

struct spim_device {
	nrf_spim_bit_order_t	bit_order;
	int8_t					cs_pin;
	nrf_spim_frequency_t	freq;
	nrf_spim_mode_t			mode;
	uint8_t					orc;
};

struct spim_client {
	struct spim_bus			bus;
	struct spim_device		device;
};

#if (FEAT_HW_SPIM == 1)

err_code spim_client_transfer(const struct spim_client * const p_client,
		const uint8_t *p_tx_buf, const uint16_t tx_size,
		uint8_t *p_rx_buf, const uint16_t rx_size);
err_code spim_client_unreg(const struct spim_client * const p_client);
err_code spim_client_reg(const struct spim_client * const p_client);
err_code spim_init(void);

#else

__STATIC_INLINE
err_code spim_client_transfer(const struct spim_client * const p_client,
		const uint8_t *p_tx_buf, const uint16_t tx_size,
		uint8_t *p_rx_buf, const uint16_t rx_size)
	{ return ESPIM_NO_INIT; }
__STATIC_INLINE
err_code spim_client_unreg(const struct spim_client * const p_client)
	{ return ESPIM_NO_INIT; }
__STATIC_INLINE
err_code spim_client_reg(const struct spim_client * const p_client)
	{ return ESPIM_NO_INIT; }
__STATIC_INLINE
err_code spim_init(void)
	{ return ERROR_OK; }

#endif
