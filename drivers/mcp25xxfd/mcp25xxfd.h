#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include <nrf_spim.h>

#include "drivers/can.h"
#include "drivers/mcp25xxfd/mcp25xxfd_def.h"
#include "drivers/mcp25xxfd/mcp25xxfd_reg.h"
#include "error.h"

#define EMCP25XXFD_NO_INIT						(EMCP25XXFD_BASE + 0x00)
#define EMCP25XXFD_PDATA						(EMCP25XXFD_BASE + 0x01)
#define EMCP25XXFD_INVALID_ARG					(EMCP25XXFD_BASE + 0x02)
#define EMCP25XXFD_RESET						(EMCP25XXFD_BASE + 0x03)
#define EMCP25XXFD_SPI_TMO						(EMCP25XXFD_BASE + 0x04)
#define EMCP25XXFD_RX_CRC						(EMCP25XXFD_BASE + 0x05)
#define EMCP25XXFD_CAN_SYS_CLK					(EMCP25XXFD_BASE + 0x06)
#define EMCP25XXFD_CAN_BIT_TIME_NOM				(EMCP25XXFD_BASE + 0x07)
#define EMCP25XXFD_CAN_BIT_TIME_DAT				(EMCP25XXFD_BASE + 0x08)
#define EMCP25XXFD_CAN_BIT_TIME_DAT_INV			(EMCP25XXFD_BASE + 0x09)
#define EMCP25XXFD_INVALID_MODE					(EMCP25XXFD_BASE + 0x0A)
#define EMCP25XXFD_TX_NOT_ENABLED				(EMCP25XXFD_BASE + 0x0B)
#define EMCP25XXFD_TX_DLC_TOO_SMALL				(EMCP25XXFD_BASE + 0x0C)
#define EMCP25XXFD_RX_CAN_TXQUEUE_CH0			(EMCP25XXFD_BASE + 0x0D)
#define EMCP25XXFD_RX_NOT_ENABLED				(EMCP25XXFD_BASE + 0x0E)
#define EMCP25XXFD_PIN_POS						(EMCP25XXFD_BASE + 0x0F)
#define EMCP25XXFD_BUF_OVERSIZE					(EMCP25XXFD_BASE + 0x10)
#define EMCP25XXFD_SET_MODE						(EMCP25XXFD_BASE + 0x11)
#define EMCP25XXFD_TX_FIFO_FULL					(EMCP25XXFD_BASE + 0x12)
#define EMCP25XXFD_RX_FIFO_FULL					(EMCP25XXFD_BASE + 0x13)
#define EMCP25XXFD_CAN_CLIENT_OVERFLOW			(EMCP25XXFD_BASE + 0x14)
#define EMCP25XXFD_CAN_CLIENT_STD_IDS_OVERFLOW	(EMCP25XXFD_BASE + 0x15)
#define EMCP25XXFD_INVALID_CAN_CLIENT_HANDLE	(EMCP25XXFD_BASE + 0x16)
#define EMCP25XXFD_MUTEX						(EMCP25XXFD_BASE + 0x17)
#define EMCP25XXFD_FIFO_SIZE					(EMCP25XXFD_BASE + 0x18)

struct mcp25xxfd_pins {
	int8_t stdby;
	uint8_t spi_cs;
	uint8_t spi_clk;
	uint8_t spi_miso;
	uint8_t spi_mosi;
	int8_t clk_op;
	int8_t int_op;
	int8_t int_0;
	int8_t int_1;
} __packed;

struct mcp25xxfd_can_cfg {
	enum mcp25_can_bittime_setup bit_time;
	enum mcp25_can_sysclk_speed sysclk;
	uint8_t tx_fifo_size;
	uint8_t rx_fifo_size;
};

struct mcp25xxfd_board {
	struct mcp25xxfd_can_cfg can_cfg;
	struct mcp25xxfd_pins pins;
	nrf_spim_frequency_t frequency;
	NRF_SPIM_Type *p_instance;
};

#if (FEAT_HW_MCP25XXFD == 1)

err_code mcp25xxfd_send_can_frame(const struct can_id can_id,
	const uint8_t * const p_buf, const uint8_t buf_size);
err_code mcp25xxfd_unregister_can_rx_client(const uint32_t handle);
err_code mcp25xxfd_register_can_rx_client(uint32_t * const p_handle,
	const struct can_rx_client * const p_clnt);
err_code mcp25xxfd_deinit(void);
err_code mcp25xxfd_init(const struct mcp25xxfd_board * const p_board);

#else

__STATIC_INLINE
err_code mcp25xxfd_send_can_frame(const struct can_id can_id,
	const uint8_t *const p_buf, const uint8_t buf_size)
	{ return EMCP25XXFD_NO_INIT; }
__STATIC_INLINE
err_code mcp25xxfd_unregister_can_rx_client(const uint32_t handle)
	{ return EMCP25XXFD_NO_INIT; }
__STATIC_INLINE
err_code mcp25xxfd_register_can_rx_client(uint32_t * const p_handle,
	const struct can_rx_client * const p_clnt)
	{ return EMCP25XXFD_NO_INIT; }
__STATIC_INLINE
err_code mcp25xxfd_deinit(void)
	{ return ERROR_OK; }
__STATIC_INLINE
err_code mcp25xxfd_init(const struct mcp25xxfd_board * const p_board)
	{ return ERROR_OK; }
#endif
