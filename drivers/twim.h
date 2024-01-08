#pragma once

#include <nrf_twim.h>

#include "error.h"

#define ETWIM_NO_INIT						(ETWIM_BASE + 0x00)
#define ETWIM_INVALID_ARG					(ETWIM_BASE + 0x01)
#define ETWIM_FREQ							(ETWIM_BASE + 0x02)
#define ETWIM_BUSY							(ETWIM_BASE + 0x03)
#define ETWIM_NO_RESPONSE					(ETWIM_BASE + 0x04)
#define ETWIM_NO_ACK						(ETWIM_BASE + 0x05)
#define ETWIM_NO_RESOURCES					(ETWIM_BASE + 0x06)
#define ETWIM_MAX_NBR_CLNTS					(ETWIM_BASE + 0x07)
#define ETWIM_CLNT_ADDR_NOT_FOUND			(ETWIM_BASE + 0x08)
#define ETWIM_CLNT_REG_NOT_FOUND			(ETWIM_BASE + 0x09)
#define ETWIM_CLNT_INST_NOT_FOUND			(ETWIM_BASE + 0x0A)
#define ETWIM_MUTEX							(ETWIM_BASE + 0x0B)
#define ETWIM_CLIENT						(ETWIM_BASE + 0x0C)
#define ETWIM_ARG_NOT_RAM					(ETWIM_BASE + 0x0D)

struct twim_pins {
	uint8_t sda;
	uint8_t scl;
} __packed;

struct twim_client {
	struct twim_pins	pins;
	uint8_t				address;
};

struct twim_board {
	nrf_twim_frequency_t	freq;
};

#if (FEAT_HW_TWIM == 1)

err_code twim_client_read(const struct twim_client * const p_client,
		uint8_t *p_buf, const uint8_t size);
err_code twim_client_indexed_read(const struct twim_client * const p_client,
		const uint8_t * const p_idx_buf, const uint8_t idx_size,
		uint8_t *p_read_buf, const uint8_t read_size);
err_code twim_client_write(const struct twim_client * const p_client,
		const uint8_t * const p_buf, const uint8_t size);
err_code twim_client_unreg(const struct twim_client * const p_client);
err_code twim_client_reg(const struct twim_client * const p_client);
err_code twim_init(const struct twim_board * const p_board);

#else

__STATIC_INLINE
err_code twim_client_read(const struct twim_client * const p_client,
		uint8_t *p_buf, const uint8_t size)
	{ return ETWIM_NO_INIT; }
__STATIC_INLINE
err_code twim_client_indexed_read(const struct twim_client * const p_client,
		const uint8_t * const p_idx_buf, const uint8_t idx_size,
		uint8_t *p_read_buf, const uint8_t read_size)
	{ return ETWIM_NO_INIT; }
__STATIC_INLINE
err_code twim_client_write(const struct twim_client * const p_client,
		const uint8_t * const p_buf, const uint8_t size)
	{ return ETWIM_NO_INIT; }
__STATIC_INLINE
err_code twim_client_unreg(const struct twim_client * const p_client)
	{ return ETWIM_NO_INIT; }
__STATIC_INLINE
err_code twim_client_reg(const struct twim_client * const p_client)
	{ return ETWIM_NO_INIT; }
__STATIC_INLINE
err_code twim_init(const struct twim_board * const p_board)
	{return ERROR_OK;}

#endif
