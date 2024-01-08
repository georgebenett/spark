#pragma once

#include <nrf_saadc.h>

#include "error.h"

#define EAPEMBTN_NO_INIT					(EAPEMBTN_BASE + 0x00)
#define EAPEMBTN_INVALID_ARG				(EAPEMBTN_BASE + 0x01)

struct apem_btn_board {
	int8_t				btn_pin;
};

#if (FEAT_HW_APEM_BTN == 1)

err_code apem_btn_get_val(uint8_t * const p_val);
err_code apem_btn_enable(void);
err_code apem_btn_disable(void);
err_code apem_btn_init(const struct apem_btn_board * const p_board);

#else

__STATIC_INLINE
err_code apem_btn_get_val(uint8_t * const p_val)
	{ return EAPEMBTN_NO_INIT; }
__STATIC_INLINE
err_code apem_btn_enable(void)
	{ return EAPEMBTN_NO_INIT; }
__STATIC_INLINE
err_code apem_btn_disable(void)
	{ return EAPEMBTN_NO_INIT; }
__STATIC_INLINE
err_code apem_btn_init(const struct apem_btn_board * const p_board)
	{ return ERROR_OK; }

#endif
