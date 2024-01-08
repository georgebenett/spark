#pragma once

#include <stdint.h>

#include "error.h"

#define EBUZZ_NO_INIT		(EBUZZ_BASE + 0x00)
#define EBUZZ_INVALID_ARG	(EBUZZ_BASE + 0x01)
#define EBUZZ_INVALID_PWM	(EBUZZ_BASE + 0x02)

struct buzzer_board {
	uint8_t	gpio;
};

enum buzzer_seq {
	BUZZ_SEQ_RAM_TUNE = 0,
	BUZZ_SEQ_START,
	BUZZ_SEQ_DMS_ON,
	BUZZ_SEQ_DMS_OFF,
	BUZZ_SEQ_ESC_PRECHARGED,
	BUZZ_SEQ_GPS_FIX,
	NBR_OF_BUZZ_SEQ
};

#if (FEAT_HW_BUZZER == 1)

err_code buzzer_run(enum buzzer_seq buzz_seq);
err_code buzzer_upload(const uint32_t *p_data, uint32_t len);
err_code buzzer_init(const struct buzzer_board * const p_board);

#else

__STATIC_INLINE
err_code buzzer_run(enum buzzer_seq buzz_seq)
	{ return EBUZZ_NO_INIT; }
__STATIC_INLINE
err_code buzzer_upload(const uint32_t *p_data, uint32_t len)
	{ return EBUZZ_NO_INIT; }
__STATIC_INLINE
err_code buzzer_init(const struct buzzer_board * const p_board)
	{ return ERROR_OK; }

#endif
