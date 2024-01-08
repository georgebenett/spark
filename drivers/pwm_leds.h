#pragma once

#include "error.h"

#include "drivers/leds.h"

#define EPWM_LEDS_NO_INIT		(EPWM_LEDS_BASE + 0x00)
#define EPWM_LEDS_INVALID_ARG	(EPWM_LEDS_BASE + 0x01)
#define EPWM_LEDS_MUTEX			(EPWM_LEDS_BASE + 0x02)
#define EPWM_LEDS_TIMER			(EPWM_LEDS_BASE + 0x03)

struct pwm_leds_board {
	uint8_t					pins[NUM_OF_LED_IDX];
	enum leds_active_state	active_state;
};

#if (FEAT_HW_PWM_LEDS == 1)

err_code pwm_leds_set_level(const enum leds_index led, const uint8_t level);
err_code pwm_leds_set_pattern(const enum leds_index led,
	const enum leds_pattern pattern);
err_code pwm_leds_init(const struct pwm_leds_board * const p_board);

#else

__STATIC_INLINE
err_code pwm_leds_set_level(const enum leds_index led, const uint8_t level)
	{ return EPWM_LEDS_NO_INIT; }
__STATIC_INLINE
err_code pwm_leds_set_pattern(const enum leds_index led,
	const enum leds_pattern pattern)
	{ return EPWM_LEDS_NO_INIT; }
__STATIC_INLINE
err_code pwm_leds_init(const struct pwm_leds_board * const p_board)
	{ return ERROR_OK; }

#endif
