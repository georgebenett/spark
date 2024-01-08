#pragma once

#include <app_util.h>
#include "error.h"

#define ELEDS_NO_INIT				(ELEDS_BASE + 0x00)
#define ELEDS_INVALID_ARG			(ELEDS_BASE + 0x01)
#define ELEDS_NO_SET_LEVEL_FUNC		(ELEDS_BASE + 0x02)
#define ELEDS_NO_SET_RGB_LEVEL_FUNC	(ELEDS_BASE + 0x03)
#define ELEDS_NO_SET_PATTERN_FUNC	(ELEDS_BASE + 0x04)

#define LED_LEVEL_ON				255
#define LED_LEVEL_OFF				0

enum leds_pattern {
	LEDS_PATTERN_BREATHE = 0,
	LEDS_PATTERN_BLINK,
	NUM_OF_LED_PATTERNS
};

enum leds_active_state {
	LEDS_ACT_STATE_LOW = 0,
	LEDS_ACT_STATE_HIGH,
	NUM_OF_LED_ACT_STATES,
};

enum leds_index {
	LEDS_IDX_RED = 0,
	LEDS_IDX_GREEN,
	LEDS_IDX_BLUE,
	NUM_OF_LED_IDX,
};

struct leds_rgb_levels {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

typedef err_code (*leds_set_level_fnc)(const uint8_t led, const uint8_t level);
typedef err_code (*leds_set_rgb_level_fnc)(const uint8_t led,
	const struct leds_rgb_levels levels);
typedef err_code (*leds_set_pattern_fnc)(const uint8_t led,
	const enum leds_pattern pattern);

struct leds_board {
	leds_set_level_fnc		fp_set_level;
	leds_set_rgb_level_fnc	fp_set_rgb_level;
	leds_set_pattern_fnc	fp_set_pattern;
};

#if (FEAT_HW_LEDS == 1)

err_code leds_set_level(const uint8_t led, const uint8_t level);
err_code leds_set_rgb_level(const uint8_t led,
	const struct leds_rgb_levels levels);
err_code leds_set_pattern(const uint8_t led, const enum leds_pattern pattern);
err_code leds_init(const struct leds_board * const p_board);

#else

__STATIC_INLINE
err_code leds_set_level(const uint8_t led, const uint8_t level)
	{ return ELEDS_NO_INIT; }
__STATIC_INLINE
err_code leds_set_rgb_level(const uint8_t led,
	const struct leds_rgb_levels levels)
	{ return ELEDS_NO_INIT; }
__STATIC_INLINE
err_code leds_set_pattern(const uint8_t led, const enum leds_pattern pattern)
	{ return ELEDS_NO_INIT; }
__STATIC_INLINE
err_code leds_init(const struct leds_board * const p_board)
	{ return ERROR_OK; }

#endif
