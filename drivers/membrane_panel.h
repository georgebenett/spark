#pragma once

#include "app_util.h"
#include "error.h"
#include "leds.h"

#define EMEMBRANE_PANEL_NO_INIT			(EMEMBRANE_PANEL_BASE + 0x00)
#define EMEMBRANE_PANEL_INVALID_ARG		(EMEMBRANE_PANEL_BASE + 0x01)
#define EMEMBRANE_PANEL_BLINK_TIMER		(EMEMBRANE_PANEL_BASE + 0x02)

enum membrane_panel_led {
	MEMBRANE_PANEL_LED_CONN = 0,
	MEMBRANE_PANEL_LED_INACTIVE,
	MEMBRANE_PANEL_LED_INFO,
	MEMBRANE_PANEL_LED_WARNING,
	NBR_MEMBRANE_PANEL_LEDS
};

enum membrane_panel_event {
	MEMBRANE_PANEL_NOT_PRESSED = 0,
	MEMBRANE_PANEL_SHORT_PRESS,
	MEMBRANE_PANEL_LONG_PRESS,
	MEMBRANE_PANEL_VERY_LONG_PRESS,
};

enum membrane_panel_blink {
	MEMBRANE_PANEL_BLINK_NONE = 0,
	MEMBRANE_PANEL_BLINK_SLOW,
	MEMBRANE_PANEL_BLINK_FAST,
	NBR_MEMBRANE_PANEL_BLINKS
};

struct membrane_panel_board {
	uint8_t leds[NBR_MEMBRANE_PANEL_LEDS];
	int8_t button_pin;
};

struct membrane_panel_button_info {
	enum membrane_panel_event last_event;
	uint32_t last_pressed;
	uint32_t curr_state;
};

#if (FEAT_HW_MEMBRANE_PANEL == 1)

err_code membrane_panel_set_led(enum membrane_panel_led led_idx, uint8_t level);
err_code membrane_panel_set_rgb_levels(struct leds_rgb_levels levels,
	enum membrane_panel_blink blink);
err_code membrane_panel_get_button_info(
	struct membrane_panel_button_info * const p_info);
err_code membrane_panel_init(const struct membrane_panel_board * const p_board);

#else

__STATIC_INLINE
err_code membrane_panel_set_led(enum membrane_panel_led led_idx, uint8_t level)
	{ return EMEMBRANE_PANEL_NO_INIT; }
__STATIC_INLINE
err_code membrane_panel_set_rgb_levels(struct leds_rgb_levels levels,
	enum membrane_panel_blink blink)
	{ return EMEMBRANE_PANEL_NO_INIT; }
__STATIC_INLINE
err_code membrane_panel_get_button_info(
	struct membrane_panel_button_info * const p_info)
	{ return EMEMBRANE_PANEL_NO_INIT; }
__STATIC_INLINE
err_code membrane_panel_init(const struct membrane_panel_board * const p_board)
	{ return ERROR_OK; }

#endif
