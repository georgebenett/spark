#pragma once

#include <stdint.h>

#include "app_util.h"
#include "error.h"
#include "leds.h"

#define ELED_BARGRAPH_NO_INIT			(ELED_BARGRAPH_BASE + 0x00)
#define ELED_BARGRAPH_INVALID_ARG		(ELED_BARGRAPH_BASE + 0x01)
#define ELED_BARGRAPH_MUTEX				(ELED_BARGRAPH_BASE + 0x02)
#define ELED_BARGRAPH_QUEUE_FULL		(ELED_BARGRAPH_BASE + 0x03)
#define ELED_BARGRAPH_TIMER				(ELED_BARGRAPH_BASE + 0x04)

enum led_bargraph_led {
	LED_BARGRAPH_LED_0 = 0,
	LED_BARGRAPH_LED_1,
	LED_BARGRAPH_LED_2,
	LED_BARGRAPH_LED_3,
	LED_BARGRAPH_LED_4,
	NBR_LED_BARGRAPH_LEDS
};

enum led_bargraph_mode {
	LED_BARGRAPH_MODE_SINGLE = 0,
	LED_BARGRAPH_MODE_BAR,
	NBR_LED_BARGRAPH_MODES,
};

enum led_bargraph_blink {
	LED_BARGRAPH_BLINK_OFF = 0,
	LED_BARGRAPH_BLINK_ON,
	LED_BARGRAPH_BLINK_TOP,
	NBR_LED_BARGRAPH_BLINKS,
};

struct led_bargraph_board {
	uint8_t leds[NBR_LED_BARGRAPH_LEDS];
};

#if (FEAT_HW_LED_BARGRAPH == 1)

err_code led_bargraph_set_display(enum led_bargraph_led led_idx, uint8_t level,
	enum led_bargraph_mode mode, enum led_bargraph_blink blink);
err_code led_bargraph_set_rgb_display(enum led_bargraph_led led_idx,
	struct leds_rgb_levels levels, enum led_bargraph_mode mode,
	enum led_bargraph_blink blink);
err_code led_bargraph_clear_display(void);
err_code led_bargraph_set_blink_period(uint32_t blink_period_ms);
err_code led_bargraph_init(const struct led_bargraph_board * const p_board);

#else

__STATIC_INLINE
err_code led_bargraph_set_display(enum led_bargraph_led led_idx, uint8_t level,
	enum led_bargraph_mode mode, enum led_bargraph_blink blink)
	{ return ELED_BARGRAPH_NO_INIT; }
__STATIC_INLINE
err_code led_bargraph_set_rgb_display(enum led_bargraph_led led_idx,
	struct leds_rgb_levels levels, enum led_bargraph_mode mode,
	enum led_bargraph_blink blink)
	{ return ELED_BARGRAPH_NO_INIT; }
__STATIC_INLINE
err_code led_bargraph_clear_display(void)
	{ return ELED_BARGRAPH_NO_INIT; }
__STATIC_INLINE
err_code led_bargraph_set_blink_period(uint32_t blink_period_ms)
	{ return ELED_BARGRAPH_NO_INIT; }
__STATIC_INLINE
err_code led_bargraph_init(const struct led_bargraph_board * const p_board)
	{ return ERROR_OK; }

#endif
