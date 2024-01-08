#include <stdbool.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>

#include "drivers/led_bargraph.h"
#include "drivers/max6964.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "util.h"

#define MODULE_NAME					BARGR
#define LOG_TAG						STRINGIFY(MODULE_NAME)
#include "log.h"

#define MUTEX_NAME					MODULE_NAME
#define TIMER_NAME					MODULE_NAME

#define DEFAULT_BLINK_PERIOD_MS		1000

static const uint8_t mono_led_off = 0x00;
static const struct leds_rgb_levels rgb_led_off = {
	.red	= 0x00,
	.green	= 0x00,
	.blue	= 0x00,
};

STATIC_MUTEX(MUTEX_NAME);
STATIC_TIMER(TIMER_NAME);

static struct {
	const struct led_bargraph_board		*p_board;

	SemaphoreHandle_t					mutex_handle;
	TimerHandle_t						timer_handle;

	bool								blink_state;
	bool								blink_mode[NBR_LED_BARGRAPH_LEDS];
} me;

static void timer_callback(TimerHandle_t timer)
{
	enum led_bargraph_led i;

	me.blink_state = !me.blink_state;

	for (i = 0; i < NBR_LED_BARGRAPH_LEDS; i++) {
		if (!me.blink_mode[i])
			continue;

		(void)max6964_led_set_visible(me.p_board->leds[i], me.blink_state);
	}
}

static err_code set_level(enum led_bargraph_led i,
	const uint8_t * const p_level,
	const struct leds_rgb_levels * const p_rgb_levels)
{
	if (p_level != NULL)
		return max6964_led_set_level(me.p_board->leds[i], *p_level);
	else if (p_rgb_levels != NULL)
		return max6964_led_set_rgb_level(me.p_board->leds[i], *p_rgb_levels);
	else
		return ELED_BARGRAPH_INVALID_ARG;
}

static err_code set_display_levels(enum led_bargraph_led led_idx,
	const uint8_t * const p_level,
	const struct leds_rgb_levels * const p_rgb_levels,
	enum led_bargraph_mode mode)
{
	enum led_bargraph_led i;
	err_code r;

	for (i = 0; i < NBR_LED_BARGRAPH_LEDS; i++) {
		/* configure led color/brightness level */
		switch (mode) {
		case LED_BARGRAPH_MODE_SINGLE:
			if (i == led_idx)
				r = set_level(i, p_level, p_rgb_levels);
			else
				r = set_level(i, &mono_led_off, &rgb_led_off);
			break;
		case LED_BARGRAPH_MODE_BAR:
			if (i <= led_idx)
				r = set_level(i, p_level, p_rgb_levels);
			else
				r = set_level(i, &mono_led_off, &rgb_led_off);
			break;
		default:
			/* should never happen */
			break;
		}
		ERR_CHECK(r);
	}

	return ERROR_OK;
}

static err_code set_display_blink(enum led_bargraph_led led_idx,
	enum led_bargraph_blink blink)
{
	enum led_bargraph_led i;
	err_code r;

	for (i = 0; i < NBR_LED_BARGRAPH_LEDS; i++) {
		/* configure led blink mode */
		switch (blink) {
		case LED_BARGRAPH_BLINK_OFF:
			me.blink_mode[i] = false;
			break;
		case LED_BARGRAPH_BLINK_ON:
			me.blink_mode[i] = true;
			break;
		case LED_BARGRAPH_BLINK_TOP:
			if (i == led_idx)
				me.blink_mode[i] = true;
			else
				me.blink_mode[i] = false;
			break;
		default:
			/* should never happen */
			break;
		}

		/* update visibility to reflect current blink state */
		r = max6964_led_set_visible(me.p_board->leds[i],
			me.blink_mode[i] ? me.blink_state : true);
		ERR_CHECK(r);
	}

	return ERROR_OK;
}

err_code led_bargraph_set_display(enum led_bargraph_led led_idx, uint8_t level,
	enum led_bargraph_mode mode, enum led_bargraph_blink blink)
{
	err_code r;

	if (!me.p_board)
		return ELED_BARGRAPH_NO_INIT;

	if ((led_idx >= NBR_LED_BARGRAPH_LEDS) ||
		(mode >= NBR_LED_BARGRAPH_MODES) ||
		(blink >= NBR_LED_BARGRAPH_BLINKS))
		return ELED_BARGRAPH_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return ELED_BARGRAPH_MUTEX;

	r = set_display_levels(led_idx, &level, NULL, mode);
	ERR_CHECK_GOTO(r, exit);

	r = set_display_blink(led_idx, blink);

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code led_bargraph_set_rgb_display(enum led_bargraph_led led_idx,
	struct leds_rgb_levels levels, enum led_bargraph_mode mode,
	enum led_bargraph_blink blink)
{
	err_code r;

	if (!me.p_board)
		return ELED_BARGRAPH_NO_INIT;

	if ((led_idx >= NBR_LED_BARGRAPH_LEDS) ||
		(mode >= NBR_LED_BARGRAPH_MODES) ||
		(blink >= NBR_LED_BARGRAPH_BLINKS))
		return ELED_BARGRAPH_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return ELED_BARGRAPH_MUTEX;

	r = set_display_levels(led_idx, NULL, &levels, mode);
	ERR_CHECK_GOTO(r, exit);

	r = set_display_blink(led_idx, blink);

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code led_bargraph_clear_display(void)
{
	enum led_bargraph_led i;
	err_code r;

	if (!me.p_board)
		return ELED_BARGRAPH_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return ELED_BARGRAPH_MUTEX;

	for (i = 0; i < NBR_LED_BARGRAPH_LEDS; i++) {
		me.blink_mode[i] = false;
		r = max6964_led_set_visible(me.p_board->leds[i], false);
		ERR_CHECK_GOTO(r, exit);
	}

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code led_bargraph_set_blink_period(uint32_t blink_period_ms)
{

	if (!me.p_board)
		return ELED_BARGRAPH_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return ELED_BARGRAPH_MUTEX;

	if (xTimerChangePeriod(
			me.timer_handle,
			pdMS_TO_TICKS(blink_period_ms / 2),
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdTRUE)
		LOGE("%s: xTimerChangePeriod failed", __func__);

	util_mutex_unlock(me.mutex_handle);
	return ERROR_OK;
}

err_code led_bargraph_init(const struct led_bargraph_board * const p_board)
{
	if (me.p_board)
		return ERROR_OK;

	if (!p_board)
		return ELED_BARGRAPH_INVALID_ARG;

	me.mutex_handle = CREATE_STATIC_MUTEX(MUTEX_NAME);
	me.timer_handle = CREATE_STATIC_TIMER(
		TIMER_NAME,
		pdMS_TO_TICKS(DEFAULT_BLINK_PERIOD_MS / 2),
		pdTRUE,
		NULL,
		timer_callback);

	me.p_board = p_board;

	if (xTimerReset(
			me.timer_handle,
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdTRUE)
		return ELED_BARGRAPH_TIMER;

	return ERROR_OK;
}
