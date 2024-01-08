#include <nrf_gpio.h>
#include <FreeRTOS.h>
#include <timers.h>

#include "drivers/membrane_panel.h"
#include "drivers/gpio_irq.h"
#include "drivers/max6964.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "util.h"

#define MODULE_NAME					MEMBR
#define LOG_TAG						STRINGIFY(MODULE_NAME)
#include "log.h"

#define DEBOUNCE_TIMER_NAME			MEMBR_DBC_TIMER
#define DEBOUNCE_TIME_MS			10

#define BLINK_TIMER_NAME			MEMBR_BLK_TIMER
#define BLINK_TIME_SLOW_MS			1500
#define BLINK_TIME_FAST_MS			500

#define BUTTON_STATE_PRESSED		0
#define BUTTON_LONG_PRESS_MS		500
#define BUTTON_VERY_LONG_PRESS_MS	2500

STATIC_TIMER(DEBOUNCE_TIMER_NAME);
STATIC_TIMER(BLINK_TIMER_NAME);

static struct {
	const struct membrane_panel_board	*p_board;

	TimerHandle_t						debounce_timer_handle;
	uint32_t							button_state;
	TickType_t							button_pressed_ts;
	TickType_t							button_released_ts;
	enum membrane_panel_event			button_event;

	TimerHandle_t						blink_timer_handle;
	bool								blink_state;
} me;

static void gpio_irq_callback(uint8_t gpio, uint32_t value)
{
	BaseType_t yield;

	if (!me.debounce_timer_handle)
		return;

	yield = pdFALSE;
	(void)xTimerResetFromISR(me.debounce_timer_handle, &yield);
	portYIELD_FROM_ISR(yield);
}

static void debounce_timer_callback(TimerHandle_t timer)
{
	struct eventpump_param param;
	uint32_t button_state;
	TickType_t dt;

	button_state = nrf_gpio_pin_read(me.p_board->button_pin);

	if (button_state == me.button_state)
		return;
	me.button_state = button_state;

	if (button_state == BUTTON_STATE_PRESSED) {
		me.button_pressed_ts = xTaskGetTickCount();
		return;
	}

	me.button_released_ts = xTaskGetTickCount();
	dt = me.button_released_ts - me.button_pressed_ts;

	param.source = EVENT_MEMBRANE_PANEL;
	if (dt < pdMS_TO_TICKS(BUTTON_LONG_PRESS_MS)) {
		me.button_event = MEMBRANE_PANEL_SHORT_PRESS;
	} else if (dt < pdMS_TO_TICKS(BUTTON_VERY_LONG_PRESS_MS)) {
		me.button_event = MEMBRANE_PANEL_LONG_PRESS;
	} else {
		me.button_event = MEMBRANE_PANEL_VERY_LONG_PRESS;
	}

	param.membrane_panel.event = me.button_event;
	(void)eventpump_post(&param);
}

static void blink_timer_callback(TimerHandle_t timer)
{
	(void)timer;

	me.blink_state = !me.blink_state;

	(void)max6964_led_set_visible(MEMBRANE_PANEL_LED_CONN, me.blink_state);
}

static err_code configure_blink_timer(enum membrane_panel_blink blink)
{
	err_code r;

	r = ERROR_OK;

	switch (blink) {
	case MEMBRANE_PANEL_BLINK_NONE:
		if (xTimerStop(
				me.blink_timer_handle,
				pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
			r = EMEMBRANE_PANEL_BLINK_TIMER;
		break;
	case MEMBRANE_PANEL_BLINK_SLOW:
		if (xTimerChangePeriod(
				me.blink_timer_handle,
				pdMS_TO_TICKS(BLINK_TIME_SLOW_MS / 2),
				pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
			r = EMEMBRANE_PANEL_BLINK_TIMER;
		break;
	case MEMBRANE_PANEL_BLINK_FAST:
		if (xTimerChangePeriod(
				me.blink_timer_handle,
				pdMS_TO_TICKS(BLINK_TIME_FAST_MS / 2),
				pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
			r = EMEMBRANE_PANEL_BLINK_TIMER;
		break;
	default:
		/* should never happen */
		break;
	}
	ERR_CHECK(r);

	return max6964_led_set_visible(MEMBRANE_PANEL_LED_CONN,
		(blink == MEMBRANE_PANEL_BLINK_NONE) ? true : me.blink_state);
}

err_code membrane_panel_set_led(enum membrane_panel_led led_idx, uint8_t level)
{
	if (!me.p_board)
		return EMEMBRANE_PANEL_NO_INIT;

	if (led_idx >= NBR_MEMBRANE_PANEL_LEDS)
		return EMEMBRANE_PANEL_INVALID_ARG;

	return max6964_led_set_level(me.p_board->leds[led_idx], level);
}

err_code membrane_panel_set_rgb_levels(struct leds_rgb_levels levels,
	enum membrane_panel_blink blink)
{
	err_code r;

	if (!me.p_board)
		return EMEMBRANE_PANEL_NO_INIT;

	if (blink >= NBR_MEMBRANE_PANEL_BLINKS)
		return EMEMBRANE_PANEL_INVALID_ARG;

	r = max6964_led_set_rgb_level(me.p_board->leds[MEMBRANE_PANEL_LED_CONN],
		levels);
	ERR_CHECK(r);

	r = configure_blink_timer(blink);
	ERR_CHECK(r);

	return r;
}

err_code membrane_panel_get_button_info(
	struct membrane_panel_button_info * const p_info)
{
	p_info->last_event = me.button_event;
	p_info->last_pressed =
		pdTICKS_TO_MS(xTaskGetTickCount() - me.button_released_ts);
	p_info->curr_state = (me.button_state == BUTTON_STATE_PRESSED) ? 1 : 0;

	return ERROR_OK;
}

err_code membrane_panel_init(const struct membrane_panel_board * const p_board)
{
	err_code r;

	if (me.p_board)
		return ERROR_OK;

	if (!p_board)
		return EMEMBRANE_PANEL_INVALID_ARG;

	r = util_validate_pins((uint8_t *)&p_board->button_pin,
		sizeof(p_board->button_pin));
	ERR_CHECK(r);

	nrf_gpio_cfg_input(p_board->button_pin, NRF_GPIO_PIN_NOPULL);

	r = gpio_irq_register(p_board->button_pin, GPIO_IRQ_POL_TOGGLE,
		gpio_irq_callback);
	ERR_CHECK(r);

	me.button_state = !BUTTON_STATE_PRESSED;

	me.debounce_timer_handle = CREATE_STATIC_TIMER(
			DEBOUNCE_TIMER_NAME,
			pdMS_TO_TICKS(DEBOUNCE_TIME_MS),
			pdFALSE,
			NULL,
			debounce_timer_callback);

	me.blink_timer_handle = CREATE_STATIC_TIMER(
			BLINK_TIMER_NAME,
			pdMS_TO_TICKS(BLINK_TIME_SLOW_MS / 2),
			pdTRUE,
			NULL,
			blink_timer_callback);

	me.p_board = p_board;

	return ERROR_OK;
}
