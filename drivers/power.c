#include <nrfx_power.h>
#include <nrf_drv_lpcomp.h>
#include <nrf_gpio.h>
#include <nrf_lpcomp.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>

#include "boards.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "persistent.h"
#include "power.h"
#include "util.h"

#define DRV_NAME				PWR
#define LOG_TAG					STRINGIFY(DRV_NAME)
#include "log.h"

#define SHUTDOWN_CLIENTS_MAX		3
#define DEBOUNCE_TIME_MS			100
#define MAX_REGULATOR_5V_HANDLES	4

STATIC_MUTEX(DRV_NAME);
STATIC_TIMER(DRV_NAME);

static struct {
	SemaphoreHandle_t			mutex_handle;
	TimerHandle_t				timer_handle;
	const struct power_board	*p_board;
	struct persistent_diag		*p_diag;
	uint32_t					reset_reason;
	uint32_t					ticks_start;
	power_event_cb				on_off_irq_cb;
	uint8_t						num_regulator_5v_handles;
	bool						regulator_5v_state[MAX_REGULATOR_5V_HANDLES];
} me;

static void lpcomp_irq_callback(nrf_lpcomp_event_t event)
{
	struct eventpump_param param;
	err_code r;

	if (event == NRF_LPCOMP_EVENT_UP) {
		if (me.on_off_irq_cb)
			me.on_off_irq_cb(POWER_ON);
		return;
	}

	if (event != NRF_LPCOMP_EVENT_DOWN)
		return;

	if (me.on_off_irq_cb)
		me.on_off_irq_cb(POWER_OFF);

	param.source = EVENT_POWER;
	param.power.event = POWER_OFF;
	r = eventpump_post_irq(&param);
	if (r != ERROR_OK)
		LOGE("%s: r=0x%08lX", __func__, r);
}

static void gpio_irq_callback(uint8_t gpio, uint32_t value)
{
	BaseType_t yield;

	if (!me.timer_handle)
		return;

	yield = pdFALSE;
	(void)xTimerResetFromISR(me.timer_handle, &yield);
	portYIELD_FROM_ISR(yield);
}

static void timer_cb(TimerHandle_t timer) {
	struct eventpump_param param;
	err_code r;

	param.source = EVENT_POWER;

	if (nrf_gpio_pin_read(me.p_board->pins.ext_evt))
		param.power.event = POWER_EXT_EVT_PIN_HIGH;
	else
		param.power.event = POWER_EXT_PIN_EVT_LOW;


	r = eventpump_post(&param);
	if (r != ERROR_OK)
		LOGE("%s: r=0x%08lX", __func__, r);
}

static void pofwarn_handler(void)
{
	struct eventpump_param param;
	err_code r;

	param.source = EVENT_POWER;
	param.power.event = POWER_OFF;
	if (util_is_irq_context())
		r = eventpump_post_irq(&param);
	else
		r = eventpump_post(&param);
	if (r != ERROR_OK)
		LOGE("%s: r=0x%08lX", __func__, r);
}

err_code power_register_on_off_irq_cb(const power_event_cb cb)
{
	err_code r;

	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (!cb)
		return EPOWER_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EPOWER_MUTEX;

	if (me.on_off_irq_cb) {
		r = EPOWER_ON_OFF_IRQ_CB_IN_USE;
		goto exit;
	}

	me.on_off_irq_cb = cb;
	r = ERROR_OK;

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

void power_sleep_hook(void)
{
	uint32_t now_ticks = NRF_RTC1->COUNTER;

	if (me.p_diag == NULL)
		return;

	/* We wrapped around */
	if (me.ticks_start > now_ticks)
		now_ticks += 1 << RTC_CNT_BITLENGTH;
	me.p_diag->ticks_executed += (now_ticks - me.ticks_start);
}

void power_wakeup_hook(void)
{
	if (me.p_diag == NULL)
		return;

	me.ticks_start = NRF_RTC1->COUNTER;
	me.p_diag->num_of_wakeups++;
}

err_code power_read_ext_evt_pin(bool * const p_level)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (!p_level)
		return EPOWER_INVALID_ARG;

	if (me.p_board->pins.ext_evt == BOARD_UNUSED_PIN) {
		*p_level = false;
		return ERROR_OK;
	}

	*p_level = nrf_gpio_pin_read(me.p_board->pins.ext_evt);

	return ERROR_OK;
}

err_code power_get_regulator_enable_state(bool * const p_state)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (!p_state)
		return EPOWER_INVALID_ARG;

	if (me.p_board->pins.regulator_enable == BOARD_UNUSED_PIN)
		return EPOWER_REG_EN_PIN_NOT_PRESENT;

	*p_state = !!nrf_gpio_pin_out_read(me.p_board->pins.regulator_enable);

	return ERROR_OK;
}

err_code power_set_regulator_enable_state(const bool state)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (me.p_board->pins.regulator_enable == BOARD_UNUSED_PIN)
		return EPOWER_REG_EN_PIN_NOT_PRESENT;

	if (state)
		nrf_gpio_pin_set(me.p_board->pins.regulator_enable);
	else
		nrf_gpio_pin_clear(me.p_board->pins.regulator_enable);

	return ERROR_OK;
}

err_code power_register_regulator_5v(uint8_t * const p_handle)
{
	err_code r;

	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (!p_handle)
		return EPOWER_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EPOWER_MUTEX;

	r = ERROR_OK;

	if (me.num_regulator_5v_handles >= MAX_REGULATOR_5V_HANDLES) {
		r = EPOWER_REG_5V_HANDLE;
		goto exit;
	}

	*p_handle = me.num_regulator_5v_handles;
	me.num_regulator_5v_handles++;

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code power_get_regulator_5v_enable_state(bool * const p_state)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (!p_state)
		return EPOWER_INVALID_ARG;

	if (me.p_board->pins.regulator_5v_enable == BOARD_UNUSED_PIN)
		return EPOWER_REG_EN_PIN_NOT_PRESENT;

	*p_state =
		!!nrf_gpio_pin_out_read(me.p_board->pins.regulator_5v_enable);

	return ERROR_OK;
}

err_code power_set_regulator_5v_enable_state(const bool state,
	const uint8_t handle)
{
	bool new_state;
	err_code r;
	uint8_t i;

	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (me.p_board->pins.regulator_5v_enable == BOARD_UNUSED_PIN)
		return EPOWER_REG_EN_PIN_NOT_PRESENT;

	if (!util_mutex_lock(me.mutex_handle))
		return EPOWER_MUTEX;

	r = ERROR_OK;

	if (handle >= me.num_regulator_5v_handles) {
		r = EPOWER_REG_5V_HANDLE;
		goto exit;
	}

	me.regulator_5v_state[handle] = state;

	new_state = false;
	for (i = 0; i < me.num_regulator_5v_handles; i++)
		new_state |= me.regulator_5v_state[i];

	nrf_gpio_pin_write(me.p_board->pins.regulator_5v_enable, new_state);

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code power_get_regulator_24v_enable_state(bool * const p_state)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (!p_state)
		return EPOWER_INVALID_ARG;

	if (me.p_board->pins.regulator_24v_enable == BOARD_UNUSED_PIN)
		return EPOWER_REG_EN_PIN_NOT_PRESENT;

	*p_state =
		!!nrf_gpio_pin_out_read(me.p_board->pins.regulator_24v_enable);

	return ERROR_OK;
}

err_code power_set_regulator_24v_enable_state(const bool state)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (me.p_board->pins.regulator_24v_enable == BOARD_UNUSED_PIN)
		return EPOWER_REG_EN_PIN_NOT_PRESENT;

	nrf_gpio_pin_write(me.p_board->pins.regulator_24v_enable, state);

	return ERROR_OK;
}

err_code power_set_wakeup_on_ext_evt_pin(const bool level)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (me.p_board->pins.ext_evt == BOARD_UNUSED_PIN)
		return ERROR_OK;

	if (level)
		nrf_gpio_cfg_sense_input(me.p_board->pins.ext_evt,
			NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_SENSE_HIGH);
	else
		nrf_gpio_cfg_sense_input(me.p_board->pins.ext_evt,
			NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_SENSE_LOW);

	return ERROR_OK;
}

err_code power_get_reset_reason(uint32_t * const p_reason)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (!p_reason)
		return EPOWER_INVALID_ARG;

	*p_reason = me.reset_reason;

	return ERROR_OK;
}

err_code power_get_regulator_can_enable_state(bool * const p_state)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (!p_state)
		return EPOWER_INVALID_ARG;

	if (me.p_board->pins.regulator_can_enable == BOARD_UNUSED_PIN)
		return EPOWER_REG_EN_PIN_NOT_PRESENT;

	*p_state = !nrf_gpio_pin_out_read(me.p_board->pins.regulator_can_enable);

	return ERROR_OK;
}

err_code power_set_regulator_can_enable_state(const bool state)
{
	if (!me.p_board)
		return EPOWER_NO_INIT;

	if (me.p_board->pins.regulator_can_enable == BOARD_UNUSED_PIN)
		return ERROR_OK;

	if (state)
		nrf_gpio_pin_clear(me.p_board->pins.regulator_can_enable);
	else
		nrf_gpio_pin_set(me.p_board->pins.regulator_can_enable);

	return ERROR_OK;
}

bool power_is_initiated(void)
{
	return (me.p_board == NULL ? false : true);
}

err_code power_init(const struct power_board * const p_board)
{
	nrfx_power_pofwarn_config_t pofwarn_config = { 0 };
	nrf_drv_lpcomp_config_t lpcomp_config = { 0 };
	ret_code_t nrf_err;
	err_code r;

	/* Save and clear the reset reason */
	me.reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = 0xFFFFFFFF;

	if (!p_board)
		return EPOWER_INVALID_ARG;

	if (me.p_board)
		return ERROR_OK;

	r = util_validate_pins((uint8_t *)&p_board->pins,
			sizeof(p_board->pins));
	ERR_CHECK(r);

	if (p_board->pins.regulator_enable != BOARD_UNUSED_PIN) {
		nrf_gpio_cfg_output(p_board->pins.regulator_enable);
		nrf_gpio_pin_set(p_board->pins.regulator_enable);
	}

	if (p_board->pins.regulator_5v_enable != BOARD_UNUSED_PIN) {
		nrf_gpio_cfg_output(p_board->pins.regulator_5v_enable);
		nrf_gpio_pin_clear(p_board->pins.regulator_5v_enable);
	}

	if (p_board->pins.regulator_24v_enable != BOARD_UNUSED_PIN) {
		nrf_gpio_cfg_output(p_board->pins.regulator_24v_enable);
		nrf_gpio_pin_clear(p_board->pins.regulator_24v_enable);
	}

	if (p_board->pins.regulator_can_enable != BOARD_UNUSED_PIN) {
		nrf_gpio_pin_set(p_board->pins.regulator_can_enable);
		nrf_gpio_cfg_output(p_board->pins.regulator_can_enable);
	}

	/* GPIO shared with the STM. Event though it's not used by SW,
	 * this pin must be configured correctly if present on the board,
	 * this to avoid interference with the STM pin configuration.
	 */
	if (p_board->pins.plug_detect1 != BOARD_UNUSED_PIN)
		nrf_gpio_cfg_input(p_board->pins.plug_detect1,
			NRF_GPIO_PIN_NOPULL);

	if (p_board->pins.ext_evt != BOARD_UNUSED_PIN) {
		nrf_gpio_cfg_input(p_board->pins.ext_evt, NRF_GPIO_PIN_NOPULL);

		me.timer_handle = CREATE_STATIC_TIMER(
			DRV_NAME,
			pdMS_TO_TICKS(DEBOUNCE_TIME_MS),
			pdFALSE,
			NULL,
			timer_cb);

		r = gpio_irq_register(p_board->pins.ext_evt, GPIO_IRQ_POL_TOGGLE,
							gpio_irq_callback);
		ERR_CHECK(r);
	}

	nrf_err = nrfx_power_init(&p_board->config);
	APP_ERROR_CHECK(nrf_err);

	if (p_board->batt_eject_type >= NUM_POWER_BATT_EJECT_TYPES)
		return EPOWER_INVALID_ARG;

	if (p_board->batt_eject_type == POWER_BATT_EJECT_POFWARN) {
		pofwarn_config.thr = NRF_POWER_POFTHR_V25;
		pofwarn_config.handler = pofwarn_handler;
		nrfx_power_pof_init(&pofwarn_config);
		nrfx_power_pof_enable(&pofwarn_config);
	} else {
		lpcomp_config.interrupt_priority = LPCOMP_CONFIG_IRQ_PRIORITY;
		lpcomp_config.input = p_board->lpcomp_input;
		lpcomp_config.hal.reference = NRF_LPCOMP_REF_SUPPLY_3_8;
		lpcomp_config.hal.detection = NRF_LPCOMP_DETECT_DOWN;
		lpcomp_config.hal.hyst = NRF_LPCOMP_HYST_50mV;

		nrf_err = nrf_drv_lpcomp_init(&lpcomp_config, lpcomp_irq_callback);
		APP_ERROR_CHECK(nrf_err);

		/* The driver does not support enabling multiple interrupts via the
		 * init function so do it manually here.
		 */
		nrf_lpcomp_int_enable(LPCOMP_INTENSET_UP_Msk);
		nrfx_lpcomp_enable();
	}

	me.mutex_handle = CREATE_STATIC_MUTEX(DRV_NAME);
	me.p_board = p_board;

	return ERROR_OK;
}
