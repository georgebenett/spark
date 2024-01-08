#include <nrf_gpio.h>
#include <nrf_gpiote.h>

#include <FreeRTOS.h>
#include <timers.h>

#include "boards.h"
#include "drivers/adc.h"
#include "drivers/gpio_irq.h"
#include "drivers/rhc_bat.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "util.h"

#define MODULE_NAME					RHCBA
#define LOG_TAG						STRINGIFY(MODULE_NAME)
#include "log.h"

#define TIMER_NAME					MODULE_NAME
#define DEBOUNCE_TIME_MS			10
#define DEBOUNCE_ON_MSK				0xFFFF
#define DEBOUNCE_OFF_MSK			0x0000
#define DEBOUNCE_START_MSK			0xAAAA

#define ADC_SOC_FULL				14300	/* About 4.2 V */
#define ADC_SOC_EMPTY				10200	/* About 3.0 V */
#define SOC_MAX						100
#define SOC_FROM_ADC(adc_val)		((adc_val - ADC_SOC_EMPTY) /\
									((ADC_SOC_FULL - ADC_SOC_EMPTY) / SOC_MAX))

STATIC_TIMER(TIMER_NAME);

static struct {
	const struct rhc_bat_board	*p_board;
	TimerHandle_t				timer_handle;
	uint32_t					adc_handle;
	uint16_t					debounce;
} me;

static void charge_status_gpio_irq_cb(const uint8_t gpio, const uint32_t value)
{
	BaseType_t yield;

	if ((!me.timer_handle) || (gpio != me.p_board->pins.charge_status))
		return;

	gpio_irq_disable(me.p_board->pins.charge_status);

	yield = pdFALSE;
	(void)xTimerResetFromISR(me.timer_handle, &yield);

	portYIELD_FROM_ISR(yield);
}

static void charge_status_timer_cb(const TimerHandle_t timer)
{
	struct eventpump_param param;
	err_code r;

	me.debounce = ((me.debounce << 1) |
		!nrf_gpio_pin_read(me.p_board->pins.charge_status));

	if ((me.debounce != DEBOUNCE_ON_MSK) && (me.debounce != DEBOUNCE_OFF_MSK)) {
		if (xTimerReset(me.timer_handle, pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) ==
				pdFALSE)
			LOGE("Restart timer");
		return;
	}

	param.source = EVENT_RHC_BAT;
	param.rhc_bat.state = (me.debounce ==
		DEBOUNCE_ON_MSK ? RHC_BAT_CHARGE_STATE_ON : RHC_BAT_CHARGE_STATE_OFF);
	r = eventpump_post(&param);
	if (r != ERROR_OK)
		LOGE("eventpump_post r=0x%08lX", r);

	gpio_irq_enable(me.p_board->pins.charge_status);
}

err_code rhc_bat_get_soc(uint8_t * const p_soc)
{
	int16_t adc_value;
	err_code r;

	if (!me.p_board)
		return ERHCBAT_NO_INIT;

	r = adc_perform_conversion(me.adc_handle, &adc_value);
	ERR_CHECK(r);

	if (adc_value < ADC_SOC_EMPTY)
		*p_soc = 0;
	else
		*p_soc = MIN(SOC_FROM_ADC(adc_value), SOC_MAX);

	return r;
}

err_code rhc_bat_enable_charging(void)
{
	if (!me.p_board)
		return ERHCBAT_NO_INIT;

	nrf_gpio_pin_clear(me.p_board->pins.charge_enable_1);
	nrf_gpio_pin_clear(me.p_board->pins.charge_enable_2);

	return ERROR_OK;
}

err_code rhc_bat_disable_charging(void)
{
	if (!me.p_board)
		return ERHCBAT_NO_INIT;

	nrf_gpio_pin_set(me.p_board->pins.charge_enable_1);
	nrf_gpio_pin_set(me.p_board->pins.charge_enable_2);

	return ERROR_OK;
}

err_code rhc_bat_soc_meas_enable(void)
{
	if (!me.p_board)
		return ERHCBAT_NO_INIT;

	nrf_gpio_pin_set(me.p_board->pins.soc_meas_enable);

	return ERROR_OK;
}

err_code rhc_bat_soc_meas_disable(void)
{
	if (!me.p_board)
		return ERHCBAT_NO_INIT;

	nrf_gpio_pin_clear(me.p_board->pins.soc_meas_enable);

	return ERROR_OK;
}

err_code rhc_bat_init(const struct rhc_bat_board * const p_board)
{
	err_code r;

	if (me.p_board)
		return ERROR_OK;

	if (p_board == NULL)
		return ERHCBAT_INVALID_ARG;

	r = util_validate_pins((uint8_t *)&p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	nrf_gpio_cfg_input(p_board->pins.charge_status, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_pin_clear(p_board->pins.charge_enable_1);
	nrf_gpio_pin_clear(p_board->pins.charge_enable_2);
	nrf_gpio_cfg_output(p_board->pins.charge_enable_1);
	nrf_gpio_cfg_output(p_board->pins.charge_enable_2);
	if (p_board->pins.soc_meas_enable != BOARD_UNUSED_PIN) {
		nrf_gpio_pin_clear(p_board->pins.soc_meas_enable);
		nrf_gpio_cfg_output(p_board->pins.soc_meas_enable);
	}
	if (p_board->pins.soc != BOARD_UNUSED_PIN) {
		nrf_gpio_cfg_input(p_board->pins.soc, NRF_GPIO_PIN_NOPULL);
		r = adc_register(p_board->pins.soc, &me.adc_handle);
		ERR_CHECK(r);
	}

	me.debounce = DEBOUNCE_START_MSK;

	me.timer_handle = CREATE_STATIC_TIMER(TIMER_NAME,
		pdMS_TO_TICKS(DEBOUNCE_TIME_MS), pdFALSE, NULL, charge_status_timer_cb);

	r = gpio_irq_register(p_board->pins.charge_status, GPIO_IRQ_POL_TOGGLE,
		charge_status_gpio_irq_cb);
	ERR_CHECK(r);

	if (xTimerReset(me.timer_handle, pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) ==
			pdFALSE)
		return ERHCBAT_TIMER_RESET;

	me.p_board = p_board;

	return ERROR_OK;
}
