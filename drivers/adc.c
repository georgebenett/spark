#include <nrf_gpio.h>
#include <nrf_saadc.h>

#include <FreeRTOS.h>
#include <task.h>

#include <semphr.h>

#include "adc.h"
#include "freertos_static.h"
#include "util.h"

#define DRV_NAME			ADC

#define ADC_CONVERSION_TMO_MS	1

STATIC_MUTEX(DRV_NAME);

static struct {
	TaskHandle_t				notify_handle;
	SemaphoreHandle_t			mutex_handle;
	nrf_saadc_channel_config_t	adc_ch_cfg;
	nrf_saadc_value_t			adc_result[NRF_SAADC_CHANNEL_COUNT];
	nrf_saadc_input_t			active_inputs[NRF_SAADC_CHANNEL_COUNT];
	uint8_t						nbr_active_ch;
	bool						initialized;
} me;

void SAADC_IRQHandler(void)
{
	BaseType_t yield_req = pdFALSE;

	if (nrf_saadc_event_check(NRF_SAADC_EVENT_DONE))
		nrf_saadc_event_clear(NRF_SAADC_EVENT_DONE);

	if (!nrf_saadc_event_check(NRF_SAADC_EVENT_END))
		return;

	nrf_saadc_event_clear(NRF_SAADC_EVENT_END);

	if (me.notify_handle != NULL)
		vTaskNotifyGiveFromISR(me.notify_handle, &yield_req);

	portYIELD_FROM_ISR(yield_req);
}

struct pin_to_input_mapping {
	int8_t				pin;
	nrf_saadc_input_t	input;
};

static const struct pin_to_input_mapping pin_to_input_table[] = {
	{ .pin = NRF_GPIO_PIN_MAP(0, 2), .input = NRF_SAADC_INPUT_AIN0 },
	{ .pin = NRF_GPIO_PIN_MAP(0, 3), .input = NRF_SAADC_INPUT_AIN1 },
	{ .pin = NRF_GPIO_PIN_MAP(0, 4), .input = NRF_SAADC_INPUT_AIN2 },
	{ .pin = NRF_GPIO_PIN_MAP(0, 5), .input = NRF_SAADC_INPUT_AIN3 },
	{ .pin = NRF_GPIO_PIN_MAP(0, 28), .input = NRF_SAADC_INPUT_AIN4 },
	{ .pin = NRF_GPIO_PIN_MAP(0, 29), .input = NRF_SAADC_INPUT_AIN5 },
	{ .pin = NRF_GPIO_PIN_MAP(0, 30), .input = NRF_SAADC_INPUT_AIN6 },
	{ .pin = NRF_GPIO_PIN_MAP(0, 31), .input = NRF_SAADC_INPUT_AIN7 },
};

err_code pin_to_input(const int8_t pin, nrf_saadc_input_t * const p_input)
{
	uint8_t i;

	for (i = 0; i < NRF_SAADC_CHANNEL_COUNT; i++)
		if (pin == pin_to_input_table[i].pin) {
			*p_input = pin_to_input_table[i].input;
			return ERROR_OK;
		}

	return EADC_INVALID_PIN;
}

err_code adc_perform_conversion(const uint32_t handle, int16_t * const p_result)
{
	err_code r;

	if (!me.initialized)
		return EADC_NO_INIT;

	if (handle >= me.nbr_active_ch)
		return EADC_HANDLE;

	if (!util_mutex_lock(me.mutex_handle))
		return EADC_MUTEX;

	me.notify_handle = xTaskGetCurrentTaskHandle();

	nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
	nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
	nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
	nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);

	r = ERROR_OK;
	if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(ADC_CONVERSION_TMO_MS)) != pdTRUE) {
		r = EADC_SAMPLE_TMO;
		goto exit;
	}

	*p_result = me.adc_result[handle];

exit:
	me.notify_handle = NULL;
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code adc_register(const uint8_t pin, uint32_t * const p_handle)
{
	nrf_saadc_input_t input;
	err_code r;
	uint8_t i;

	if (!me.initialized)
		return EADC_NO_INIT;

	if (!p_handle)
		return EADC_INVALID_PARAM;

	if (me.nbr_active_ch >= NRF_SAADC_CHANNEL_COUNT)
		return EADC_CHANNEL_OVERFLOW;

	r = pin_to_input(pin, &input);
	ERR_CHECK(r);

	if (!util_mutex_lock(me.mutex_handle))
		return EADC_MUTEX;

	for (i = 0; i < me.nbr_active_ch; i++)
		if (input == me.active_inputs[i]) {
			r = EADC_INPUT_BUSY;
			goto exit;
		}

	me.adc_ch_cfg.pin_p = input;

	nrf_saadc_channel_init(me.nbr_active_ch, &me.adc_ch_cfg);
	nrf_saadc_buffer_init(&me.adc_result[0], (me.nbr_active_ch + 1));

	me.active_inputs[me.nbr_active_ch] = input;
	*p_handle = me.nbr_active_ch;
	me.nbr_active_ch++;

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code adc_init(void)
{
	nrf_saadc_int_disable(NRF_SAADC_INT_ALL);

	nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_14BIT);
	nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_8X);

	nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
	nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
	nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
	NRFX_IRQ_PRIORITY_SET(SAADC_IRQn, APP_IRQ_PRIORITY_MID);
	NRFX_IRQ_ENABLE(SAADC_IRQn);
	nrf_saadc_int_enable(NRF_SAADC_INT_END);
	nrf_saadc_continuous_mode_disable();
	nrf_saadc_enable();

	me.adc_ch_cfg.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	me.adc_ch_cfg.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	me.adc_ch_cfg.gain = NRF_SAADC_GAIN1_4;
	me.adc_ch_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;
	me.adc_ch_cfg.acq_time = NRF_SAADC_ACQTIME_3US;
	me.adc_ch_cfg.mode = NRF_SAADC_MODE_SINGLE_ENDED;
	me.adc_ch_cfg.burst = NRF_SAADC_BURST_ENABLED;
	me.adc_ch_cfg.pin_n = NRF_SAADC_INPUT_DISABLED;

	me.mutex_handle = CREATE_STATIC_MUTEX(DRV_NAME);

	NVIC_EnableIRQ(SAADC_IRQn);

	me.initialized = true;

	return ERROR_OK;
}
