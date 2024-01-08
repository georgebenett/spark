#include <app_util_platform.h>
#include <nrf_gpio.h>
#include <nrf_pwm.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>

#include "drivers/leds.h"
#include "drivers/pwm.h"
#include "drivers/pwm_leds.h"
#include "freertos_static.h"
#include "util.h"

#define DRV_NAME					PWM_LED
#define LOG_TAG						STRINGIFY(DRV_NAME)
#include "log.h"

#define SEQUENCE_REPEAT_NUMBER		500
#define SEQUENCE_END_DELAY			0
#define SEQUENCE_ARRAY_SIZE			21
#define SEQUENCE_RESTART_DELAY_MS	10
/* The sequence length is the total number of uint6_t words that will be
 * passed in the pwm waveform array which has the type
 * nrf_pwm_values_wave_form_t (with a content of four uint16_t members).
 */
#define SEQUENCE_LENGTH				(SEQUENCE_ARRAY_SIZE * 4)
#define PWM_COUNTER_TOP_VALUE		4000
#define LED_ON						PWM_COUNTER_TOP_VALUE
#define LED_OFF						0
#define POLARITY_POS				15
#define POLARITY_MSK(active_state)	(active_state << POLARITY_POS)

STATIC_MUTEX(DRV_NAME);
STATIC_TIMER(DRV_NAME);

static struct {
	const struct pwm_leds_board		*p_board;
	SemaphoreHandle_t			mutex_handle;
	TimerHandle_t				timer_handle;
	NRF_PWM_Type				*p_pwm;
	nrf_pwm_values_wave_form_t	pwm_wf[SEQUENCE_ARRAY_SIZE];
	nrf_pwm_sequence_t			pwm_seq;
} me;

/* LED sequence patterns */
static const uint16_t sequence_blink[SEQUENCE_ARRAY_SIZE] = {
	LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF,
	LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF,
	LED_ON, LED_OFF, LED_OFF, LED_OFF, LED_ON, LED_OFF, LED_OFF
};

static const uint16_t sequence_breathe[SEQUENCE_ARRAY_SIZE] = {
	(0.0 * LED_ON), (0.1 * LED_ON), (0.2 * LED_ON), (0.3 * LED_ON),
	(0.4 * LED_ON), (0.5 * LED_ON), (0.6 * LED_ON), (0.7 * LED_ON),
	(0.8 * LED_ON), (0.9 * LED_ON), LED_ON, (0.9 * LED_ON), (0.8 * LED_ON),
	(0.7 * LED_ON), (0.6 * LED_ON), (0.5 * LED_ON), (0.4 * LED_ON),
	(0.3 * LED_ON), (0.2 * LED_ON), (0.1 * LED_ON), (0.0 * LED_ON)
};

static void sequence_end_callback(void)
{
	BaseType_t yield = pdFALSE;

	(void)xTimerStartFromISR(me.timer_handle, &yield);
	portYIELD_FROM_ISR(yield);
}

static void timer_callback(TimerHandle_t timer)
{
	err_code r;

	r = pwm_stop(me.p_pwm);
	if (r != ERROR_OK) {
		LOGE("pwm_stop r=0x%08lX", r);
		return;
	}

	r = pwm_channels_start_seq(me.p_pwm, &me.pwm_seq);
	if (r != ERROR_OK)
		LOGE("pwm_channels_start_seq r=0x%08lX", r);
}

err_code pwm_leds_set_level(enum leds_index led, const uint8_t level)
{
	uint16_t *p_wf_channel;
	uint16_t level_to_set;
	err_code r;
	uint8_t i;

	if (me.p_board == NULL)
		return EPWM_LEDS_NO_INIT;

	if (led >= NUM_OF_LED_IDX)
		return EPWM_LEDS_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EPWM_LEDS_MUTEX;

	/* Set the PWM waveform pointer to the appropriate channel offset */
	p_wf_channel = (uint16_t *)&me.pwm_wf[0];
	switch (led) {
	case LEDS_IDX_RED:
		p_wf_channel += offsetof(nrf_pwm_values_wave_form_t, channel_0) /
									sizeof(uint16_t);
		break;
	case LEDS_IDX_GREEN:
		p_wf_channel += offsetof(nrf_pwm_values_wave_form_t, channel_1) /
									sizeof(uint16_t);
		break;
	case LEDS_IDX_BLUE:
		p_wf_channel += offsetof(nrf_pwm_values_wave_form_t, channel_2) /
									sizeof(uint16_t);
		break;
	default:
		/* Should never happen */
		break;
	}

	level_to_set = PWM_COUNTER_TOP_VALUE -
			(((uint32_t)PWM_COUNTER_TOP_VALUE * level) / UINT8_MAX);

	/* Populate the new PWM waveform data into the pointed-out channel */
	for (i = 0; i < SEQUENCE_ARRAY_SIZE; i++) {
		*p_wf_channel = level_to_set | POLARITY_MSK(!me.p_board->active_state);
		p_wf_channel += sizeof(nrf_pwm_values_wave_form_t) / sizeof(uint16_t);
	}

	if (xTimerReset(me.timer_handle,
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
		r = EPWM_LEDS_TIMER;
	else
		r = ERROR_OK;

	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code pwm_leds_set_pattern(const enum leds_index led,
	const enum leds_pattern pattern)
{
	uint16_t *p_wf_channel;
	const uint16_t *p_wf_data;
	err_code r;
	uint8_t i;

	if (me.p_board == NULL)
		return EPWM_LEDS_NO_INIT;

	if ((led >= NUM_OF_LED_IDX) || (pattern >= NUM_OF_LED_PATTERNS))
		return EPWM_LEDS_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EPWM_LEDS_MUTEX;

	/* Set the PWM waveform pointer to the appropriate channel offset */
	p_wf_channel = (uint16_t *)&me.pwm_wf[0];
	switch (led) {
	case LEDS_IDX_RED:
		p_wf_channel += offsetof(nrf_pwm_values_wave_form_t, channel_0) /
									sizeof(uint16_t);
		break;
	case LEDS_IDX_GREEN:
		p_wf_channel += offsetof(nrf_pwm_values_wave_form_t, channel_1) /
									sizeof(uint16_t);
		break;
	case LEDS_IDX_BLUE:
		p_wf_channel += offsetof(nrf_pwm_values_wave_form_t, channel_2) /
									sizeof(uint16_t);
		break;
	default:
		/* Should never happen */
		break;
	}

	/* Set the data pointer to point at the new PWM waveform data */
	p_wf_data = NULL;
	switch (pattern) {
	case LEDS_PATTERN_BREATHE:
		p_wf_data = sequence_breathe;
		break;
	case LEDS_PATTERN_BLINK:
		p_wf_data = sequence_blink;
		break;
	default:
		/* Should never happen */
		break;
	}

	/* Populate the new PWM waveform data into the pointed-out channel */
	for (i = 0; i < SEQUENCE_ARRAY_SIZE; i++) {
		*p_wf_channel = *p_wf_data | POLARITY_MSK(!me.p_board->active_state);
		p_wf_channel += sizeof(nrf_pwm_values_wave_form_t) / sizeof(uint16_t);
		p_wf_data++;
	}

	if (xTimerReset(me.timer_handle,
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
		r = EPWM_LEDS_TIMER;
	else
		r = ERROR_OK;

	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code pwm_leds_init(const struct pwm_leds_board * const p_board)
{
	err_code r;
	uint8_t i;

	if (me.p_board != NULL)
		return ERROR_OK;

	if (p_board == NULL)
		return EPWM_LEDS_INVALID_ARG;

	r = util_validate_pins(p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	/* Register a PWM channel for each LED pin according to this scheme:
	 * LEDS_IDX_RED = Channel_0
	 * LEDS_IDX_GREEN = Channel_1
	 * LEDS_IDX_BLUE = Channel_2
	 */
	r = pwm_channels_reg(&me.p_pwm, p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	pwm_channels_config(me.p_pwm, NRF_PWM_CLK_16MHz,
						PWM_COUNTER_TOP_VALUE, NRF_PWM_LOAD_WAVE_FORM);

	r = pwm_set_seq_end_handler(me.p_pwm, sequence_end_callback);
	ERR_CHECK(r);

	/* TODO: Adjust LED_ON and OFF macro according to leds_active_state */
	for (i = 0; i < SEQUENCE_ARRAY_SIZE; i++) {
		me.pwm_wf[i].channel_0 = LED_OFF;
		me.pwm_wf[i].channel_1 = LED_OFF;
		me.pwm_wf[i].channel_2 = LED_OFF;
		me.pwm_wf[i].counter_top = PWM_COUNTER_TOP_VALUE;
	}

	me.pwm_seq.values.p_wave_form = me.pwm_wf;
	me.pwm_seq.length = SEQUENCE_LENGTH;
	me.pwm_seq.repeats = SEQUENCE_REPEAT_NUMBER;
	me.pwm_seq.end_delay = SEQUENCE_END_DELAY;

	me.mutex_handle = CREATE_STATIC_MUTEX(DRV_NAME);
	me.timer_handle = CREATE_STATIC_TIMER(DRV_NAME,
						pdMS_TO_TICKS(SEQUENCE_RESTART_DELAY_MS),
						pdFALSE,
						NULL,
						timer_callback);

	me.p_board = p_board;

	return ERROR_OK;
}
