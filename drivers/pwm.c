#include <nrf_gpio.h>
#include <nrf_pwm.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include "drivers/pwm.h"
#include "freertos_static.h"
#include "util.h"

#define DRV_NAME					PWM
#define LOG_TAG						STRINGIFY(DRV_NAME)
#include "log.h"

#define PWM_SEQUENCE0				0
/* Number for max channels in waveform mode is 3. */
#define PWM_MAX_CHANNELS			3
#define SELECT_OUTPUT_PIN(pin)		((pin << PWM_PSEL_OUT_PIN_Pos) | \
										(PWM_PSEL_OUT_CONNECT_Connected << \
											PWM_PSEL_OUT_CONNECT_Pos))

#define PWMx_IRQHandler(PWM_INSTANCE) \
	void PWM_INSTANCE##_IRQHandler(void) \
	{ \
		irq_handler(NRF_##PWM_INSTANCE); \
	}

STATIC_MUTEX(DRV_NAME);

enum pwm_index {
	PWM0_INDEX = 0,
	PWM1_INDEX,
	PWM2_INDEX,
	PWM3_INDEX,
	NUM_PWM_INDEXES,
};

static struct {
	const struct pwm_board	*p_board;
	SemaphoreHandle_t		mutex_handle;
	pwm_seq_end_callback	cb[NUM_PWM_INDEXES];
	uint8_t					registered_clients;
} me;

static enum pwm_index pwm_reg_to_index(NRF_PWM_Type *p_reg)
{
	if (p_reg == NRF_PWM0)
		return PWM0_INDEX;
	else if (p_reg == NRF_PWM1)
		return PWM1_INDEX;
	else if (p_reg == NRF_PWM2)
		return PWM2_INDEX;
	else if (p_reg == NRF_PWM3)
		return PWM3_INDEX;
	return PWM0_INDEX;
}

static void irq_handler(NRF_PWM_Type *p_reg)
{
	if (nrf_pwm_event_check(p_reg, NRF_PWM_EVENT_SEQEND0)) {
		nrf_pwm_event_clear(p_reg, NRF_PWM_EVENT_SEQEND0);
		if (me.cb[pwm_reg_to_index(p_reg)])
			me.cb[pwm_reg_to_index(p_reg)]();
	}
}

PWMx_IRQHandler(PWM0)
PWMx_IRQHandler(PWM1)
PWMx_IRQHandler(PWM2)
PWMx_IRQHandler(PWM3)

err_code pwm_channels_reg(NRF_PWM_Type **pp_reg,
					const uint8_t *p_pins, const uint8_t nbr_pins)
{
	err_code r;
	uint8_t i;

	if (me.p_board == NULL)
		return EPWM_NO_INIT;

	if (me.registered_clients >= me.p_board->max_nbr_clients)
		return EPWM_MAX_CLIENTS;

	if (pp_reg == NULL || p_pins == NULL)
		return EPWM_INVALID_ARG;

	if (nbr_pins > PWM_MAX_CHANNELS)
		return EPWM_MAX_CHANNELS;

	if (!util_mutex_lock(me.mutex_handle))
		return EPWM_MUTEX;

	r = ERROR_OK;

	/* Map client index to PWM instance */
	switch (me.registered_clients++) {
	case 0:
		*pp_reg = NRF_PWM0;
		break;
	case 1:
		*pp_reg = NRF_PWM1;
		break;
	case 2:
		*pp_reg = NRF_PWM2;
		break;
	case 3:
		*pp_reg = NRF_PWM3;
		break;
	default:
		/* Should never happen */
		r = EPWM_NO_FREE_PWM;
		goto exit;
	}

	/* Register the pins to be used for each channel */
	for (i = 0; i < nbr_pins; i++) {
		(*pp_reg)->PSEL.OUT[i] = SELECT_OUTPUT_PIN(p_pins[i]);
		nrf_gpio_cfg_output(p_pins[i]);
		nrf_gpio_pin_clear(p_pins[i]);
	}

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

void pwm_channels_config(NRF_PWM_Type *p_reg,
						nrf_pwm_clk_t prescaler, uint16_t countertop,
						nrf_pwm_dec_load_t dec_load)
{
	/* Configure the counters and waveform decoder */
	nrf_pwm_configure(p_reg, prescaler, NRF_PWM_MODE_UP, countertop);
	nrf_pwm_decoder_set(p_reg, dec_load, NRF_PWM_STEP_AUTO);
	nrf_pwm_loop_set(p_reg, 0);
	nrf_pwm_shorts_set(p_reg, 0);
	nrf_pwm_int_set(p_reg, NRF_PWM_INT_SEQEND0_MASK);
}

err_code pwm_channels_start_seq(NRF_PWM_Type *p_reg,
						const nrf_pwm_sequence_t *p_sequence)
{
	if (me.p_board == NULL)
		return EPWM_NO_INIT;

	if (!p_reg || ((p_reg != NRF_PWM0) && (p_reg != NRF_PWM1) &&
			(p_reg != NRF_PWM2) && (p_reg != NRF_PWM3)))
		return EPWM_INVALID_PWM;

	if (!util_is_irq_context())
		if (!util_mutex_lock(me.mutex_handle))
			return EPWM_MUTEX;

	nrf_pwm_event_clear(p_reg, NRF_PWM_EVENT_SEQEND0);
	nrf_pwm_sequence_set(p_reg, PWM_SEQUENCE0, p_sequence);

	NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(p_reg), APP_IRQ_PRIORITY_LOW);
	NRFX_IRQ_ENABLE(nrfx_get_irq_number(p_reg));

	nrf_pwm_enable(p_reg);
	nrf_pwm_task_trigger(p_reg, NRF_PWM_TASK_SEQSTART0);

	if (!util_is_irq_context())
		util_mutex_unlock(me.mutex_handle);
	return ERROR_OK;
}

err_code pwm_stop(NRF_PWM_Type *p_reg)
{
	if (me.p_board == NULL)
		return EPWM_NO_INIT;

	if (!util_is_irq_context())
		if (!util_mutex_lock(me.mutex_handle))
			return EPWM_MUTEX;

	NRFX_IRQ_DISABLE(nrfx_get_irq_number(p_reg));

	nrf_pwm_event_clear(p_reg, NRF_PWM_EVENT_SEQEND0);
	nrf_pwm_task_trigger(p_reg, NRF_PWM_TASK_STOP);
	nrf_pwm_disable(p_reg);

	if (!util_is_irq_context())
		util_mutex_unlock(me.mutex_handle);
	return ERROR_OK;
}

err_code pwm_set_seq_end_handler(NRF_PWM_Type *p_reg, pwm_seq_end_callback cb)
{
	if (me.p_board == NULL)
		return EPWM_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return EPWM_MUTEX;

	me.cb[pwm_reg_to_index(p_reg)] = cb;

	util_mutex_unlock(me.mutex_handle);
	return ERROR_OK;
}

err_code pwm_init(const struct pwm_board * const p_board)
{
	if (me.p_board != NULL)
		return ERROR_OK;

	if (p_board == NULL)
		return EPWM_INVALID_ARG;

	if (!p_board->max_nbr_clients ||
			p_board->max_nbr_clients > NUM_PWM_INDEXES)
		return EPWM_MAX_CLIENTS;

	me.mutex_handle = CREATE_STATIC_MUTEX(DRV_NAME);
	me.p_board = p_board;

	return ERROR_OK;
}
