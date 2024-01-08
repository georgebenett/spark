#include <nrf_gpio.h>

#include "drivers/adc.h"
#include "drivers/apem_btn.h"
#include "drivers/power.h"
#include "util.h"

#define DRV_NAME			APEMB
#define LOG_TAG				STRINGIFY(DRV_NAME)
#include "log.h"

#define BTN_TOP_VALUE		255

static struct {
	const struct apem_btn_board	*p_board;
	uint32_t					adc_handle;
	int16_t						adc_min;
	int16_t						adc_max;
	uint8_t						regulator_5v_handle;
} me;

err_code apem_btn_get_val(uint8_t * const p_val)
{
	int16_t raw_val;
	err_code r;

	if (!me.p_board)
		return EAPEMBTN_NO_INIT;

	if (!p_val)
		return EAPEMBTN_INVALID_ARG;

	r = adc_perform_conversion(me.adc_handle, &raw_val);
	ERR_CHECK(r);

	if (raw_val < me.adc_min)
		*p_val = 0;
	else if (raw_val > me.adc_max)
		*p_val = BTN_TOP_VALUE;
	else
		*p_val = (((raw_val - me.adc_min) * BTN_TOP_VALUE) /
			(me.adc_max - me.adc_min));

	return ERROR_OK;
}

err_code apem_btn_enable(void)
{
	if (!me.p_board)
		return EAPEMBTN_NO_INIT;

	return power_set_regulator_5v_enable_state(true, me.regulator_5v_handle);
}

err_code apem_btn_disable(void)
{
	if (!me.p_board)
		return EAPEMBTN_NO_INIT;

	return power_set_regulator_5v_enable_state(false, me.regulator_5v_handle);
}

err_code apem_btn_init(const struct apem_btn_board * const p_board)
{
	err_code r;

	if (me.p_board)
		return ERROR_OK;

	if (p_board == NULL)
		return EAPEMBTN_INVALID_ARG;

	r = util_validate_pins((uint8_t *)&p_board->btn_pin,
		sizeof(p_board->btn_pin));
	ERR_CHECK(r);

	nrf_gpio_cfg_input(p_board->btn_pin, NRF_GPIO_PIN_NOPULL);
	r = adc_register(p_board->btn_pin, &me.adc_handle);
	ERR_CHECK(r);

	r = power_register_regulator_5v(&me.regulator_5v_handle);
	ERR_CHECK(r);

	r = power_set_regulator_5v_enable_state(false, me.regulator_5v_handle);
	ERR_CHECK(r);

	me.p_board = p_board;

	/* TODO: Add calibration of min/max values. */
	me.adc_min = 1200;
	me.adc_max = 9100;

	return ERROR_OK;
}
