#include <app_util_platform.h>
#include <nrf_drv_common.h>
#include <nrf_drv_gpiote.h>
#include <nrf_gpio.h>
#include <string.h>

#include "gpio_irq.h"

/*
 * The driver implements an API for registering gpio interrupts.
 *
 * For this purpose the GPIOTE block and the corresponding event interrupts are
 * used. Through this, other drivers are able to register an interrupt event
 * connected to a specific gpio and callback function. The GPIOTE block provides
 * gpio event interrupts on positive, negative and toggled gpio edges.
 */

#define MAX_CALLBACKS			8 /* NRF_GPIOTE_EVENTS_IN_0 .. 7 */
#define GPIOTE_EVENT_IN(index)	offsetof(NRF_GPIOTE_Type, EVENTS_IN[index])
#define INT_ENABLE_MSK(index)	(0x1UL << index)

struct callback {
	gpio_irq_cb	func;
	uint8_t		gpio;
};

static struct {
	struct callback		cb[MAX_CALLBACKS];
	uint8_t				num_callbacks;
	bool				initialized;
} me;

void GPIOTE_IRQHandler(void)
{
	const struct callback *p_cb;
	uint8_t cb_index;

	for (cb_index = 0; cb_index < me.num_callbacks; cb_index++) {
		if (!nrf_gpiote_event_is_set(GPIOTE_EVENT_IN(cb_index)))
			continue;

		nrf_gpiote_event_clear(GPIOTE_EVENT_IN(cb_index));

		p_cb = &me.cb[cb_index];
		if (!p_cb->func)
			continue;

		p_cb->func(p_cb->gpio, nrf_gpio_pin_read(p_cb->gpio));
	}
}

err_code gpio_irq_register(uint8_t gpio, enum gpio_irq_polarity pol,
	gpio_irq_cb cb)
{
	nrf_gpiote_polarity_t gpiote_pol;
	struct callback *p_cb;
	uint8_t cb_index;

	if (!me.initialized)
		return EGPIO_IRQ_NO_INIT;

	if (me.num_callbacks >= MAX_CALLBACKS)
		return EGPIO_IRQ_CB_OVERFLOW;

	if (pol >= NRB_OF_GPIO_IRQ_POLARITIES)
		return EGPIO_IRQ_POLARITY;

	cb_index = me.num_callbacks++;
	p_cb = &me.cb[cb_index];

	p_cb->func = cb;
	p_cb->gpio = gpio;

	switch (pol) {
	case GPIO_IRQ_POL_LOW:
		gpiote_pol = NRF_GPIOTE_POLARITY_HITOLO;
		break;
	case GPIO_IRQ_POL_HIGH:
		gpiote_pol = NRF_GPIOTE_POLARITY_LOTOHI;
		break;
	case GPIO_IRQ_POL_TOGGLE:
		gpiote_pol = NRF_GPIOTE_POLARITY_TOGGLE;
		break;
	default:
		/* Should never be reached */
		break;
	}

	nrf_gpiote_event_configure(cb_index, gpio, gpiote_pol);
	nrf_gpiote_event_clear(GPIOTE_EVENT_IN(cb_index));
	nrf_gpiote_event_enable(cb_index);
	nrf_gpiote_int_enable(INT_ENABLE_MSK(cb_index));

	return ERROR_OK;
}

err_code gpio_irq_enable(const int8_t gpio)
{
	uint8_t i;

	if (!me.initialized)
		return EGPIO_IRQ_NO_INIT;

	for (i = 0; i < me.num_callbacks; i++) {
		if (gpio == me.cb[i].gpio) {
			nrf_gpiote_event_clear(GPIOTE_EVENT_IN(i));
			nrf_gpiote_event_enable(i);
			return ERROR_OK;
		}
	}

	return EGPIO_IRQ_PARAM;
}

err_code gpio_irq_disable(const int8_t gpio)
{
	uint8_t i;

	if (!me.initialized)
		return EGPIO_IRQ_NO_INIT;

	for (i = 0; i < me.num_callbacks; i++) {
		if (gpio == me.cb[i].gpio) {
			nrf_gpiote_event_disable(i);
			return ERROR_OK;
		}
	}

	return EGPIO_IRQ_PARAM;
}

err_code gpio_irq_init(const struct gpio_irq_pdata * const pdata)
{
	if (me.initialized)
		return ERROR_OK;

	NRFX_IRQ_PRIORITY_SET(GPIOTE_IRQn, pdata->irq_prio);
	NRFX_IRQ_ENABLE(GPIOTE_IRQn);

	me.initialized = true;

	return ERROR_OK;
}
