#pragma once

#include <app_util_platform.h>
#include "error.h"

#define EGPIO_IRQ_NO_INIT		(EGPIO_IRQ_BASE + 0x00)
#define EGPIO_IRQ_CB_OVERFLOW	(EGPIO_IRQ_BASE + 0x01)
#define EGPIO_IRQ_POLARITY		(EGPIO_IRQ_BASE + 0x02)
#define EGPIO_IRQ_PARAM			(EGPIO_IRQ_BASE + 0x03)

struct gpio_irq_pdata {
	app_irq_priority_t	irq_prio;
};

enum gpio_irq_polarity {
	GPIO_IRQ_POL_LOW = 0,
	GPIO_IRQ_POL_HIGH,
	GPIO_IRQ_POL_TOGGLE,
	NRB_OF_GPIO_IRQ_POLARITIES,
};

typedef void (*gpio_irq_cb)(uint8_t gpio, uint32_t value);

err_code gpio_irq_register(uint8_t gpio, enum gpio_irq_polarity pol,
	gpio_irq_cb cb);
err_code gpio_irq_enable(const int8_t gpio);
err_code gpio_irq_disable(const int8_t gpio);
err_code gpio_irq_init(const struct gpio_irq_pdata * const pdata);
