#pragma once

#include <nrf_spim.h>
#include <nrf_twim.h>

#include "error.h"

#define ETWIM_SPIM_IRQ_BUSY					(ETWIM_SPIM_IRQ_BASE + 0x00)
#define ETWIM_SPIM_IRQ_CB_OWERFLOW			(ETWIM_SPIM_IRQ_BASE + 0x01)
#define ETWIM_SPIM_IRQ_INVALID_INSTANCE		(ETWIM_SPIM_IRQ_BASE + 0x02)

typedef void (*twim_spim_irq_cb)();

err_code twim_spim_irq_register_twim(NRF_TWIM_Type *p_instance,
	twim_spim_irq_cb cb);
err_code twim_spim_irq_register_spim(NRF_SPIM_Type *p_instance,
	twim_spim_irq_cb cb);
