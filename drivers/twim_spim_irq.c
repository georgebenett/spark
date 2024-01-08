#include "twim_spim_irq.h"

enum instance {
	TWIM_SPIM_IRQ_0 = 0,
	TWIM_SPIM_IRQ_1,
	TWIM_SPIM_IRQ_2,
	TWIM_SPIM_IRQ_3,
	NBR_OF_TWIM_SPIM_INSTANCES,
};

struct callback {
	twim_spim_irq_cb	func;
	enum instance		instance;
};

static struct {
	struct callback		cb[NBR_OF_TWIM_SPIM_INSTANCES];
	uint8_t				num_cb;
} me;

static void process_irq(enum instance instance)
{
	uint8_t i;

	for (i = 0; i < NBR_OF_TWIM_SPIM_INSTANCES; i++) {
		if (me.cb[i].instance == instance) {
			if (me.cb[i].func)
				me.cb[i].func();
			break;
		}
	}
}

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
{
	process_irq(TWIM_SPIM_IRQ_0);
}

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)
{
	process_irq(TWIM_SPIM_IRQ_1);
}

void SPIM2_SPIS2_SPI2_IRQHandler(void)
{
	process_irq(TWIM_SPIM_IRQ_2);
}

void SPIM3_IRQHandler(void)
{
	process_irq(TWIM_SPIM_IRQ_3);
}

static err_code twim_spim_irq_register(enum instance instance,
	twim_spim_irq_cb cb)
{
	uint8_t i;

	for (i = 0; i < NBR_OF_TWIM_SPIM_INSTANCES; i++)
		if (me.cb[i].func && (me.cb[i].instance == instance))
			return ETWIM_SPIM_IRQ_BUSY;

	for (i = 0; i < NBR_OF_TWIM_SPIM_INSTANCES; i++) {
		if (me.cb[i].func)
			continue;

		me.cb[i].func = cb;
		me.cb[i].instance = instance;

		return ERROR_OK;
	}

	return ETWIM_SPIM_IRQ_CB_OWERFLOW;
}

err_code twim_spim_irq_register_twim(NRF_TWIM_Type *p_instance,
	twim_spim_irq_cb cb)
{
	if (p_instance == NRF_TWIM0)
		return twim_spim_irq_register(TWIM_SPIM_IRQ_0, cb);
	else if (p_instance == NRF_TWIM1)
		return twim_spim_irq_register(TWIM_SPIM_IRQ_1, cb);
	else
		return ETWIM_SPIM_IRQ_INVALID_INSTANCE;
}

err_code twim_spim_irq_register_spim(NRF_SPIM_Type *p_instance,
	twim_spim_irq_cb cb)
{
	if (p_instance == NRF_SPIM0)
		return twim_spim_irq_register(TWIM_SPIM_IRQ_0, cb);
	else if (p_instance == NRF_SPIM1)
		return twim_spim_irq_register(TWIM_SPIM_IRQ_1, cb);
	else if (p_instance == NRF_SPIM2)
		return twim_spim_irq_register(TWIM_SPIM_IRQ_2, cb);
	else if (p_instance == NRF_SPIM3)
		return twim_spim_irq_register(TWIM_SPIM_IRQ_3, cb);
	else
		return ETWIM_SPIM_IRQ_INVALID_INSTANCE;
}
