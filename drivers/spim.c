#include <nrf_gpio.h>
#include <nrf_spim.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <string.h>
#include <task.h>

#include "drivers/spim.h"
#include "freertos_static.h"
#include "twim_spim_irq.h"
#include "util.h"

#define MODULE_NAME					SPIM
#define LOG_TAG						STRINGIFY(MODULE_NAME)
#include "log.h"

#define MAX_NBR_OF_CLIENTS		8

#define SPI_COMPLETION_TMO_MS	150
#define SPI_DMA_MAX_LEN			255

#define SPIM_HS_INST_IDX		SPIM_INST_IDX_3

STATIC_MUTEX(SPIM0_MUTEX);
STATIC_MUTEX(SPIM1_MUTEX);
STATIC_MUTEX(SPIM2_MUTEX);
STATIC_MUTEX(SPIM3_MUTEX);

enum spim_instance_idx {
	SPIM_INST_IDX_0 = 0,
	SPIM_INST_IDX_1,
	SPIM_INST_IDX_2,
	SPIM_INST_IDX_3,
	NBR_OF_SPIM_INST_INDEXES,
};

struct spim_instance {
	SemaphoreHandle_t		mutex_handle;
	TaskHandle_t			task_handle;
	struct spim_bus			bus;
	struct spim_device		device[MAX_NBR_OF_CLIENTS];
	NRF_SPIM_Type			*p_spim;
};

static struct {
	struct spim_instance	instance[NBR_OF_SPIM_INST_INDEXES];
	bool					initialized;
} me;

static void spim_irq_handler(struct spim_instance *p_inst)
{
	BaseType_t yield_req;

	yield_req = pdFALSE;

	if (nrf_spim_event_check(p_inst->p_spim, NRF_SPIM_EVENT_END)) {
		nrf_spim_event_clear(p_inst->p_spim, NRF_SPIM_EVENT_END);
		if (p_inst->task_handle)
			vTaskNotifyGiveFromISR(p_inst->task_handle, &yield_req);
	}

	portYIELD_FROM_ISR(yield_req);
}

static void spim_0_irq_callback(void)
{
	if (!me.initialized || !me.instance[SPIM_INST_IDX_0].p_spim)
		return;

	spim_irq_handler(&me.instance[SPIM_INST_IDX_0]);
}

static void spim_1_irq_callback(void)
{
	if (!me.initialized || !me.instance[SPIM_INST_IDX_1].p_spim)
		return;

	spim_irq_handler(&me.instance[SPIM_INST_IDX_1]);
}

static void spim_2_irq_callback(void)
{
	if (!me.initialized || !me.instance[SPIM_INST_IDX_2].p_spim)
		return;

	spim_irq_handler(&me.instance[SPIM_INST_IDX_2]);
}

static void spim_3_irq_callback(void)
{
	if (!me.initialized || !me.instance[SPIM_INST_IDX_3].p_spim)
		return;

	spim_irq_handler(&me.instance[SPIM_INST_IDX_3]);
}

static err_code find_instance(const struct spim_client * const p_client,
	struct spim_instance ** const pp_inst)
{
	int i;

	for (i = NBR_OF_SPIM_INST_INDEXES - 1; i >= 0; i--) {
		*pp_inst = &me.instance[i];

		if (!(*pp_inst)->p_spim)
			continue;

		if (((*pp_inst)->bus.pins.clk == p_client->bus.pins.clk) &&
				((*pp_inst)->bus.pins.miso == p_client->bus.pins.miso) &&
				((*pp_inst)->bus.pins.mosi == p_client->bus.pins.mosi))
			return ERROR_OK;
	}

	return ESPIM_CLNT_INST_NOT_FOUND;
}

static err_code find_client_index(const struct spim_client * const p_client,
	struct spim_instance * const p_inst, int * const p_i)
{
	for (*p_i = 0; *p_i < MAX_NBR_OF_CLIENTS; (*p_i)++)
		if (p_inst->device[*p_i].cs_pin == p_client->device.cs_pin)
			return ERROR_OK;

	return ESPIM_CLNT_REG_NOT_FOUND;
}

static err_code init_client(const struct spim_client * const p_client,
	struct spim_instance * const p_inst)
{
	err_code r;
	int i;

	r = util_validate_pins((uint8_t *)&p_client->device.cs_pin, 1);
	ERR_CHECK(r);

	for (i = 0; i < MAX_NBR_OF_CLIENTS; i++) {
		if (p_inst->device[i].cs_pin == p_client->device.cs_pin)
			return ERROR_OK;

		if (!p_inst->device[i].cs_pin) {
			p_inst->device[i] = p_client->device;

			nrf_gpio_pin_set(p_inst->device[i].cs_pin);
			nrf_gpio_cfg_output(p_inst->device[i].cs_pin);

			return ERROR_OK;
		}
	}

	return ESPIM_MAX_NBR_CLNTS;
}

static err_code deinit_client(const struct spim_client * const p_client,
	struct spim_instance * const p_inst)
{
	err_code r;
	int i;

	r = find_client_index(p_client, p_inst, &i);
	ERR_CHECK(r);

	nrf_gpio_cfg_default(p_inst->device[i].cs_pin);
	memset(&p_inst->device[i], 0, sizeof(p_inst->device[i]));

	return ERROR_OK;
}

static err_code init_instance(const struct spim_bus * const p_bus,
	struct spim_instance ** const pp_inst, bool high_speed)
{
	void (*cb_func)(void);
	err_code r;
	int i;

	r = util_validate_pins((uint8_t *)&p_bus->pins, sizeof(p_bus->pins));
	ERR_CHECK(r);

	for (i = NBR_OF_SPIM_INST_INDEXES - 1; i >= 0; i--) {
		*pp_inst = &me.instance[i];

		if ((*pp_inst)->p_spim != NULL)
			continue;

		if (high_speed && (i != SPIM_HS_INST_IDX))
			continue;

		if (!high_speed && (i == SPIM_HS_INST_IDX))
			continue;

		if (i == SPIM_INST_IDX_0) {
			(*pp_inst)->p_spim = NRF_SPIM0;
			cb_func = spim_0_irq_callback;
			break;
		} else if (i == SPIM_INST_IDX_1) {
			(*pp_inst)->p_spim = NRF_SPIM1;
			cb_func = spim_1_irq_callback;
			break;
		} else if (i == SPIM_INST_IDX_2) {
			(*pp_inst)->p_spim = NRF_SPIM2;
			cb_func = spim_2_irq_callback;
			break;
		} else if (i == SPIM_INST_IDX_3) {
			(*pp_inst)->p_spim = NRF_SPIM3;
			cb_func = spim_3_irq_callback;
			break;
		} else {
			/* Should never be reached */
			return ESPIM_NO_RESOURCES;
		}
	}

	if (i < 0)
		return ESPIM_NO_RESOURCES;

	(*pp_inst)->bus = *p_bus;

	nrf_gpio_cfg_output((*pp_inst)->bus.pins.clk);
	nrf_gpio_cfg_output((*pp_inst)->bus.pins.mosi);
	nrf_gpio_cfg_input((*pp_inst)->bus.pins.miso, NRF_GPIO_PIN_NOPULL);

	nrf_spim_pins_set((*pp_inst)->p_spim, (*pp_inst)->bus.pins.clk,
		(*pp_inst)->bus.pins.mosi, (*pp_inst)->bus.pins.miso);

	r = twim_spim_irq_register_spim((*pp_inst)->p_spim, cb_func);
	ERR_CHECK(r);

	nrf_spim_int_enable((*pp_inst)->p_spim, NRF_SPIM_INT_END_MASK);

	nrf_spim_enable((*pp_inst)->p_spim);

	NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number((*pp_inst)->p_spim),
			APP_IRQ_PRIORITY_LOW);
	NRFX_IRQ_ENABLE(nrfx_get_irq_number((*pp_inst)->p_spim));

	return ERROR_OK;
}

static void try_deinit_instance(struct spim_instance * const p_inst)
{
	int i;

	for (i = 0; i < MAX_NBR_OF_CLIENTS; i++)
		if (p_inst->device[i].cs_pin != 0)
			return;

	NRFX_IRQ_DISABLE(nrfx_get_irq_number(p_inst->p_spim));

	nrf_spim_int_disable(p_inst->p_spim, NRF_SPIM_ALL_INTS_MASK);
	if (!nrf_spim_event_check(p_inst->p_spim, NRF_SPIM_EVENT_STOPPED)) {
		nrf_spim_task_trigger(p_inst->p_spim, NRF_SPIM_TASK_STOP);
		while (!nrf_spim_event_check(p_inst->p_spim, NRF_SPIM_EVENT_STOPPED)) {
		}
	}

	nrf_spim_event_clear(p_inst->p_spim, NRF_SPIM_EVENT_END);
	nrf_spim_disable(p_inst->p_spim);

	nrf_gpio_cfg_default(p_inst->bus.pins.clk);
	nrf_gpio_cfg_default(p_inst->bus.pins.miso);
	nrf_gpio_cfg_default(p_inst->bus.pins.mosi);

	p_inst->p_spim = NULL;
}

err_code spim_client_transfer(const struct spim_client * const p_client,
		const uint8_t *p_tx_buf, const uint16_t tx_size,
		uint8_t *p_rx_buf, const uint16_t rx_size)
{
	struct spim_instance *p_inst;
	uint8_t rx_chunk;
	uint8_t tx_chunk;
	uint16_t rx_left;
	uint16_t tx_left;
	err_code r;
	int i;

	if (!me.initialized)
		return ESPIM_NO_INIT;

	if (!p_client)
		return ESPIM_INVALID_ARG;

	if (!p_rx_buf && !p_tx_buf)
		return ESPIM_INVALID_ARG;

	if ((p_rx_buf && !rx_size) || (p_tx_buf && !tx_size))
		return ESPIM_INVALID_ARG;

	if ((!p_rx_buf && rx_size) || (!p_tx_buf && tx_size))
		return ESPIM_INVALID_ARG;

	if (p_rx_buf && !nrfx_is_in_ram(p_rx_buf))
		return ESPIM_ARG_NOT_RAM;

	if (p_tx_buf && !nrfx_is_in_ram(p_tx_buf))
		return ESPIM_ARG_NOT_RAM;

	r = find_instance(p_client, &p_inst);
	ERR_CHECK(r);

	r = find_client_index(p_client, p_inst, &i);
	ERR_CHECK(r);

	if (!util_mutex_lock(p_inst->mutex_handle))
		return ESPIM_MUTEX;

	nrf_spim_frequency_set(p_inst->p_spim, p_client->device.freq);

	nrf_spim_configure(p_inst->p_spim, p_client->device.mode,
		p_client->device.bit_order);

	nrf_spim_orc_set(p_inst->p_spim, p_client->device.orc);

	p_inst->task_handle = xTaskGetCurrentTaskHandle();
	rx_left = rx_size;
	tx_left = tx_size;

	nrf_gpio_pin_clear(p_client->device.cs_pin);

	do {
		rx_chunk = MIN(SPI_DMA_MAX_LEN, rx_left);
		tx_chunk = MIN(SPI_DMA_MAX_LEN, tx_left);

		nrf_spim_rx_buffer_set(p_inst->p_spim, p_rx_buf, rx_chunk);
		nrf_spim_tx_buffer_set(p_inst->p_spim, p_tx_buf, tx_chunk);

		nrf_spim_event_clear(p_inst->p_spim, NRF_SPIM_EVENT_END);
		nrf_spim_task_trigger(p_inst->p_spim, NRF_SPIM_TASK_START);

		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
				pdMS_TO_TICKS(SPI_COMPLETION_TMO_MS)) != pdTRUE) {
			LOGE("timed out waiting for SPI transaction!");
			nrf_spim_task_trigger(p_inst->p_spim, NRF_SPIM_TASK_STOP);
			r = ESPIM_TMO;
			break;
		}

		p_rx_buf += rx_chunk;
		p_tx_buf += tx_chunk;

		rx_left -= rx_chunk;
		tx_left -= tx_chunk;
	} while ((rx_left > 0) || (tx_left > 0));

	nrf_gpio_pin_set(p_client->device.cs_pin);

	p_inst->task_handle = NULL;

	util_mutex_unlock(p_inst->mutex_handle);
	return r;
}

err_code spim_client_unreg(const struct spim_client * const p_client)
{
	struct spim_instance *p_inst;
	err_code r;

	if (!me.initialized)
		return ESPIM_NO_INIT;

	if (!p_client)
		return ESPIM_INVALID_ARG;

	r = find_instance(p_client, &p_inst);
	ERR_CHECK(r);

	if (!util_mutex_lock(p_inst->mutex_handle))
		return ESPIM_MUTEX;

	r = deinit_client(p_client, p_inst);
	ERR_CHECK_GOTO(r, exit);

	try_deinit_instance(p_inst);

exit:
	util_mutex_unlock(p_inst->mutex_handle);
	return r;
}

err_code spim_client_reg(const struct spim_client * const p_client)
{
	struct spim_instance *p_inst;
	err_code r;

	if (!me.initialized)
		return ESPIM_NO_INIT;

	if (!p_client)
		return ESPIM_INVALID_ARG;

	r = find_instance(p_client, &p_inst);
	if (r == ESPIM_CLNT_INST_NOT_FOUND)
		r = init_instance(&p_client->bus, &p_inst, p_client->bus.high_speed);
	ERR_CHECK(r);

	if (!util_mutex_lock(p_inst->mutex_handle))
		return ESPIM_MUTEX;

	r = init_client(p_client, p_inst);

	util_mutex_unlock(p_inst->mutex_handle);
	return r;
}

err_code spim_init(void)
{
	if (me.initialized)
		return ERROR_OK;

	me.instance[0].mutex_handle = CREATE_STATIC_MUTEX(SPIM0_MUTEX);
	me.instance[1].mutex_handle = CREATE_STATIC_MUTEX(SPIM1_MUTEX);
	me.instance[2].mutex_handle = CREATE_STATIC_MUTEX(SPIM2_MUTEX);
	me.instance[3].mutex_handle = CREATE_STATIC_MUTEX(SPIM3_MUTEX);

	me.initialized = true;

	return ERROR_OK;
}
