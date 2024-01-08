#include <nrf_gpio.h>
#include <nrf_twim.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "drivers/twim.h"
#include "freertos_static.h"
#include "twim_spim_irq.h"
#include "util.h"

#define MUTEX_NAME				TWIM
#define COMPLETION_TMO_MS		30
#define MAX_NBR_OF_CLIENTS		8

#define TWIM_PIN_INIT(_pin) nrf_gpio_cfg(	\
		_pin,								\
		NRF_GPIO_PIN_DIR_INPUT,				\
		NRF_GPIO_PIN_INPUT_DISCONNECT,		\
		NRF_GPIO_PIN_PULLUP,				\
		NRF_GPIO_PIN_S0D1,					\
		NRF_GPIO_PIN_NOSENSE)

STATIC_MUTEX(MUTEX_NAME);

enum twim_evt {
	TWIM_EVT_START = 0,
	TWIM_EVT_DONE,
	TWIM_EVT_ADDRESS_NACK,
	TWIM_EVT_DATA_NACK,
	NBR_OF_TWIM_EVTS,
};

enum twim_instance_idx {
	TWIM_INST_IDX_0 = 0,
	TWIM_INST_IDX_1,
	NBR_OF_TWIM_INST_INDEXES,
};

struct twim_instance {
	NRF_TWIM_Type				*p_twim;
	struct twim_pins			pins;
	uint8_t						address[MAX_NBR_OF_CLIENTS];
	volatile enum twim_evt		last_evt;
};

static struct {
	const struct twim_board		*p_board;
	struct twim_instance	instance[NBR_OF_TWIM_INST_INDEXES];
	SemaphoreHandle_t		mutex_handle;
	TaskHandle_t			trx_handle;
} me;

static void twim_irq_handler(struct twim_instance *p_inst)
{
	BaseType_t yield_req = pdFALSE;
	uint32_t errorsrc;

	if (nrf_twim_event_check(p_inst->p_twim, NRF_TWIM_EVENT_ERROR)) {
		nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_ERROR);

		if (!nrf_twim_event_check(p_inst->p_twim, NRF_TWIM_EVENT_STOPPED)) {
			nrf_twim_int_disable(p_inst->p_twim, NRF_TWIM_ALL_INTS_MASK);
			nrf_twim_int_enable(p_inst->p_twim, NRF_TWIM_INT_STOPPED_MASK);

			nrf_twim_task_trigger(p_inst->p_twim, NRF_TWIM_TASK_RESUME);
			nrf_twim_task_trigger(p_inst->p_twim, NRF_TWIM_TASK_STOP);
			return;
		}
	}

	if (nrf_twim_event_check(p_inst->p_twim, NRF_TWIM_EVENT_STOPPED)) {
		nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_STOPPED);
		nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_LASTTX);
		nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_LASTRX);

		nrf_twim_shorts_set(p_inst->p_twim, 0);
		nrf_twim_int_disable(p_inst->p_twim, NRF_TWIM_ALL_INTS_MASK);
	}

	errorsrc = nrf_twim_errorsrc_get_and_clear(p_inst->p_twim);

	if (errorsrc & NRF_TWIM_ERROR_ADDRESS_NACK)
		p_inst->last_evt = TWIM_EVT_ADDRESS_NACK;
	else if (errorsrc & NRF_TWIM_ERROR_DATA_NACK)
		p_inst->last_evt = TWIM_EVT_DATA_NACK;
	else
		p_inst->last_evt = TWIM_EVT_DONE;

	vTaskNotifyGiveFromISR(me.trx_handle, &yield_req);
	portYIELD_FROM_ISR(yield_req);
}

static void twim_0_irq_callback(void)
{
	if (me.p_board == NULL || me.instance[TWIM_INST_IDX_0].p_twim == NULL)
		return;

	twim_irq_handler(&me.instance[TWIM_INST_IDX_0]);
}

static void twim_1_irq_callback(void)
{
	if (me.p_board == NULL || me.instance[TWIM_INST_IDX_1].p_twim == NULL)
		return;

	twim_irq_handler(&me.instance[TWIM_INST_IDX_1]);
}

static err_code get_client_instance(const struct twim_client *p_client,
		struct twim_instance **pp_inst)
{
	uint8_t i;
	uint8_t j;

	for (i = 0; i < NBR_OF_TWIM_INST_INDEXES; i++) {
		*pp_inst = &me.instance[i];

		if ((*pp_inst)->p_twim == NULL)
			continue;

		if ((*pp_inst)->pins.sda != p_client->pins.sda ||
				(*pp_inst)->pins.scl != p_client->pins.scl)
			continue;

		for (j = 0; j < MAX_NBR_OF_CLIENTS; j++) {
			if ((*pp_inst)->address[j] == p_client->address)
				return ERROR_OK;
		}

		return ETWIM_CLNT_ADDR_NOT_FOUND;
	}

	return ETWIM_CLNT_INST_NOT_FOUND;
}

static void init_twim_instance(struct twim_instance *p_inst)
{
	TWIM_PIN_INIT(p_inst->pins.scl);
	TWIM_PIN_INIT(p_inst->pins.sda);

	nrf_twim_pins_set(p_inst->p_twim, p_inst->pins.scl, p_inst->pins.sda);
	nrf_twim_frequency_set(p_inst->p_twim, me.p_board->freq);

	NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(p_inst->p_twim),
			APP_IRQ_PRIORITY_LOWEST);
	NRFX_IRQ_ENABLE(nrfx_get_irq_number(p_inst->p_twim));

	nrf_twim_enable(p_inst->p_twim);
}

static void deinit_twim_instance(struct twim_instance *p_inst)
{
	NRFX_IRQ_DISABLE(nrfx_get_irq_number(p_inst->p_twim));

	nrf_twim_int_disable(p_inst->p_twim, NRF_TWIM_ALL_INTS_MASK);
	nrf_twim_shorts_disable(p_inst->p_twim, NRF_TWIM_ALL_SHORTS_MASK);
	nrf_twim_disable(p_inst->p_twim);
}

err_code twim_client_read(const struct twim_client * const p_client,
		uint8_t *p_buf, const uint8_t size)
{
	struct twim_instance *p_inst;
	err_code r;

	if (me.p_board == NULL)
		return ETWIM_NO_INIT;

	if (p_client == NULL || !p_client->address)
		return ETWIM_CLIENT;

	if ((p_buf == NULL) || (size == 0))
		return ETWIM_INVALID_ARG;

	if (!nrfx_is_in_ram(p_buf))
		return ETWIM_ARG_NOT_RAM;

	if (!util_mutex_lock(me.mutex_handle))
		return ETWIM_MUTEX;

	r = get_client_instance(p_client, &p_inst);
	ERR_CHECK_GOTO(r, exit);

	nrf_twim_address_set(p_inst->p_twim, p_client->address);
	nrf_twim_rx_buffer_set(p_inst->p_twim, p_buf, size);
	nrf_twim_shorts_set(p_inst->p_twim, NRF_TWIM_SHORT_LASTRX_STOP_MASK);

	me.trx_handle = xTaskGetCurrentTaskHandle();

	nrf_twim_int_enable(p_inst->p_twim,
			NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK);

	p_inst->last_evt = TWIM_EVT_START;
	nrf_twim_task_trigger(p_inst->p_twim, NRF_TWIM_TASK_STARTRX);

	if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(COMPLETION_TMO_MS)) != pdTRUE)
		r = ETWIM_NO_RESPONSE;

	ERR_CHECK_GOTO(r, exit);

	if (p_inst->last_evt != TWIM_EVT_DONE)
		r = ETWIM_NO_ACK;

exit:
	nrf_twim_int_disable(p_inst->p_twim, NRF_TWIM_ALL_INTS_MASK);

	nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_STOPPED);
	nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_ERROR);

	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code twim_client_indexed_read(const struct twim_client * const p_client,
								const uint8_t * const p_idx_buf,
								const uint8_t idx_size,
								uint8_t *p_read_buf,
								const uint8_t read_size)
{
	struct twim_instance *p_inst;
	err_code r;

	if (me.p_board == NULL)
		return ETWIM_NO_INIT;

	if (p_client == NULL || !p_client->address)
		return ETWIM_CLIENT;

	if ((p_idx_buf == NULL) || (idx_size == 0) ||
			(p_read_buf == NULL) || (read_size == 0))
		return ETWIM_INVALID_ARG;

	if (!nrfx_is_in_ram(p_idx_buf) || !nrfx_is_in_ram(p_read_buf))
		return ETWIM_ARG_NOT_RAM;

	if (!util_mutex_lock(me.mutex_handle))
		return ETWIM_MUTEX;

	r = get_client_instance(p_client, &p_inst);
	ERR_CHECK_GOTO(r, exit);

	nrf_twim_address_set(p_inst->p_twim, p_client->address);
	nrf_twim_tx_buffer_set(p_inst->p_twim, p_idx_buf, idx_size);
	nrf_twim_rx_buffer_set(p_inst->p_twim, p_read_buf, read_size);
	/* Shorts for bypassing STOP bit in between consecutive write-and-read
	 * operation.
	 */
	nrf_twim_shorts_set(p_inst->p_twim, NRF_TWIM_SHORT_LASTTX_STARTRX_MASK |
			NRF_TWIM_SHORT_LASTRX_STOP_MASK);

	me.trx_handle = xTaskGetCurrentTaskHandle();

	nrf_twim_int_enable(p_inst->p_twim,
			NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK);

	p_inst->last_evt = TWIM_EVT_START;
	nrf_twim_task_trigger(p_inst->p_twim, NRF_TWIM_TASK_STARTTX);

	if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(COMPLETION_TMO_MS)) != pdTRUE)
		r = ETWIM_NO_RESPONSE;

	ERR_CHECK_GOTO(r, exit);

	if (p_inst->last_evt != TWIM_EVT_DONE)
		r = ETWIM_NO_ACK;

exit:
	nrf_twim_int_disable(p_inst->p_twim, NRF_TWIM_ALL_INTS_MASK);

	nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_STOPPED);
	nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_ERROR);

	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code twim_client_write(const struct twim_client * const p_client,
								const uint8_t * const p_buf, const uint8_t size)
{
	struct twim_instance *p_inst;
	err_code r;

	if (me.p_board == NULL)
		return ETWIM_NO_INIT;

	if (p_client == NULL || !p_client->address)
		return ETWIM_CLIENT;

	if ((p_buf == NULL) || (size == 0))
		return ETWIM_INVALID_ARG;

	if (!nrfx_is_in_ram(p_buf))
		return ETWIM_ARG_NOT_RAM;

	if (!util_mutex_lock(me.mutex_handle))
		return ETWIM_MUTEX;

	r = get_client_instance(p_client, &p_inst);
	ERR_CHECK_GOTO(r, exit);

	nrf_twim_address_set(p_inst->p_twim, p_client->address);
	nrf_twim_tx_buffer_set(p_inst->p_twim, p_buf, size);
	nrf_twim_shorts_set(p_inst->p_twim, NRF_TWIM_SHORT_LASTTX_STOP_MASK);

	me.trx_handle = xTaskGetCurrentTaskHandle();

	nrf_twim_int_enable(p_inst->p_twim,
			NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK);

	p_inst->last_evt = TWIM_EVT_START;
	nrf_twim_task_trigger(p_inst->p_twim, NRF_TWIM_TASK_STARTTX);

	if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(COMPLETION_TMO_MS)) != pdTRUE)
		r = ETWIM_NO_RESPONSE;

	ERR_CHECK_GOTO(r, exit);

	if (p_inst->last_evt != TWIM_EVT_DONE)
		r = ETWIM_NO_ACK;

exit:
	nrf_twim_int_disable(p_inst->p_twim, NRF_TWIM_ALL_INTS_MASK);

	nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_STOPPED);
	nrf_twim_event_clear(p_inst->p_twim, NRF_TWIM_EVENT_ERROR);

	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code twim_client_unreg(const struct twim_client * const p_client)
{
	struct twim_instance *p_inst;
	bool deinit_instance;
	err_code r;
	uint8_t i;
	uint8_t j;
	uint8_t k;

	if (me.p_board == NULL)
		return ETWIM_NO_INIT;

	if (p_client == NULL || !p_client->address)
		return ETWIM_CLIENT;

	if (!util_mutex_lock(me.mutex_handle))
		return ETWIM_MUTEX;

	for (i = 0; i < NBR_OF_TWIM_INST_INDEXES; i++) {
		p_inst = &me.instance[i];

		if (p_inst->p_twim == NULL)
			continue;

		if (p_inst->pins.sda == p_client->pins.sda &&
				p_inst->pins.scl == p_client->pins.scl) {
			for (j = 0; j < MAX_NBR_OF_CLIENTS; j++) {
				if (p_inst->address[j] != p_client->address)
					continue;

				p_inst->address[j] = 0;

				/* Client unregistered, now check if we need to truncate the
				 * rest of the client list and if needed deactivate
				 * TWIM-instance if no clients left.
				 */
				deinit_instance = true;
				for (k = 0; k < MAX_NBR_OF_CLIENTS; k++) {
					if (!p_inst->address[k])
						continue;

					deinit_instance = false;

					if (k == 0)
						continue;

					if (!p_inst->address[k - 1]) {
						p_inst->address[k - 1] = p_inst->address[k];
						p_inst->address[k] = 0;
					}
				}

				if (deinit_instance) {
					deinit_twim_instance(p_inst);
					p_inst->p_twim = NULL;
				}

				r = ERROR_OK;
				goto exit;
			}

			r = ETWIM_CLNT_ADDR_NOT_FOUND;
			goto exit;
		}
	}

	r = ETWIM_CLNT_REG_NOT_FOUND;

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code twim_client_reg(const struct twim_client * const p_client)
{
	enum twim_instance_idx first_free_instance;
	struct twim_instance *p_inst;
	err_code r;
	uint8_t i;
	uint8_t j;

	if (me.p_board == NULL)
		return ETWIM_NO_INIT;

	if (p_client == NULL)
		return ETWIM_CLIENT;

	r = util_validate_pins((uint8_t *)&p_client->pins, sizeof(p_client->pins));
	ERR_CHECK(r);

	if (!util_mutex_lock(me.mutex_handle))
		return ETWIM_MUTEX;

	first_free_instance = NBR_OF_TWIM_INST_INDEXES;
	for (i = TWIM_INST_IDX_0; i < NBR_OF_TWIM_INST_INDEXES; i++) {
		p_inst = &me.instance[i];

		/* Upon empty instance, mark it as first available to use */
		if (p_inst->p_twim == NULL) {
			if (first_free_instance == NBR_OF_TWIM_INST_INDEXES)
				first_free_instance = i;
			continue;
		}

		/* Upon same pins, occupy the first available addr-slot */
		if (p_inst->pins.sda == p_client->pins.sda &&
				p_inst->pins.scl == p_client->pins.scl) {
			for (j = 0; j < MAX_NBR_OF_CLIENTS; j++) {
				if (p_inst->address[j]) {
					if (p_inst->address[j] == p_client->address) {
						r = ERROR_OK;
						goto exit;
					}
					continue;
				}

				p_inst->address[j] = p_client->address;
				r = ERROR_OK;
				goto exit;
			}

			r = ETWIM_MAX_NBR_CLNTS;
			goto exit;
		}
	}

	switch (first_free_instance) {
	case TWIM_INST_IDX_0:
		me.instance[first_free_instance].p_twim = NRF_TWIM0;
		r = twim_spim_irq_register_twim(NRF_TWIM0, twim_0_irq_callback);
		break;

	case TWIM_INST_IDX_1:
		me.instance[first_free_instance].p_twim = NRF_TWIM1;
		r = twim_spim_irq_register_twim(NRF_TWIM1, twim_1_irq_callback);
		break;

	default:
		r = ETWIM_NO_RESOURCES;
		break;
	}
	ERR_CHECK_GOTO(r, exit);

	me.instance[first_free_instance].pins.sda = p_client->pins.sda;
	me.instance[first_free_instance].pins.scl = p_client->pins.scl;
	me.instance[first_free_instance].address[0] = p_client->address;

	init_twim_instance(&me.instance[first_free_instance]);
	r = ERROR_OK;

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code twim_init(const struct twim_board * const p_board)
{
	if (me.p_board != NULL)
		return ERROR_OK;

	if (p_board == NULL)
		return ETWIM_INVALID_ARG;

	if (p_board->freq != NRF_TWIM_FREQ_100K &&
			p_board->freq != NRF_TWIM_FREQ_250K &&
				p_board->freq != NRF_TWIM_FREQ_400K)
		return ETWIM_FREQ;

	me.mutex_handle = CREATE_STATIC_MUTEX(MUTEX_NAME);

	me.p_board = p_board;

	return ERROR_OK;
}
