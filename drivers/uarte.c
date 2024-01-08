/**
 * Design notes:
 *
 * When really fast reception of data is needed, like for 1 MBit baudrate,
 * there is not enough time to service an interrupt for each received byte.
 * For that case this driver supports a 'fastrx' mode that use hardware timers
 * TIMER3 and TIMER4 together with large DMA buffers to eliminate the need for
 * most interrupts. Since TIMER0 is typically used for BT and TIMER2 for Gazell,
 * only one of the UART:s can use this mode. The mode roughly works as follow:
 *
 * - When RX has started, we immediately configure the DMA buffer to use for
 *   the next transfer, so it's already ready when the first buffer is filled.
 *
 * - A hardware PPI (Programmable Periphial Interconnect) is configured to
 *   automatically start a new RX transfer when a buffer is filled. This ensures
 *   no byte is lost when switching between buffers.
 *
 * - When a buffer is filled an interrupt is also triggered so the reader task
 *   can be notified of new data. However, there is no built-in support in the
 *   UART hardware for silence detection to see if reception is stopped before
 *   a buffer is filled (which is likely). To solve this, a separate hardware
 *   timer is configured to trigger an interrupt after a suitable timeout.
 *   Additional PPI:s are used to start and stop the timer so it can react to
 *   each incoming byte without involving software. This allows us to get a
 *   reaction time measured in microseconds instead of milliseconds, greatly
 *   increasing the utilization of the UART for query-response protocols.
 *
 * - One problem with the timout handling is that the number of received bytes
 *   is only updated whenever a buffer is filled. This is solved by using one
 *   additional hardware timer as a pure byte counter. PPI:s are configured so
 *   the counter is increased on each received byte, and to store the current
 *   value in capture channels when we need it. Two separate capture channels
 *   are used for this: one for when a buffer is filled and one for when we
 *   get a reception timeout. The reason for this is that the counter value is
 *   updated _before_ the data has been transferred to RAM, and if these two
 *   events occur close enough to each other, and the interrupts has not been
 *   serviced timely enough, we could end up reading bytes that has not been
 *   transferred yet. This problem is eliminated by using separate capture
 *   channels so that the time-of-capture is guaranteed to be separated from
 *   the time-of-read by a few cycles, enough to ensure the RAM transfer for
 *   those bytes has finished.
 */
#include <string.h>

#include <nrf_drv_common.h>
#include <nrf_gpio.h>
#include <nrf_ppi.h>
#include <nrf_timer.h>
#include <nrf_uarte.h>
#include <nrfx_ppi.h>
#include <nrfx_timer.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "drivers/uarte.h"
#include "freertos_static.h"
#include "util.h"

#define DRV_NAME				UARTE
#define DMA_MAX_BUF_LEN			255	/* TODO: could be increased by using chained blocks */
#define FLUSH_BUF_LEN			5	/* Normal receive length + 4 Bytes */
#define ONE_CHAR_LEN			1
#define RX_BUF_LEN				FLUSH_BUF_LEN
#define COMPLETION_TMO_MS		30
#define STOPTX_LOOP_DELAY_MS	1
#define IRQ_PRIORITY			APP_IRQ_PRIORITY_MID

#define UARTE_ALL_INT_MASK		(NRF_UARTE_INT_CTS_MASK |\
									NRF_UARTE_INT_NCTS_MASK |\
									NRF_UARTE_INT_RXDRDY_MASK |\
									NRF_UARTE_INT_ENDRX_MASK |\
									NRF_UARTE_INT_TXDRDY_MASK |\
									NRF_UARTE_INT_ENDTX_MASK |\
									NRF_UARTE_INT_ERROR_MASK |\
									NRF_UARTE_INT_RXTO_MASK |\
									NRF_UARTE_INT_RXSTARTED_MASK |\
									NRF_UARTE_INT_TXSTARTED_MASK |\
									NRF_UARTE_INT_TXSTOPPED_MASK)

#define TX_INTERRUPT_MASK		(NRF_UARTE_INT_ERROR_MASK | \
									NRF_UARTE_INT_ENDTX_MASK | \
									NRF_UARTE_INT_TXSTOPPED_MASK)

#define RX_INTERRUPT_MASK		(NRF_UARTE_INT_ERROR_MASK | \
									NRF_UARTE_INT_ENDRX_MASK | \
									NRF_UARTE_INT_RXTO_MASK)

#define FASTRX_INTERRUPT_MASK	(NRF_UARTE_INT_ENDRX_MASK | \
									NRF_UARTE_INT_RXSTARTED_MASK)

#define ALLRX_INTERRUPT_MASK	(RX_INTERRUPT_MASK | FASTRX_INTERRUPT_MASK)

#define QUEUE_LEN				16

#define RX_CNT_TIMER			NRF_TIMER3
#define RX_CNT_CAP_ENDRX_CH		NRF_TIMER_CC_CHANNEL0
#define RX_CNT_CAP_TMO_CH		NRF_TIMER_CC_CHANNEL1

/* If this is changed, also update the name of the IRQ handler. */
#define RX_TMO_TIMER			NRF_TIMER4
#define RX_TMO_TIME_US			(100) /* 0,1 ms, 10 bytes */
#define RX_TMO_COMP_CH			NRF_TIMER_CC_CHANNEL0

/* Larger buffers means less interrupts, but smaller buffers could in theory
 * be used to start processing CRC before a complete packet has been received.
 */
#define FASTRX_BUF_LEN			1024
#define FASTRX_BUF_CNT			4
#define NEXT_BUF_IDX(idx)		((idx + 1) & (FASTRX_BUF_CNT - 1))

STATIC_ASSERT_MSG((FASTRX_BUF_CNT >= 2), "Invalid FASTRX_BUF_CNT size!");

STATIC_MUTEX(UARTE0_MUTEX);
STATIC_MUTEX(UARTE1_MUTEX);
STATIC_MUTEX(UARTE0_READ_MUTEX);
STATIC_MUTEX(UARTE1_READ_MUTEX);
STATIC_QUEUE(UARTE0_RX, QUEUE_LEN, uint8_t);
STATIC_QUEUE(UARTE1_RX, QUEUE_LEN, uint8_t);

/* On the current nRF52840 MCU there is only two available UARTE hardware
 * instances, these are NRF_UARTE0 and NRF_UARTE1.
 */
enum uarte_instance_idx
{
	UARTE_INSTANCE_INVALID = -1,
	UARTE_INSTANCE_IDX_0,
	UARTE_INSTANCE_IDX_1,
	NUM_UARTE_INSTANCES,
};

struct instance_data {
	uint8_t				rx_buf[RX_BUF_LEN];
	bool				rx_enabled;
	bool				tx_enabled;
	bool				volatile transmitting;
	TaskHandle_t		tx_handle;
	SemaphoreHandle_t	mutex_handle;
	SemaphoreHandle_t	read_mutex_handle;
	QueueHandle_t		queue_handle;
	bool				use_fastrx;
};

struct fastrx_buf {
	uint8_t				data[FASTRX_BUF_LEN];
	/* Volatile since this is written from interrupts. */
	volatile uint32_t	len;
};

struct fastrx {
	struct fastrx_buf	bufs[FASTRX_BUF_CNT];
	/* Volatile since this is written from interrupts. */
	volatile uint8_t	write_buf_idx;
	uint8_t				read_buf_idx;
	uint32_t			read_pos;
	uint32_t			prev_cc;
	TaskHandle_t		rx_handle;
	nrf_ppi_channel_t	ppi_ch_rxrdy1;
	nrf_ppi_channel_t	ppi_ch_rxrdy2;
	nrf_ppi_channel_t	ppi_ch_endrx;
	nrf_ppi_channel_t	ppi_ch_tmo;
};

static struct {
	const struct uarte_board	*p_board;
	struct instance_data		ins_data[NUM_UARTE_INSTANCES];
	/* Singleton since there are only enough timers for one uart. */
	struct fastrx				fastrx;
} me;

__STATIC_INLINE int uarte_nrf_to_idx(const NRF_UARTE_Type *const p_nrf)
{
	if (p_nrf == NRF_UARTE0)
		return UARTE_INSTANCE_IDX_0;
	if (p_nrf == NRF_UARTE1)
		return UARTE_INSTANCE_IDX_1;
	return UARTE_INSTANCE_INVALID;
}

__STATIC_INLINE err_code alloc_ppi(nrf_ppi_channel_t * const p_ch)
{
	if (nrfx_ppi_channel_alloc(p_ch) != NRFX_SUCCESS)
		return EUARTE_PPI_CH_EXHAUST;
	return ERROR_OK;
}

static void irq_handler(NRF_UARTE_Type *p_nrf)
{
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;
	uint8_t write_buf_idx;
	uint32_t endrx_cc;
	uint32_t new_len;
	uint8_t rx_bytes;
	BaseType_t yield;

	if (!me.p_board)
		return;

	yield = pdFALSE;
	instance_idx = uarte_nrf_to_idx(p_nrf);
	p_ins_data = &me.ins_data[instance_idx];

	if (nrf_uarte_event_check(p_nrf, NRF_UARTE_EVENT_ERROR)) {
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_ERROR);
		(void)nrf_uarte_errorsrc_get_and_clear(p_nrf);
		/* Upon Error, attempt to restart ongoing Rx activation
		 * through a STOP-and-FLUSH sequence.
		 */
		if (p_ins_data->rx_enabled && !p_ins_data->use_fastrx)
			nrf_uarte_task_trigger(p_nrf, NRF_UARTE_TASK_STOPRX);
	}

	if (nrf_uarte_event_check(p_nrf, NRF_UARTE_EVENT_RXTO)) {
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_RXTO);
		/* The transmission has either timed-out or it has been stopped.
		 * Either ways the buffer needs to be emptied and prepared for the
		 * next transmission round, hence the flush.
		 */
		if (!p_ins_data->use_fastrx) {
			nrf_uarte_rx_buffer_set(p_nrf, p_ins_data->rx_buf, FLUSH_BUF_LEN);
			nrf_uarte_task_trigger(p_nrf, NRF_UARTE_TASK_FLUSHRX);
		}
	}

	if (nrf_uarte_event_check(p_nrf, NRF_UARTE_EVENT_ENDRX)) {
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_ENDRX);
		/* This event can be generated by the following scenarios:
		 *
		 * RAM buffer is full (data ready)
		 * Issuing NRF_UARTE_TASK_FLUSHRX
		 * Issuing NRF_UARTE_TASK_STOPRX
		 */
		if (!p_ins_data->rx_enabled) {
			/* Do nothing. */
		} else if (p_ins_data->use_fastrx) {
			/* Volatile read. */
			write_buf_idx = me.fastrx.write_buf_idx;
			endrx_cc = nrf_timer_cc_read(RX_CNT_TIMER, RX_CNT_CAP_ENDRX_CH);
			new_len = (me.fastrx.bufs[write_buf_idx].len +
				(endrx_cc - me.fastrx.prev_cc));
			/* If the interrupts are starved then new_len could overflow. */
			if (new_len > FASTRX_BUF_LEN)
				new_len = FASTRX_BUF_LEN;
			me.fastrx.bufs[write_buf_idx].len = new_len;
			me.fastrx.prev_cc = endrx_cc;
			/* When this interrupt occurs we've already switched to the next
			 * buffer. Update state to reflect reality.
			 */
			write_buf_idx = NEXT_BUF_IDX(write_buf_idx);
			me.fastrx.bufs[write_buf_idx].len = 0;
			me.fastrx.write_buf_idx = write_buf_idx;

			if (me.fastrx.rx_handle)
				vTaskNotifyGiveFromISR(me.fastrx.rx_handle, &yield);

		} else {
			rx_bytes = nrf_uarte_rx_amount_get(p_nrf);
			/* Treat singe-byte transmissions as normal behaviour, everything
			 * else is considered as a flush result (i.e just restart).
			 */
			if (rx_bytes == ONE_CHAR_LEN)
				(void)xQueueSendToBackFromISR(p_ins_data->queue_handle,
						(char *)p_ins_data->rx_buf, &yield);

			nrf_uarte_rx_buffer_set(p_nrf, p_ins_data->rx_buf, ONE_CHAR_LEN);
			nrf_uarte_task_trigger(p_nrf, NRF_UARTE_TASK_STARTRX);
		}
	}

	/* Ensure the ENDRX event is processed before this. */
	if (nrf_uarte_event_check(p_nrf, NRF_UARTE_EVENT_RXSTARTED)) {
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_RXSTARTED);

		if (p_ins_data->use_fastrx) {
			/* Prepare the next buffer for usage. STARTRX will automatically
			 * be invoked by a PPI when ENDRX is triggered.
			 */
			nrf_uarte_rx_buffer_set(p_nrf,
				me.fastrx.bufs[NEXT_BUF_IDX(me.fastrx.write_buf_idx)].data,
				FASTRX_BUF_LEN);
		}
	}

	if (nrf_uarte_event_check(p_nrf, NRF_UARTE_EVENT_ENDTX)) {
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_ENDTX);
		if (p_ins_data->transmitting)
			nrf_uarte_task_trigger(p_nrf, NRF_UARTE_TASK_STOPTX);
	}

	if (nrf_uarte_event_check(p_nrf, NRF_UARTE_EVENT_TXSTOPPED)) {
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_TXSTOPPED);
		if (p_ins_data->transmitting) {
			p_ins_data->transmitting = false;
			if (p_ins_data->tx_handle != NULL)
				vTaskNotifyGiveFromISR(p_ins_data->tx_handle, &yield);
		}
	}

	portYIELD_FROM_ISR(yield);
}

void UARTE0_UART0_IRQHandler(void)
{
	irq_handler(NRF_UARTE0);
}

void UARTE1_IRQHandler(void)
{
	irq_handler(NRF_UARTE1);
}

void TIMER4_IRQHandler(void)
{
	BaseType_t yield;
	uint32_t new_len;
	uint32_t tmo_cc;

	yield = pdFALSE;

	if (nrf_timer_event_check(RX_TMO_TIMER,
			nrf_timer_compare_event_get(RX_TMO_COMP_CH))) {
		nrf_timer_event_clear(RX_TMO_TIMER,
			nrf_timer_compare_event_get(RX_TMO_COMP_CH));

		tmo_cc = nrf_timer_cc_read(RX_CNT_TIMER, RX_CNT_CAP_TMO_CH);
		new_len = (me.fastrx.bufs[me.fastrx.write_buf_idx].len +
			(tmo_cc - me.fastrx.prev_cc));
		if (new_len > FASTRX_BUF_LEN)
			new_len = FASTRX_BUF_LEN;
		me.fastrx.bufs[me.fastrx.write_buf_idx].len = new_len;
		me.fastrx.prev_cc = tmo_cc;

		if (me.fastrx.rx_handle)
			vTaskNotifyGiveFromISR(me.fastrx.rx_handle, &yield);
	}

	portYIELD_FROM_ISR(yield);
}

static void clear_interrupt_mask(NRF_UARTE_Type *p_nrf, uint32_t int_mask)
{
	if (int_mask & NRF_UARTE_INT_CTS_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_CTS);
	if (int_mask & NRF_UARTE_INT_NCTS_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_NCTS);
	if (int_mask & NRF_UARTE_INT_RXDRDY_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_RXDRDY);
	if (int_mask & NRF_UARTE_INT_ENDRX_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_ENDRX);
	if (int_mask & NRF_UARTE_INT_TXDRDY_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_TXDRDY);
	if (int_mask & NRF_UARTE_INT_ENDTX_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_ENDTX);
	if (int_mask & NRF_UARTE_INT_ERROR_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_ERROR);
	if (int_mask & NRF_UARTE_INT_RXTO_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_RXTO);
	if (int_mask & NRF_UARTE_INT_RXSTARTED_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_RXSTARTED);
	if (int_mask & NRF_UARTE_INT_TXSTARTED_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_TXSTARTED);
	if (int_mask & NRF_UARTE_INT_TXSTOPPED_MASK)
		nrf_uarte_event_clear(p_nrf, NRF_UARTE_EVENT_TXSTOPPED);
}

static void disable_uarte_instance(const struct uarte_instance *p_ins_board)
{
	nrf_uarte_int_disable(p_ins_board->p_nrf, UARTE_ALL_INT_MASK);
	clear_interrupt_mask(p_ins_board->p_nrf, UARTE_ALL_INT_MASK);
	NRFX_IRQ_DISABLE(nrf_drv_get_IRQn(p_ins_board->p_nrf));
	nrf_uarte_disable(p_ins_board->p_nrf);
}

static void enable_uarte_instance(const struct uarte_instance *p_ins_board)
{
	NRFX_IRQ_ENABLE(nrf_drv_get_IRQn(p_ins_board->p_nrf));
}

static void transfer_data(uint8_t **pp_dst, uint16_t *p_dst_len,
	uint16_t * const p_numread)
{
	uint32_t byte_count;
	uint32_t buf_len;

	while (*p_dst_len > 0) {
		/* Length is volatile so store it in local variable. */
		buf_len = me.fastrx.bufs[me.fastrx.read_buf_idx].len;

		if (me.fastrx.read_pos > buf_len) {
			/* We've most likely lost data. Recover. */
			me.fastrx.read_buf_idx = me.fastrx.write_buf_idx;
			me.fastrx.read_pos = me.fastrx.bufs[me.fastrx.read_buf_idx].len;
			break;
		}

		if (me.fastrx.read_pos < buf_len) {
			byte_count = (buf_len - me.fastrx.read_pos);
			byte_count = MIN(byte_count, *p_dst_len);

			memcpy(*pp_dst, &me.fastrx.bufs[me.fastrx.read_buf_idx].data[
				me.fastrx.read_pos], byte_count);

			me.fastrx.read_pos += byte_count;
			*p_numread += byte_count;
			*pp_dst += byte_count;
			*p_dst_len -= byte_count;
		}

		/* If there is data left then we must've filled the buffer. */
		if (me.fastrx.read_pos < buf_len)
			break;

		/* Ensure we don't move past the write point. */
		if (me.fastrx.read_buf_idx == me.fastrx.write_buf_idx)
			break;

		me.fastrx.read_buf_idx = NEXT_BUF_IDX(me.fastrx.read_buf_idx);
		me.fastrx.read_pos = 0;
	}
}

static void uarte_write_blocking(const struct uarte_instance *p_uarte_ins,
		struct instance_data *p_ins_data, const uint8_t *p_data, uint16_t len)
{
	uint8_t nested;
	uint8_t tx_len;

	sd_nvic_critical_region_enter(&nested);

	nrf_gpio_cfg_output(p_uarte_ins->uarte_pins.pin_tx);
	if (!p_ins_data->rx_enabled)
		nrf_uarte_enable(p_uarte_ins->p_nrf);

	/* Stop any ongoing TX operations */
	nrf_uarte_task_trigger(p_uarte_ins->p_nrf, NRF_UARTE_TASK_STOPTX);
	while (!nrf_uarte_event_check(p_uarte_ins->p_nrf,
			NRF_UARTE_EVENT_TXSTOPPED)){};
	nrf_uarte_event_clear(p_uarte_ins->p_nrf, NRF_UARTE_EVENT_TXSTOPPED);

	do {
		tx_len = MIN(DMA_MAX_BUF_LEN, len);

		nrf_uarte_tx_buffer_set(p_uarte_ins->p_nrf, p_data, tx_len);
		nrf_uarte_task_trigger(p_uarte_ins->p_nrf, NRF_UARTE_TASK_STARTTX);

		while (!nrf_uarte_event_check(p_uarte_ins->p_nrf,
				NRF_UARTE_EVENT_ENDTX)){};
		nrf_uarte_event_clear(p_uarte_ins->p_nrf, NRF_UARTE_EVENT_ENDTX);

		nrf_uarte_task_trigger(p_uarte_ins->p_nrf, NRF_UARTE_TASK_STOPTX);
		while (!nrf_uarte_event_check(p_uarte_ins->p_nrf,
				NRF_UARTE_EVENT_TXSTOPPED)){};
		nrf_uarte_event_clear(p_uarte_ins->p_nrf, NRF_UARTE_EVENT_TXSTOPPED);

		len -= tx_len;
		p_data += tx_len;
	} while (len > 0);

	nrf_gpio_cfg_default(p_uarte_ins->uarte_pins.pin_tx);
	if (!p_ins_data->rx_enabled)
		nrf_uarte_disable(p_uarte_ins->p_nrf);

	sd_nvic_critical_region_exit(nested);
}

err_code uarte_set_baudrate(const NRF_UARTE_Type * const p_nrf,
	const nrf_uarte_baudrate_t baudrate)
{
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;

	if (me.p_board == NULL)
		return EUARTE_NO_INIT;

	if (util_is_irq_context())
		return EUARTE_WRONG_STATE;

	instance_idx = uarte_nrf_to_idx(p_nrf);

	if ((instance_idx < 0) || (instance_idx >= me.p_board->num_instances))
		return EUARTE_INVALID_INSTANCE;

	p_ins_data = &me.ins_data[instance_idx];

	/* The BAUDRATE_* constants have increasing values so this will work. */
	if ((baudrate > NRF_UARTE_BAUDRATE_115200) &&
		!p_ins_data->use_fastrx)
		return EUARTE_NEED_FASTRX;

	if (!util_mutex_lock(p_ins_data->mutex_handle))
		return EUARTE_MUTEX;

	nrf_uarte_baudrate_set(me.p_board->p_ins[instance_idx].p_nrf, baudrate);

	util_mutex_unlock(p_ins_data->mutex_handle);
	return ERROR_OK;
}

err_code uarte_read(const NRF_UARTE_Type * const p_nrf, uint8_t *p_data,
	uint16_t len, uint16_t * const p_numread, uint32_t tmo_ticks)
{
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;
	uint32_t start_ticks;
	uint32_t ticks_left;

	if (me.p_board == NULL)
		return EUARTE_NO_INIT;

	if ((len == 0) || (p_data == NULL) || (p_numread == NULL))
		return EUARTE_INVALID_ARG;

	if (util_is_irq_context() || (xTaskGetSchedulerState() !=
			taskSCHEDULER_RUNNING))
		return EUARTE_WRONG_STATE;

	instance_idx = uarte_nrf_to_idx(p_nrf);

	if ((instance_idx < 0) || (instance_idx >= me.p_board->num_instances))
		return EUARTE_INVALID_INSTANCE;

	p_ins_data = &me.ins_data[instance_idx];

	if (!util_mutex_lock(p_ins_data->read_mutex_handle))
		return EUARTE_READ_MUTEX;

	*p_numread = 0;

	if (p_ins_data->use_fastrx) {
		(void)ulTaskNotifyTake(pdTRUE, 0);
		me.fastrx.rx_handle = xTaskGetCurrentTaskHandle();

		start_ticks = util_ticks_now();
		while (true) {
			transfer_data(&p_data, &len, p_numread);

			if (*p_numread > 0)
				break;

			ticks_left = util_ticks_remaining(start_ticks, tmo_ticks);
			if (ticks_left == 0)
				break;

			(void)ulTaskNotifyTake(pdTRUE, ticks_left);
		}

		me.fastrx.rx_handle = NULL;

	} else {
		while ((len > 0) && (xQueueReceive(p_ins_data->queue_handle,
				p_data, tmo_ticks) == pdTRUE)) {
			/* Once the first byte has been received, fill the buffer as much as
			 * possible without waiting anymore.
			 */
			tmo_ticks = 0;
			p_data++;
			(*p_numread)++;
			len--;
		}
	}

	util_mutex_unlock(p_ins_data->read_mutex_handle);
	return ERROR_OK;
}

err_code uarte_write(const NRF_UARTE_Type * const p_nrf,
	const uint8_t *p_data, uint16_t len)
{
	const struct uarte_instance *p_ins_board;
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;
	uint8_t tx_len;
	err_code r;

	if (me.p_board == NULL)
		return EUARTE_NO_INIT;

	if (p_data == NULL)
		return EUARTE_INVALID_ARG;

	if (!len)
		return ERROR_OK;

	instance_idx = uarte_nrf_to_idx(p_nrf);

	if ((instance_idx < 0) || (instance_idx >= me.p_board->num_instances))
		return EUARTE_INVALID_INSTANCE;

	p_ins_board = &me.p_board->p_ins[instance_idx];
	p_ins_data = &me.ins_data[instance_idx];

	if (!nrfx_is_in_ram(p_data)) {
		configASSERT(0);
		return EUARTE_FLASH_MEM;
	}

	if (util_is_irq_context() || (xTaskGetSchedulerState() !=
			taskSCHEDULER_RUNNING)) {

		if (!p_ins_data->tx_enabled)
			return EUARTE_TX_DISABLED;

		uarte_write_blocking(p_ins_board, p_ins_data, p_data, len);
		return ERROR_OK;
	}

	if (!util_mutex_lock(p_ins_data->mutex_handle))
		return EUARTE_MUTEX;

	if (!p_ins_data->tx_enabled) {
		r = EUARTE_TX_DISABLED;
		goto exit;
	}

	nrf_gpio_cfg_output(p_ins_board->uarte_pins.pin_tx);
	if (!p_ins_data->rx_enabled)
		nrf_uarte_enable(p_ins_board->p_nrf);

	p_ins_data->tx_handle = xTaskGetCurrentTaskHandle();
	r = ERROR_OK;

	do {
		tx_len = (uint8_t)MIN(DMA_MAX_BUF_LEN, len);

		nrf_uarte_tx_buffer_set(p_ins_board->p_nrf, p_data, tx_len);
		nrf_uarte_event_clear(p_ins_board->p_nrf, NRF_UARTE_EVENT_ENDTX);

		p_ins_data->transmitting = true;
		nrf_uarte_task_trigger(p_ins_board->p_nrf, NRF_UARTE_TASK_STARTTX);

		len -= tx_len;
		p_data += tx_len;

		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
				pdMS_TO_TICKS(COMPLETION_TMO_MS)) != pdTRUE)
			r = EUARTE_TX_TMO;
	} while ((len > 0) && (r == ERROR_OK));

	p_ins_data->tx_handle = NULL;
	nrf_gpio_cfg_default(p_ins_board->uarte_pins.pin_tx);
	if (!p_ins_data->rx_enabled)
		nrf_uarte_disable(p_ins_board->p_nrf);

exit:
	util_mutex_unlock(p_ins_data->mutex_handle);
	return r;
}

err_code uarte_rx_flush_buffer(const NRF_UARTE_Type * const p_nrf)
{
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;
	uint8_t data;

	if (me.p_board == NULL)
		return EUARTE_NO_INIT;

	if (util_is_irq_context())
		return EUARTE_WRONG_STATE;

	instance_idx = uarte_nrf_to_idx(p_nrf);

	if ((instance_idx < 0) || (instance_idx >= me.p_board->num_instances))
		return EUARTE_INVALID_INSTANCE;

	p_ins_data = &me.ins_data[instance_idx];

	if (!util_mutex_lock(p_ins_data->read_mutex_handle))
		return EUARTE_READ_MUTEX;

	if (p_ins_data->use_fastrx) {
		me.fastrx.read_buf_idx = me.fastrx.write_buf_idx;
		me.fastrx.read_pos = me.fastrx.bufs[me.fastrx.read_buf_idx].len;
	} else {
		while (xQueueReceive(p_ins_data->queue_handle, &data, 0) == pdTRUE);
	}

	util_mutex_unlock(p_ins_data->read_mutex_handle);
	return ERROR_OK;
}

err_code uarte_rx_disable(const NRF_UARTE_Type * const p_nrf)
{
	const struct uarte_instance *p_ins_board;
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;
	err_code r;

	r = ERROR_OK;

	if (me.p_board == NULL)
		return EUARTE_NO_INIT;

	instance_idx = uarte_nrf_to_idx(p_nrf);

	if ((instance_idx < 0) || (instance_idx >= me.p_board->num_instances))
		return EUARTE_INVALID_INSTANCE;

	p_ins_board = &me.p_board->p_ins[instance_idx];
	p_ins_data = &me.ins_data[instance_idx];

	if (!util_mutex_lock(p_ins_data->mutex_handle))
		return EUARTE_MUTEX;

	if (!p_ins_data->rx_enabled)
		goto exit;

	nrf_uarte_int_disable(p_ins_board->p_nrf, ALLRX_INTERRUPT_MASK);
	clear_interrupt_mask(p_ins_board->p_nrf, ALLRX_INTERRUPT_MASK);

	if (p_ins_data->use_fastrx) {
		nrf_timer_int_disable(RX_TMO_TIMER,
			nrf_timer_compare_int_get(RX_TMO_COMP_CH));
		nrf_timer_event_clear(RX_TMO_TIMER,
			nrf_timer_compare_event_get(RX_TMO_COMP_CH));
		nrf_ppi_channel_disable(me.fastrx.ppi_ch_rxrdy1);
		nrf_ppi_channel_disable(me.fastrx.ppi_ch_rxrdy2);
		nrf_ppi_channel_disable(me.fastrx.ppi_ch_endrx);
		nrf_ppi_channel_disable(me.fastrx.ppi_ch_tmo);
		nrf_timer_task_trigger(RX_TMO_TIMER, NRF_TIMER_TASK_STOP);
		nrf_timer_task_trigger(RX_CNT_TIMER, NRF_TIMER_TASK_STOP);
		NRFX_IRQ_DISABLE(nrfx_get_irq_number(RX_TMO_TIMER));
	}

	/* Stop any ongoing Rx transmission */
	nrf_uarte_task_trigger(p_ins_board->p_nrf, NRF_UARTE_TASK_STOPRX);

	if (!p_ins_data->tx_enabled)
		disable_uarte_instance(p_ins_board);

	nrf_gpio_cfg_default(p_ins_board->uarte_pins.pin_rx);
	p_ins_data->rx_enabled = false;

exit:
	util_mutex_unlock(p_ins_data->mutex_handle);
	return r;
}

err_code uarte_rx_enable(const NRF_UARTE_Type * const p_nrf)
{
	const struct uarte_instance *p_ins_board;
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;
	uint8_t write_buf_idx;
	err_code r;

	r = ERROR_OK;

	if (me.p_board == NULL)
		return EUARTE_NO_INIT;

	instance_idx = uarte_nrf_to_idx(p_nrf);

	if ((instance_idx < 0) || (instance_idx >= me.p_board->num_instances))
		return EUARTE_INVALID_INSTANCE;

	p_ins_board = &me.p_board->p_ins[instance_idx];
	p_ins_data = &me.ins_data[instance_idx];

	if (!util_mutex_lock(p_ins_data->mutex_handle))
		return EUARTE_MUTEX;

	if (p_ins_data->rx_enabled) {
		r = EUARTE_RX_BUSY;
		goto exit;
	}

	/* Common configuration for Rx/Tx, hence only needed if no prior act. */
	if (!p_ins_data->tx_enabled)
		enable_uarte_instance(p_ins_board);

	p_ins_data->rx_enabled = true;
	nrf_gpio_cfg_input(p_ins_board->uarte_pins.pin_rx, NRF_GPIO_PIN_PULLUP);

	clear_interrupt_mask(p_ins_board->p_nrf, ALLRX_INTERRUPT_MASK);

	if (p_ins_data->use_fastrx) {
		me.fastrx.prev_cc = 0;
		/* Switch buffer so we don't have to update read_buf_idx, which is
		 * protected by a separate mutex.
		 */
		write_buf_idx = NEXT_BUF_IDX(me.fastrx.write_buf_idx);
		me.fastrx.bufs[write_buf_idx].len = 0;
		me.fastrx.write_buf_idx = write_buf_idx;
		nrf_uarte_rx_buffer_set(p_ins_board->p_nrf,
			me.fastrx.bufs[write_buf_idx].data, FASTRX_BUF_LEN);
		nrf_timer_task_trigger(RX_CNT_TIMER, NRF_TIMER_TASK_CLEAR);
		nrf_timer_task_trigger(RX_CNT_TIMER, NRF_TIMER_TASK_START);
		nrf_timer_task_trigger(RX_TMO_TIMER, NRF_TIMER_TASK_CLEAR);
		nrf_ppi_channel_enable(me.fastrx.ppi_ch_rxrdy1);
		nrf_ppi_channel_enable(me.fastrx.ppi_ch_rxrdy2);
		nrf_ppi_channel_enable(me.fastrx.ppi_ch_endrx);
		nrf_ppi_channel_enable(me.fastrx.ppi_ch_tmo);
		nrf_timer_event_clear(RX_TMO_TIMER,
			nrf_timer_compare_event_get(RX_TMO_COMP_CH));
		nrf_timer_int_enable(RX_TMO_TIMER,
			nrf_timer_compare_int_get(RX_TMO_COMP_CH));
		nrf_uarte_int_enable(p_ins_board->p_nrf, FASTRX_INTERRUPT_MASK);
		NRFX_IRQ_ENABLE(nrfx_get_irq_number(RX_TMO_TIMER));
	} else {
		nrf_uarte_rx_buffer_set(p_ins_board->p_nrf, p_ins_data->rx_buf,
			ONE_CHAR_LEN);
		nrf_uarte_int_enable(p_ins_board->p_nrf, RX_INTERRUPT_MASK);
	}

	nrf_uarte_enable(p_ins_board->p_nrf);
	nrf_uarte_task_trigger(p_ins_board->p_nrf, NRF_UARTE_TASK_STARTRX);

exit:
	util_mutex_unlock(p_ins_data->mutex_handle);
	return r;
}

err_code uarte_tx_disable(const NRF_UARTE_Type * const p_nrf)
{
	const struct uarte_instance *p_ins_board;
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;
	err_code r;

	r = ERROR_OK;

	if (me.p_board == NULL)
		return EUARTE_NO_INIT;

	instance_idx = uarte_nrf_to_idx(p_nrf);

	if ((instance_idx < 0) || (instance_idx >= me.p_board->num_instances))
		return EUARTE_INVALID_INSTANCE;

	p_ins_board = &me.p_board->p_ins[instance_idx];
	p_ins_data = &me.ins_data[instance_idx];

	if (!util_mutex_lock(p_ins_data->mutex_handle))
		return EUARTE_MUTEX;

	if (!p_ins_data->tx_enabled)
		goto exit;

	nrf_uarte_int_disable(p_ins_board->p_nrf, TX_INTERRUPT_MASK);
	clear_interrupt_mask(p_ins_board->p_nrf, TX_INTERRUPT_MASK);

	if (!p_ins_data->rx_enabled)
		disable_uarte_instance(p_ins_board);

	nrf_gpio_cfg_default(p_ins_board->uarte_pins.pin_tx);
	p_ins_data->tx_enabled = false;

exit:
	util_mutex_unlock(p_ins_data->mutex_handle);
	return r;
}

err_code uarte_tx_enable(const NRF_UARTE_Type * const p_nrf)
{
	const struct uarte_instance *p_ins_board;
	enum uarte_instance_idx instance_idx;
	struct instance_data *p_ins_data;
	err_code r;

	r = ERROR_OK;

	if (me.p_board == NULL)
		return EUARTE_NO_INIT;

	instance_idx = uarte_nrf_to_idx(p_nrf);

	if ((instance_idx < 0) || (instance_idx >= me.p_board->num_instances))
		return EUARTE_INVALID_INSTANCE;

	p_ins_board = &me.p_board->p_ins[instance_idx];
	p_ins_data = &me.ins_data[instance_idx];

	if (!util_mutex_lock(p_ins_data->mutex_handle))
		return EUARTE_MUTEX;

	if (p_ins_data->tx_enabled)
		goto exit;

	/* Common configuration for Rx/Tx, hence only needed if no prior act. */
	if (!p_ins_data->rx_enabled)
		enable_uarte_instance(p_ins_board);

	nrf_gpio_cfg_default(p_ins_board->uarte_pins.pin_tx);

	clear_interrupt_mask(p_ins_board->p_nrf, TX_INTERRUPT_MASK);
	nrf_uarte_int_enable(p_ins_board->p_nrf, TX_INTERRUPT_MASK);

	p_ins_data->tx_enabled = true;

exit:
	util_mutex_unlock(p_ins_data->mutex_handle);
	return r;
}

err_code uarte_init(const struct uarte_board * const p_board)
{
	const struct uarte_instance *p_ins_board;
	bool fastrx_in_use;
	err_code r;
	uint8_t i;

	if (me.p_board != NULL)
		return ERROR_OK;

	if (p_board == NULL)
		return EUARTE_INVALID_ARG;

	if (!p_board->num_instances || p_board->num_instances > NUM_UARTE_INSTANCES)
		return EUARTE_NUM_INSTANCE;

	fastrx_in_use = false;

	for (i = 0; i < p_board->num_instances; i++) {
		p_ins_board = &p_board->p_ins[i];

		if (p_ins_board->p_nrf != NRF_UARTE0 &&
				p_ins_board->p_nrf != NRF_UARTE1)
			return EUARTE_INVALID_INSTANCE;

		if (p_ins_board->use_fastrx && fastrx_in_use)
			return EUARTE_INVALID_ARG;

		r = util_validate_pins((uint8_t *)&p_ins_board->uarte_pins,
				sizeof(p_ins_board->uarte_pins));
		ERR_CHECK(r);

		if (p_ins_board->baud < UARTE_BAUDRATE_BAUDRATE_Baud1200 ||
				p_ins_board->baud > UARTE_BAUDRATE_BAUDRATE_Baud1M)
			return  EUARTE_INVALID_BAUD;

		nrf_gpio_cfg_default(p_ins_board->uarte_pins.pin_rx);
		nrf_gpio_cfg_default(p_ins_board->uarte_pins.pin_tx);

		if (p_ins_board->use_fastrx) {
			fastrx_in_use = true;
			me.ins_data[i].use_fastrx = true;

			r = alloc_ppi(&me.fastrx.ppi_ch_rxrdy1);
			ERR_CHECK(r);
			r = alloc_ppi(&me.fastrx.ppi_ch_rxrdy2);
			ERR_CHECK(r);
			r = alloc_ppi(&me.fastrx.ppi_ch_endrx);
			ERR_CHECK(r);
			r = alloc_ppi(&me.fastrx.ppi_ch_tmo);
			ERR_CHECK(r);

			/* Configure byte counter timer */
			nrf_timer_mode_set(RX_CNT_TIMER, NRF_TIMER_MODE_COUNTER);
			nrf_timer_bit_width_set(RX_CNT_TIMER, NRF_TIMER_BIT_WIDTH_32);

			/* Configure data timeout timer */
			nrf_timer_mode_set(RX_TMO_TIMER, NRF_TIMER_MODE_TIMER);
			nrf_timer_bit_width_set(RX_TMO_TIMER, NRF_TIMER_BIT_WIDTH_32);
			nrf_timer_frequency_set(RX_TMO_TIMER, NRF_TIMER_FREQ_1MHz);
			nrf_timer_cc_write(RX_TMO_TIMER, RX_TMO_COMP_CH,
				RX_TMO_TIME_US);
			NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(RX_TMO_TIMER),
			    IRQ_PRIORITY);

			/* Update byte counter for each received byte. */
			nrf_ppi_channel_endpoint_setup(me.fastrx.ppi_ch_rxrdy1,
				nrf_uarte_event_address_get(p_ins_board->p_nrf,
					NRF_UARTE_EVENT_RXDRDY),
					(uint32_t)nrf_timer_task_address_get(RX_CNT_TIMER,
						NRF_TIMER_TASK_COUNT));

			/* Reset timeout timer for each received byte. */
			nrf_ppi_channel_endpoint_setup(me.fastrx.ppi_ch_rxrdy2,
				nrf_uarte_event_address_get(p_ins_board->p_nrf,
					NRF_UARTE_EVENT_RXDRDY),
				(uint32_t)nrf_timer_task_address_get(RX_TMO_TIMER,
					NRF_TIMER_TASK_CLEAR));

			/* Start timeout timer when a byte is received. */
			nrf_ppi_fork_endpoint_setup(me.fastrx.ppi_ch_rxrdy2,
				(uint32_t)nrf_timer_task_address_get(RX_TMO_TIMER,
					NRF_TIMER_TASK_START));

			/* Capture byte count when a buffer is full. */
			nrf_ppi_channel_endpoint_setup(me.fastrx.ppi_ch_endrx,
				nrf_uarte_event_address_get(p_ins_board->p_nrf,
					NRF_UARTE_EVENT_ENDRX),
				(uint32_t)nrf_timer_task_address_get(RX_CNT_TIMER,
					nrf_timer_capture_task_get(RX_CNT_CAP_ENDRX_CH)));

			/* Restart RX when a buffer is full. */
			nrf_ppi_fork_endpoint_setup(me.fastrx.ppi_ch_endrx,
				nrf_uarte_task_address_get(p_ins_board->p_nrf,
					NRF_UARTE_TASK_STARTRX));

			/* Stop timer at timeout. */
			nrf_ppi_channel_endpoint_setup(me.fastrx.ppi_ch_tmo,
				(uint32_t)nrf_timer_event_address_get(RX_TMO_TIMER,
					nrf_timer_compare_event_get(RX_TMO_COMP_CH)),
				(uint32_t)nrf_timer_task_address_get(RX_TMO_TIMER,
					NRF_TIMER_TASK_STOP));

			/* Capture byte count at timeout. */
			nrf_ppi_fork_endpoint_setup(me.fastrx.ppi_ch_tmo,
				(uint32_t)nrf_timer_task_address_get(RX_CNT_TIMER,
					nrf_timer_capture_task_get(RX_CNT_CAP_TMO_CH)));
		}

		/* Configure the UART */
		nrf_uarte_txrx_pins_set(p_ins_board->p_nrf,
			p_ins_board->uarte_pins.pin_tx, p_ins_board->uarte_pins.pin_rx);
		nrf_uarte_baudrate_set(p_ins_board->p_nrf, p_ins_board->baud);
		nrf_uarte_configure(p_ins_board->p_nrf, UARTE_CONFIG_PARITY_Excluded,
			UARTE_CONFIG_HWFC_Disabled);
		NRFX_IRQ_PRIORITY_SET(nrf_drv_get_IRQn(p_ins_board->p_nrf),
			IRQ_PRIORITY);
	}

	me.ins_data[UARTE_INSTANCE_IDX_0].mutex_handle =
			CREATE_STATIC_MUTEX(UARTE0_MUTEX);
	me.ins_data[UARTE_INSTANCE_IDX_0].read_mutex_handle =
			CREATE_STATIC_MUTEX(UARTE0_READ_MUTEX);
	me.ins_data[UARTE_INSTANCE_IDX_0].queue_handle =
			CREATE_STATIC_QUEUE(UARTE0_RX);

	if (p_board->num_instances >= UARTE_INSTANCE_IDX_1) {
		me.ins_data[UARTE_INSTANCE_IDX_1].mutex_handle =
				CREATE_STATIC_MUTEX(UARTE1_MUTEX);
		me.ins_data[UARTE_INSTANCE_IDX_1].read_mutex_handle =
				CREATE_STATIC_MUTEX(UARTE1_READ_MUTEX);
		me.ins_data[UARTE_INSTANCE_IDX_1].queue_handle =
				CREATE_STATIC_QUEUE(UARTE1_RX);
	}

	me.p_board = p_board;

	return ERROR_OK;
}
