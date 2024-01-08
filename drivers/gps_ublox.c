#include <nrf_gpio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <string.h>

#include "drivers/gps_ublox.h"
#include "drivers/twim_spim_irq.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "freertos_tasktracker.h"
#include "gps.h"
#include "nmea_parser.h"
#include "persistent.h"
#include "util.h"

#define DRV_NAME							UBX
#define LOG_TAG								STRINGIFY(DRV_NAME)
#include "log.h"

/* TODO: Trim single extract size for NMEA messages */
#define TRX_BUF_SIZE						250
#define DATA_STREAM_EMPTY_BYTE				0xFF
#define BYTES_AVAILABLE_ADDR				0xFD
#define BYTES_AVAILABLE_SIZE				2
#define MAX_NBR_RETRY_ATTEMPTS				5
#define WRITE_CFG_RETRY_DELAY_MS			100

#define UBX_HDR_SIZE						2
#define UBX_CHECKSUM_SIZE					2
#define UBX_CLASS_OFFSET					2
#define UBX_ID_OFFSET						3
#define UBX_NACK_ID							0
#define UBX_ACK_ID							1

#define STARTUP_DELAY_MS					100
#define CFG_RESPOSE_TMO_MS					100
#define NMEA_RX_TMO_MS						10

#define SPI_DMA_MAX_LEN						255
#define SPI_COMPLETION_TMO_MS				150

#define TASK_NAME							DRV_NAME
#define TASK_PRIORITY						2
#define TASK_STACK_DEPTH					192
#define QUEUE_LEN							4

enum queue_item {
	QUEUE_ITEM_START_REQ = 0,
	QUEUE_ITEM_STOP_REQ,
	QUEUE_ITEM_SHUTDOWN_REQ,
	QUEUE_ITEM_NMEA_DATA_READY,
};

STATIC_TASK(TASK_NAME, TASK_STACK_DEPTH);
STATIC_QUEUE(TASK_NAME, QUEUE_LEN, enum queue_item);

static const uint8_t REQ_UBX_CFG_GNSS_M8[] = {
	0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00,
	0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00,
	0x01, 0x01,
};

static const uint8_t REQ_UBX_CFG_GNSS_8[] = {
	0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00,
	0x14, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00,
	0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00,
	0x01, 0x01,
};

static const uint8_t REQ_UBX_CFG_MSG_DISABLE_GGA[] = {
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00,
	0x00,
};

static const uint8_t REQ_UBX_CFG_MSG_DISABLE_GSV[] = {
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03,
	0x00,
};

static const uint8_t REQ_UBX_CFG_MSG_DISABLE_GLL[] = {
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01,
	0x00,
};

static const uint8_t REQ_UBX_CFG_MSG_DISABLE_VTG[] = {
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05,
	0x00,
};

static const uint8_t REQ_UBX_CFG_PRT_I2C_START[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00,
	0x37, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00,
};

static const uint8_t REQ_UBX_CFG_PRT_I2C_STOP[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00,
	0x37, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00,
};

static const uint8_t REQ_UBX_CFG_PRT_I2C_POLL[] = {
	0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x00,
};

static const uint8_t REQ_UBX_CFG_PRT_SPI_START[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x04, 0x00,
	0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00,
};

static const uint8_t REQ_UBX_CFG_PRT_SPI_STOP[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x04, 0x00,
	0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00,
};

static const uint8_t REQ_UBX_CFG_PRT_SPI_POLL[] = {
	0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x04,
};

static const uint8_t RSP_UBX_ACK[] = {
	0xB5, 0x62, 0x05,
};

struct ubx_ack_pkt {
	uint8_t		header[UBX_HDR_SIZE];
	uint8_t		class;
	uint8_t		id;
	uint16_t	size;
	uint8_t		cls_id;
	uint8_t		msg_id;
	uint8_t		ck_a;
	uint8_t		ck_b;
} __packed;

static struct {
	const struct gps_ublox_board	*p_board;
	TaskHandle_t					task_handle;
	TaskHandle_t					dry_notify_handle;
	TaskHandle_t					spi_notify_handle;
	QueueHandle_t					queue_handle;
	nmea_parser_t					nmea_data;
	uint8_t							trx_buf[TRX_BUF_SIZE];
	bool							gps_fixed;
} me;

__STATIC_INLINE void set_cs_active(void)
{
	nrf_gpio_pin_clear(me.p_board->spi.pins.spi_cs);
}

__STATIC_INLINE void set_cs_inactive(void)
{
	nrf_gpio_pin_set(me.p_board->spi.pins.spi_cs);
}

__STATIC_INLINE bool is_dry(void)
{
	return (!nrf_gpio_pin_read(me.p_board->gpio_irq));
}

static void gpio_irq_callback(uint8_t gpio, uint32_t value)
{
	enum queue_item queue_item;
	BaseType_t yield;

	yield = pdFALSE;

	/* In cases of having a waiting data-ready sequence, this event will
	 * overrule the queue handling.
	 */
	if (me.dry_notify_handle) {
		vTaskNotifyGiveFromISR(me.dry_notify_handle, &yield);
	} else if (me.queue_handle) {
		queue_item = QUEUE_ITEM_NMEA_DATA_READY;
		if (xQueueSendToBackFromISR(me.queue_handle, &queue_item, &yield)
				!= pdTRUE)
			LOGE("Dry irq queue full.");
	} else {
		return;
	}

	portYIELD_FROM_ISR(yield);
}

static void spim_irq_callback(void)
{
	BaseType_t yield;

	if (!nrf_spim_event_check(me.p_board->spi.p_instance, NRF_SPIM_EVENT_END))
		return;

	nrf_spim_event_clear(me.p_board->spi.p_instance, NRF_SPIM_EVENT_END);

	if (!me.spi_notify_handle)
		return;

	yield = pdFALSE;
	vTaskNotifyGiveFromISR(me.spi_notify_handle, &yield);
	portYIELD_FROM_ISR(yield);
}

static err_code spi_init(const struct gps_ublox_spi *const p_spi_cfg)
{
	err_code r;

	set_cs_inactive();
	nrf_gpio_cfg_output(p_spi_cfg->pins.spi_cs);

	nrf_gpio_cfg_output(p_spi_cfg->pins.spi_clk);
	nrf_gpio_cfg_output(p_spi_cfg->pins.spi_mosi);
	nrf_gpio_cfg_input(p_spi_cfg->pins.spi_miso, NRF_GPIO_PIN_NOPULL);

	nrf_spim_pins_set(p_spi_cfg->p_instance, p_spi_cfg->pins.spi_clk,
		p_spi_cfg->pins.spi_mosi, p_spi_cfg->pins.spi_miso);

	nrf_spim_frequency_set(p_spi_cfg->p_instance, p_spi_cfg->frequency);

	nrf_spim_configure(p_spi_cfg->p_instance, NRF_SPIM_MODE_0,
		NRF_SPIM_BIT_ORDER_MSB_FIRST);

	r = twim_spim_irq_register_spim(p_spi_cfg->p_instance, spim_irq_callback);
	ERR_CHECK(r);

	nrf_spim_int_enable(p_spi_cfg->p_instance, NRF_SPIM_INT_END_MASK);

	nrf_spim_enable(p_spi_cfg->p_instance);

	NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(p_spi_cfg->p_instance),
			APP_IRQ_PRIORITY_LOW);

	NRFX_IRQ_ENABLE(nrfx_get_irq_number(p_spi_cfg->p_instance));

	return ERROR_OK;
}

static err_code spi_transfer(const uint8_t *p_tx, uint8_t * const p_rx,
	const uint16_t trx_size)
{
	const uint8_t *p_tx_buf;
	uint16_t trx_bytes_left;
	uint8_t trx_chunk;
	uint8_t *p_rx_buf;
	err_code r;

	me.spi_notify_handle = xTaskGetCurrentTaskHandle();
	trx_bytes_left = trx_size;
	p_tx_buf = p_tx;
	p_rx_buf = p_rx;
	r = ERROR_OK;

	set_cs_active();

	nrf_spim_orc_set(me.p_board->spi.p_instance, 0xff);

	do {
		trx_chunk = MIN(SPI_DMA_MAX_LEN, trx_bytes_left);
		if (p_tx_buf)
			nrf_spim_tx_buffer_set(me.p_board->spi.p_instance, p_tx_buf,
				trx_chunk);
		else
			nrf_spim_tx_buffer_set(me.p_board->spi.p_instance, NULL, 0);

		if (p_rx_buf)
			nrf_spim_rx_buffer_set(me.p_board->spi.p_instance, p_rx_buf,
				trx_chunk);
		else
			nrf_spim_rx_buffer_set(me.p_board->spi.p_instance, NULL, 0);

		nrf_spim_event_clear(me.p_board->spi.p_instance, NRF_SPIM_EVENT_END);
		nrf_spim_task_trigger(me.p_board->spi.p_instance, NRF_SPIM_TASK_START);

		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
				pdMS_TO_TICKS(SPI_COMPLETION_TMO_MS)) != pdTRUE) {
			nrf_spim_task_trigger(me.p_board->spi.p_instance,
				NRF_SPIM_TASK_STOP);
			r = EGPS_UBLOX_SPI_TMO;
			break;
		}

		if (p_tx_buf)
			p_tx_buf += trx_chunk;
		if (p_rx_buf)
			p_rx_buf += trx_chunk;
		trx_bytes_left -= trx_chunk;
	} while (trx_bytes_left > 0);

	set_cs_inactive();

	me.spi_notify_handle = NULL;
	return r;
}

static err_code event_handler(const struct eventpump_param *const p_event)
{
	enum queue_item queue_item;

	if (!me.queue_handle)
		return ERROR_OK;

	if (p_event->gps.event == GPS_START_REQ)
		queue_item = QUEUE_ITEM_START_REQ;
	else if (p_event->gps.event == GPS_STOP_REQ)
		queue_item = QUEUE_ITEM_STOP_REQ;
	else
		return ERROR_OK;

	if (xQueueSendToBack(me.queue_handle, &queue_item,
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdTRUE) {
		session_mgr_append_err_code(EGPS_UBLOX_QUEUE_FULL, __func__);
		return EGPS_UBLOX_QUEUE_FULL;
	}

	return ERROR_OK;
}
REGISTER_EVENT_HANDLER(EVENT_GPS, event_handler);

static void flush_stream(void)
{
	uint8_t data;
	err_code r;

	do {
		if (me.p_board->interface == GPS_UBLOX_IF_I2C)
			r = twim_client_read(&me.p_board->twim_client, &data, sizeof(data));
		else if (me.p_board->interface == GPS_UBLOX_IF_SPI)
			r = spi_transfer(NULL, &data, sizeof(data));
		else
			return;
	} while ((r == ERROR_OK) && (data != DATA_STREAM_EMPTY_BYTE));
}

static err_code spi_read_stream(uint8_t *p_buf, uint16_t size,
		uint16_t * const p_stream_size)
{
	uint16_t nbr_stream_empty_bytes;
	uint16_t read_idx;
	err_code r;

	if (!p_buf || !size || !p_stream_size)
		return EGPS_UBLOX_INVALID_ARG;

	r = ERROR_OK;
	nbr_stream_empty_bytes = 0;

	/* The interrupt is cleared when all data has been loaded to the
	 * internal SPI block in the GPS module (that is, the irq pin can be set
	 * to inactive when there is 0 - 4 bytes left to clock out).
	 *
	 * Despite of the irq gpio, when the stream is empty 0xFF will be
	 * clocked-out.
	 * So to read the full data stream, one byte at a time is read until the
	 * irq is inactive and 4 0xFF:s has been read.
	 */
	for (read_idx = 0; read_idx < size; read_idx++) {
		r = spi_transfer(NULL, &p_buf[read_idx], 1);
		ERR_CHECK_GOTO(r, exit);

		if (is_dry())
			continue;

		if (p_buf[read_idx] == DATA_STREAM_EMPTY_BYTE)
			nbr_stream_empty_bytes++;
		else
			nbr_stream_empty_bytes = 0;

		if (nbr_stream_empty_bytes >= 4)
			break;
	}

exit:
	*p_stream_size = read_idx;

	return r;
}

static err_code i2c_read_stream(uint8_t *p_buf, uint16_t size,
		uint16_t * const p_stream_size)
{
	uint16_t stream_size;
	uint8_t read_addr;
	uint8_t i;
	err_code r;

	if (!p_buf || !size || !p_stream_size)
		return EGPS_UBLOX_INVALID_ARG;

	r = ERROR_OK;
	read_addr = BYTES_AVAILABLE_ADDR;
	*p_stream_size = 0;

	while (is_dry()) {
		for (i = 0; i <= MAX_NBR_RETRY_ATTEMPTS; i++) {
			if (i == MAX_NBR_RETRY_ATTEMPTS)
				return EGPS_UBLOX_STREAM_SIZE;

			r = twim_client_indexed_read(&me.p_board->twim_client,
					&read_addr, sizeof(read_addr),
					(uint8_t *)&stream_size, sizeof(stream_size));
			if (r == ERROR_OK) {
				stream_size = __ntohs(stream_size);
				break;
			}
		}

		if (stream_size == 0)
			return ERROR_OK;

		if (stream_size > size)
			return EGPS_UBLOX_READ_BUF_OVERFLOW;

		r = twim_client_read(&me.p_board->twim_client, p_buf, stream_size);
		ERR_CHECK(r);

		size -= stream_size;
		*p_stream_size += stream_size;
		if (!size)
			return r;
		p_buf += stream_size;
	}

	return r;
}

static err_code read_stream(uint8_t * const p_buf, const uint16_t size,
		uint16_t * const p_stream_size)
{
	uint16_t tot_stream_size;
	uint16_t stream_size;
	err_code r;

	if (!p_buf || !size || !p_stream_size)
		return EGPS_UBLOX_INVALID_ARG;

	tot_stream_size = 0;

	/* The periodic nmea position messages consist of 3 separate nmea messages.
	 * To make sure that all 3 messages are read (also if they are sent with
	 * intermediate delays), we will register ourselves for the dry interrupt
	 * and wait potential stream-sequence-packets to appear.
	 */
	me.dry_notify_handle = xTaskGetCurrentTaskHandle();

	do {
		if (me.p_board->interface == GPS_UBLOX_IF_I2C)
			r = i2c_read_stream(&p_buf[tot_stream_size],
				(size - tot_stream_size), &stream_size);
		else if (me.p_board->interface == GPS_UBLOX_IF_SPI)
			r = spi_read_stream(&p_buf[tot_stream_size],
				(size - tot_stream_size), &stream_size);
		else
			r = EGPS_UBLOX_INVALID_INTERFACE;

		ERR_CHECK_GOTO(r, exit);

		tot_stream_size += stream_size;
	} while (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(NMEA_RX_TMO_MS)) == pdTRUE);

exit:
	me.dry_notify_handle = NULL;
	*p_stream_size = tot_stream_size;

	return r;
}

static uint16_t calc_checksum(const uint8_t * const p_buf, const uint16_t size)
{
	uint8_t checksum_a;
	uint8_t checksum_b;
	uint16_t i;

	checksum_a = 0;
	checksum_b = 0;

	for (i = UBX_HDR_SIZE; i < size; i++) {
		checksum_a += p_buf[i];
		checksum_b += checksum_a;
	}

	return ((checksum_b << 8) | checksum_a);
}

static err_code write_stream(const uint8_t *p_buf, uint16_t size)
{
	uint16_t checksum;
	err_code r;

	if (!p_buf || !size)
		return EGPS_UBLOX_INVALID_ARG;

	if ((size + UBX_CHECKSUM_SIZE) > sizeof(me.trx_buf))
		return EGPS_UBLOX_WRITE_BUF_OVERFLOW;

	memcpy(me.trx_buf, p_buf, size);
	checksum = calc_checksum(me.trx_buf, size);
	me.trx_buf[size++] = (checksum & 0xff);
	me.trx_buf[size++] = (checksum >> 8);

	me.dry_notify_handle = xTaskGetCurrentTaskHandle();

	if (me.p_board->interface == GPS_UBLOX_IF_I2C)
		r = twim_client_write(&me.p_board->twim_client, me.trx_buf, size);
	else if (me.p_board->interface == GPS_UBLOX_IF_SPI)
		r = spi_transfer(me.trx_buf, NULL, size);
	else
		r = EGPS_UBLOX_INVALID_INTERFACE;

	ERR_CHECK_GOTO(r, exit);

	if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(CFG_RESPOSE_TMO_MS)) != pdTRUE)
		r = EGPS_UBLOX_NO_RESPONSE;

exit:
	me.dry_notify_handle = NULL;

	return r;
}

static err_code process_ubx_ack(const uint8_t *p_buf, const uint32_t size,
		const uint8_t cls_id, const uint8_t msg_id)
{
	struct ubx_ack_pkt *p_pkt;
	uint32_t i;

	for (i = 0; i < size; i++) {
		if ((size - i) < sizeof(struct ubx_ack_pkt))
			break;

		if (memcmp(&p_buf[i], RSP_UBX_ACK, sizeof(RSP_UBX_ACK)) == 0) {
			p_pkt = (struct ubx_ack_pkt *)&p_buf[i];
			if ((p_pkt->cls_id != cls_id) || (p_pkt->msg_id != msg_id))
				continue;

			if (p_pkt->id == UBX_ACK_ID)
				return ERROR_OK;

			if (p_pkt->id == UBX_NACK_ID)
				return EGPS_UBLOX_UBX_NACK;
		}
	}

	return EGPS_UBLOX_UBX_ERROR_RSP;
}

static err_code check_ubx_rsp(const uint8_t * const p_buf, const uint32_t size,
		const uint8_t * const p_expected_rsp, const uint32_t expected_size)
{
	uint16_t expected_checksum;
	uint32_t i;

	if ((expected_size + UBX_CHECKSUM_SIZE) > size)
		return EGPS_UBLOX_UBX_ERROR_RSP;

	expected_checksum = calc_checksum(p_expected_rsp, expected_size);

	for (i = 0; i <= (size - expected_size - UBX_CHECKSUM_SIZE); i++) {
		if ((memcmp(&p_buf[i], p_expected_rsp, expected_size) == 0) &&
				(*(uint16_t *)&p_buf[i + expected_size] == expected_checksum))
			return ERROR_OK;
	}

	return EGPS_UBLOX_UBX_ERROR_RSP;
}

static err_code write_cfg(const uint8_t *const p_buf, const uint32_t size)
{
	uint16_t stream_size;
	err_code r;
	uint8_t i;

	if (!p_buf)
		return EGPS_UBLOX_INVALID_ARG;

	if (!size)
		return ERROR_OK;

	r = ERROR_OK;

	for (i = 0; i < MAX_NBR_RETRY_ATTEMPTS; i++) {
		if (r != ERROR_OK) {
			vTaskDelay(pdMS_TO_TICKS(WRITE_CFG_RETRY_DELAY_MS));
			flush_stream();
		}

		r = write_stream(p_buf, size);
		if (r != ERROR_OK)
			continue;

		r = read_stream(&me.trx_buf[0], sizeof(me.trx_buf), &stream_size);
		if (r != ERROR_OK)
			continue;

		r = process_ubx_ack(me.trx_buf, stream_size, p_buf[UBX_CLASS_OFFSET],
			p_buf[UBX_ID_OFFSET]);
		if (r == ERROR_OK)
			break;
	}

	/* Explicitly log that the configuration failed. Return underlying error. */
	if (i >= MAX_NBR_RETRY_ATTEMPTS)
		session_mgr_append_err_code(EGPS_UBLOX_WRITE_CFG, __func__);

	return r;
}

static err_code write_cfg_prt(const uint8_t *const p_buf, const uint32_t size)
{
	uint16_t stream_size;
	err_code r;
	uint8_t i;

	if (!p_buf)
		return EGPS_UBLOX_INVALID_ARG;

	if (!size)
		return ERROR_OK;

	if ((size + UBX_CHECKSUM_SIZE) > sizeof(me.trx_buf))
		return EGPS_UBLOX_INVALID_ARG;

	r = ERROR_OK;

	for (i = 0; i < MAX_NBR_RETRY_ATTEMPTS; i++) {
		if (r != ERROR_OK) {
			vTaskDelay(pdMS_TO_TICKS(WRITE_CFG_RETRY_DELAY_MS));
			flush_stream();
		}

		r = write_stream(p_buf, size);
		if (r != ERROR_OK)
			continue;

		/* For some of the GPS modules we do not get any data at the MISO after
		 * a UBX_CFG_PRT cmd has been set.
		 * The workaround to trigger the GPS model to start sending data at MISO
		 * is to poll UBX_CFG_PRT after it has been set.
		 */
		if (me.p_board->interface == GPS_UBLOX_IF_I2C)
			r = write_stream(REQ_UBX_CFG_PRT_I2C_POLL,
				sizeof(REQ_UBX_CFG_PRT_I2C_POLL));
		else if (me.p_board->interface == GPS_UBLOX_IF_SPI)
			r = write_stream(REQ_UBX_CFG_PRT_SPI_POLL,
				sizeof(REQ_UBX_CFG_PRT_SPI_POLL));
		else
			return EGPS_UBLOX_INVALID_INTERFACE;

		/* The data ready interrupt is still active from the first UBX_CFG_PRT
		 * cmd, so we will get a EGPS_UBLOX_NO_RESPONSE from second write.
		 */
		if ((r != ERROR_OK) && (r != EGPS_UBLOX_NO_RESPONSE))
			continue;

		r = read_stream(&me.trx_buf[0], sizeof(me.trx_buf), &stream_size);
		if (r != ERROR_OK)
			continue;

		r = check_ubx_rsp(&me.trx_buf[0], stream_size, p_buf, size);
		if (r != ERROR_OK)
			continue;

		r = process_ubx_ack(&me.trx_buf[0], stream_size,
			p_buf[UBX_CLASS_OFFSET], p_buf[UBX_ID_OFFSET]);
		if (r == ERROR_OK)
			break;
	}

	/* Explicitly log that the configuration failed. Return underlying error. */
	if (i >= MAX_NBR_RETRY_ATTEMPTS)
		session_mgr_append_err_code(EGPS_UBLOX_WRITE_CFG, __func__);

	return r;
}

static err_code write_if_start(enum gps_ublox_if interface)
{
	if (interface == GPS_UBLOX_IF_I2C)
		return write_cfg_prt(REQ_UBX_CFG_PRT_I2C_START,
			sizeof(REQ_UBX_CFG_PRT_I2C_START));
	else if (me.p_board->interface == GPS_UBLOX_IF_SPI)
		return write_cfg_prt(REQ_UBX_CFG_PRT_SPI_START,
			sizeof(REQ_UBX_CFG_PRT_SPI_START));
	else
		return EGPS_UBLOX_INVALID_INTERFACE;
}

static err_code write_if_stop(enum gps_ublox_if interface)
{
	if (interface == GPS_UBLOX_IF_I2C)
		return write_cfg_prt(REQ_UBX_CFG_PRT_I2C_STOP,
			sizeof(REQ_UBX_CFG_PRT_I2C_STOP));
	else if (me.p_board->interface == GPS_UBLOX_IF_SPI)
		return write_cfg_prt(REQ_UBX_CFG_PRT_SPI_STOP,
			sizeof(REQ_UBX_CFG_PRT_SPI_STOP));
	else
		return EGPS_UBLOX_INVALID_INTERFACE;
}

static void task(void *arg)
{
	struct eventpump_param param;
	enum queue_item queue_item;
	uint16_t stream_size;
	err_code r;

	r = ERROR_OK;

	vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
	flush_stream();

	r = write_if_stop(me.p_board->interface);
	ERR_CHECK_GOTO(r, error_exit);

	r = write_cfg(REQ_UBX_CFG_MSG_DISABLE_GSV,
		sizeof(REQ_UBX_CFG_MSG_DISABLE_GSV));
	ERR_CHECK_GOTO(r, error_exit);

	r = write_cfg(REQ_UBX_CFG_MSG_DISABLE_GGA,
		sizeof(REQ_UBX_CFG_MSG_DISABLE_GGA));
	ERR_CHECK_GOTO(r, error_exit);

	r = write_cfg(REQ_UBX_CFG_MSG_DISABLE_GLL,
		sizeof(REQ_UBX_CFG_MSG_DISABLE_GLL));
	ERR_CHECK_GOTO(r, error_exit);

	r = write_cfg(REQ_UBX_CFG_MSG_DISABLE_VTG,
		sizeof(REQ_UBX_CFG_MSG_DISABLE_VTG));
	ERR_CHECK_GOTO(r, error_exit);

	switch (me.p_board->variant) {
	case GPS_UBLOX_VARIANT_8:
		r = write_cfg(REQ_UBX_CFG_GNSS_8, sizeof(REQ_UBX_CFG_GNSS_8));
		break;
	case GPS_UBLOX_VARIANT_M8:
		r = write_cfg(REQ_UBX_CFG_GNSS_M8, sizeof(REQ_UBX_CFG_GNSS_M8));
		break;
	default:
		r = EGPS_UBLOX_INVALID_VARIANT;
	}
	ERR_CHECK_GOTO(r, error_exit);

	param.source = EVENT_GPS;

	while (1) {
		if (r != ERROR_OK)
			session_mgr_append_err_code(r, __func__);

		if (xQueueReceive(me.queue_handle, &queue_item,
				portMAX_DELAY) != pdTRUE)
			continue; /* Should never happen. */

		if (queue_item == QUEUE_ITEM_START_REQ) {
			flush_stream();

			r = write_if_start(me.p_board->interface);
			if (r != ERROR_OK) {
				LOGE("Start r=0x%08lX", r);
				continue;
			}

			param.gps.event = GPS_STARTED;
			r = eventpump_post(&param);
			if (r != ERROR_OK)
				LOGE("GPS_STARTED r=0x%08lX", r);

			continue;
		}

		if (queue_item == QUEUE_ITEM_STOP_REQ) {
			flush_stream();
			r = write_cfg(REQ_UBX_CFG_PRT_I2C_STOP,
					sizeof(REQ_UBX_CFG_PRT_I2C_STOP));
			if (r != ERROR_OK) {
				LOGE("Stop r=0x%08lX", r);
				continue;
			}

			param.gps.event = GPS_STOPPED;
			r = eventpump_post(&param);
			if (r != ERROR_OK)
				LOGE("GPS_STOPPED r=0x%08lX", r);

			continue;
		}

		if (queue_item != QUEUE_ITEM_NMEA_DATA_READY)
			continue;

		r = read_stream(&me.trx_buf[0], sizeof(me.trx_buf), &stream_size);
		if (r != ERROR_OK) {
			flush_stream();
			LOGE("read_stream r=0x%08lX", r);
			continue;
		}

		if (!nmea_parser_process(&me.nmea_data, me.trx_buf, stream_size)) {
			LOGE("nmea_parser");
			continue;
		}

		if (!me.nmea_data.is_valid) {
			if (!me.gps_fixed)
				continue;

			param.gps.event = GPS_NO_SIGNAL;
			r = eventpump_post(&param);
			if (r != ERROR_OK)
				LOGE("LOST eventpump r=0x%08lX", r);
			else
				me.gps_fixed = false;
			continue;
		}

		if (!me.gps_fixed) {
			param.gps.event = GPS_FIX;
			param.gps.time.year = me.nmea_data.year;
			param.gps.time.month = me.nmea_data.month;
			param.gps.time.day = me.nmea_data.date;
			param.gps.time.hour = me.nmea_data.hours;
			param.gps.time.minute = me.nmea_data.minutes;
			param.gps.time.second = me.nmea_data.seconds;

			r = eventpump_post(&param);
			if (r != ERROR_OK)
				LOGE("FIX eventpump r=0x%08lX", r);
			else
				me.gps_fixed = true;
			continue;
		}

		if (!me.nmea_data.latitude && !me.nmea_data.longitude) {
			LOGE("Zero pos");
			continue;
		}

		if ((me.nmea_data.latitude == GPS_INVALID_POS_VAL) &&
				(me.nmea_data.longitude == GPS_INVALID_POS_VAL)) {
			LOGE("Invalid pos");
			continue;
		}

		param.gps.event = GPS_PARAMS;
		param.gps.params.latitude = me.nmea_data.latitude;
		param.gps.params.longitude = me.nmea_data.longitude;
		param.gps.params.speed = me.nmea_data.speed;
		param.gps.params.hdop = me.nmea_data.dop_h;

		r = eventpump_post(&param);
		if (r != ERROR_OK)
			LOGE("PARAMS eventpump r=0x%08lX", r);
	}

error_exit:
	LOGE("%s: r=0x%08lX", __func__, r);
	me.p_board = NULL;
	while (true)
		vTaskDelay(portMAX_DELAY);
}

err_code gps_ublox_init(const struct gps_ublox_board *const p_board)
{
	err_code r;

	if (me.p_board)
		return ERROR_OK;

	if (p_board == NULL)
		return EGPS_UBLOX_PDATA;

	if (p_board->variant >= NUM_GPS_UBLOX_VARIANTS)
		return EGPS_UBLOX_INVALID_VARIANT;

	r = util_validate_pins(&p_board->gpio_irq, sizeof(p_board->gpio_irq));
	ERR_CHECK(r);

	if ((p_board->irq_pol != GPIO_IRQ_POL_HIGH) &&
			(p_board->irq_pol != GPIO_IRQ_POL_LOW))
		return EGPS_UBLOX_INVALID_ARG;

	switch (p_board->interface) {
	case GPS_UBLOX_IF_I2C:
		r = twim_client_reg(&p_board->twim_client);
		break;
	case GPS_UBLOX_IF_SPI:
		if (p_board->spi.p_instance == NULL)
			return EGPS_UBLOX_PDATA;

		r = util_validate_pins((uint8_t *)&p_board->spi.pins,
			sizeof(p_board->spi.pins));
		ERR_CHECK(r);

		r = spi_init(&p_board->spi);
		break;
	default:
		return EGPS_UBLOX_INVALID_INTERFACE;
	}
	ERR_CHECK(r);

	r = gpio_irq_register(p_board->gpio_irq, p_board->irq_pol,
			gpio_irq_callback);
	ERR_CHECK(r);

	nrf_gpio_cfg_input(p_board->gpio_irq, NRF_GPIO_PIN_NOPULL);

	me.p_board = p_board;

	me.queue_handle = CREATE_STATIC_QUEUE(TASK_NAME);
	me.task_handle = CREATE_STATIC_TASK(task, TASK_NAME, TASK_PRIORITY);

	return tasktracker_register_task(me.task_handle);
}
