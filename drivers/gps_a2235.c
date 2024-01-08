#include <app_util_platform.h>
#include <nrf_gpio.h>
#include <nrf_spim.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "drivers/gps_a2235.h"
#include "drivers/twim_spim_irq.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "freertos_tasktracker.h"
#include "gps.h"
#include "nmea_parser.h"
#include "persistent.h"
#include "session_mgr.h"
#include "util.h"

#define DRV_NAME						A2235
#define LOG_TAG							STRINGIFY(DRV_NAME)
#include "log.h"

#define NULL_TERMINATOR_SIZE			1
#define SPI_TRANSF_TMO_MS				10
#define ENABLE_RETRY_DELAY_MS			100
#define MAX_NBR_OF_ENABLE_RETRIES		5
#define NMEA_PACKET_FW_VERS_OFFSET		9
/* TODO: Add this as p_board, or as a remote config parameter */
#define EXPECTED_FW_VERSION_STRING		"GSD4e_4.1.2"
#define EXPECTED_FW_VERSION_LEN			sizeof(EXPECTED_FW_VERSION_STRING)
#define SW_VERSION_RSP_DELAY_MS			30
#define SW_VERSION_RSP_ADD_ON_DELAY_MS	10
#define SW_VERSION_RETRY_DELAY_MS		50
#define MAX_NBR_OF_READ_SW_VERS_RETRIES	10

#define GPS_ON_OFF_PULSE_MS				200
#define GPS_RESET_PULSE_MS				10
#define GPS_WAKE_UP_DELAY_MS			1000
#define GPS_COLD_START_DELAY_MS			150
#define WAKEUP_TMO_MS					100
#define WAKEUP_MAX_PULSE_WIDTH_MS		3
#define POLL_INTERVAL_MS				200

#define NMEA_HEADER_SIZE				8
#define NMEA_HEADER_START_SIZE			1
#define NMEA_FOOTER_SIZE				5
#define NMEA_CMD_MSG_MAX_SIZE			128
#define NMEA_PAYLOAD_MAX_SIZE			(NMEA_CMD_MSG_MAX_SIZE - \
										(NMEA_HEADER_SIZE + \
										NMEA_FOOTER_SIZE + \
										NULL_TERMINATOR_SIZE))
#define NMEA_HEADER_BUF_SIZE			(NMEA_HEADER_SIZE + \
										NULL_TERMINATOR_SIZE)
#define NMEA_ENCODE_PAYLOAD(...)		snprintf(((char *)me.trx_buf + \
										NMEA_HEADER_SIZE), \
										(NMEA_PAYLOAD_MAX_SIZE + \
										NULL_TERMINATOR_SIZE), __VA_ARGS__);
#define NMEA_FW_VERS_RESP_LEN			(NMEA_PACKET_FW_VERS_OFFSET + \
										EXPECTED_FW_VERSION_LEN)
#define NMEA_OUTPUT_RATE_OFF			0
#define NMEA_OK_TO_SEND_SIZE			15
/* TODO: Trim single extract size for NMEA messages */
#define NMEA_SINGLE_EXTRACT_SIZE		192
#define TRX_BUF_SIZE					256

#define TASK_NAME						DRV_NAME
#define TASK_PRIORITY					2
#define TASK_STACK_DEPTH				256
#define QUEUE_LEN						4

STATIC_ASSERT_MSG((NMEA_SINGLE_EXTRACT_SIZE <= TRX_BUF_SIZE),
	"trx_buf too small.");

enum queue_item {
	QUEUE_ITEM_START_REQ = 0,
	QUEUE_ITEM_STOP_REQ,
	QUEUE_ITEM_SHUTDOWN_REQ,
	QUEUE_ITEM_WAKEUP_OFF,
	NBR_OF_QUEUE_ITEMS,
};

STATIC_TASK(TASK_NAME, TASK_STACK_DEPTH);
STATIC_QUEUE(TASK_NAME, QUEUE_LEN, enum queue_item);
STATIC_MUTEX(TASK_NAME);

enum nmea_msg_id {
	NMEA_MSG_ID_RATE_CONTROL = 103,
};

enum nmea_msg {
	NMEA_MSG_GGA = 0,
	NMEA_MSG_GLL,
	NMEA_MSG_GSA,
	NMEA_MSG_GSV,
	NMEA_MSG_RMC,
	NMEA_MSG_VTG,
	NMEA_MSG_MSS,
	NMEA_MSG_NOT_SUPPORTED,
	NMEA_MSG_ZDA,
	NUM_OF_NMEA_MSGS
};

enum nmea_mode {
	NMEA_MODE_SET_RATE = 0,
	NMEA_MODE_QUERY,
	NMEA_MODE_ABP_ON,
	NMEA_MODE_ABP_OFF,
	NMEA_MODE_REVERSE_EE_ON,
	NMEA_MODE_REVERSE_EE_OFF,
	NMEA_MODE_5HZ_ON,
	NMEA_MODE_5HZ_OFF,
	NMEA_MODE_SBAS_RANGING_ON,
	NMEA_MODE_SBAS_RANGING_OFF,
	NMEA_MODE_FTS_ON,
	NMEA_MODE_FTS_OFF,
	NUM_OF_NMEA_MODES,
};

static const char nmea_cmd_cold_start[] = "$PSRF101,0,0,0,0,0,0,12,4*10\r\n";
static const char nmea_cmd_read_sw_vers[] = "$PSRF125*21\r\n";
static const char nmea_rsp_read_sw_vers[] = "$PSRF195";
static const char nmea_rsp_ok_to_send[] = "$PSRF150";

static struct {
	const struct gps_a2235_board	*p_board;
	SemaphoreHandle_t				mutex_handle;
	TaskHandle_t					task_handle;
	TaskHandle_t					notify_spi_handle;
	TaskHandle_t					notify_wakeup_handle;
	QueueHandle_t					queue_handle;
	nmea_parser_t					nmea_data;
	uint8_t							trx_buf[TRX_BUF_SIZE];
	char							nmea_header[NMEA_HEADER_BUF_SIZE];
	bool							gps_fixed;
	bool							spi_enabled;
	bool							started;
} me;

__STATIC_INLINE void send_reset_pulse(void)
{
	nrf_gpio_pin_clear(me.p_board->pins.rst);
	vTaskDelay(pdMS_TO_TICKS(GPS_RESET_PULSE_MS));
	nrf_gpio_pin_set(me.p_board->pins.rst);
}

__STATIC_INLINE void send_on_off_pulse(void)
{
	nrf_gpio_pin_set(me.p_board->pins.on);
	vTaskDelay(pdMS_TO_TICKS(GPS_ON_OFF_PULSE_MS));
	nrf_gpio_pin_clear(me.p_board->pins.on);
}

__STATIC_INLINE void set_cs_active(void)
{
	nrf_gpio_pin_clear(me.p_board->pins.spi_cs);
}

__STATIC_INLINE void set_cs_inactive(void)
{
	nrf_gpio_pin_set(me.p_board->pins.spi_cs);
}

__STATIC_INLINE bool chip_is_active(void)
{
	return !!nrf_gpio_pin_read(me.p_board->pins.wakeup);
}

static void spim_irq_callback(void)
{
	BaseType_t yield_req = pdFALSE;

	if (nrf_spim_event_check(me.p_board->p_instance, NRF_SPIM_EVENT_END)) {
		nrf_spim_event_clear(me.p_board->p_instance, NRF_SPIM_EVENT_END);
		if (me.notify_spi_handle != NULL)
			vTaskNotifyGiveFromISR(me.notify_spi_handle, &yield_req);
	}

	portYIELD_FROM_ISR(yield_req);
}

static void wakeup_irq_callback(uint8_t gpio, uint32_t value)
{
	static TickType_t last_toggle_tick;
	enum queue_item queue_item;
	TickType_t pulse_width;
	BaseType_t yield_req;

	pulse_width = (xTaskGetTickCount() - last_toggle_tick);
	last_toggle_tick = xTaskGetTickCount();

	yield_req = pdFALSE;

	if (me.notify_wakeup_handle) {
		if (value)
			vTaskNotifyGiveFromISR(me.notify_wakeup_handle, &yield_req);
	} else if (!value &&
			(pulse_width > pdMS_TO_TICKS(WAKEUP_MAX_PULSE_WIDTH_MS))) {
		queue_item = QUEUE_ITEM_WAKEUP_OFF;
		if (xQueueSendToBackFromISR(me.queue_handle, &queue_item,
				&yield_req) != pdTRUE)
			LOGE("Failed to put wakeup_off to queue");
	}

	portYIELD_FROM_ISR(yield_req);
}

static err_code event_handler(const struct eventpump_param *const p_event)
{
	enum queue_item queue_item;
	err_code r;

	if (!me.p_board || !me.queue_handle)
		return ERROR_OK;

	r = ERROR_OK;

	switch (p_event->source) {
	case EVENT_POWER:
		if (p_event->power.event != POWER_OFF)
			break;

		queue_item = QUEUE_ITEM_SHUTDOWN_REQ;
		if (xQueueSendToFront(me.queue_handle, &queue_item,
				pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdTRUE)
			r = EA2235_QUEUE_FULL;
		break;
	case EVENT_GPS:
		if (p_event->gps.event == GPS_START_REQ)
			queue_item = QUEUE_ITEM_START_REQ;
		else if (p_event->gps.event == GPS_STOP_REQ)
			queue_item = QUEUE_ITEM_STOP_REQ;
		else
			break;

		if (xQueueSendToBack(me.queue_handle, &queue_item,
				pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdTRUE)
			r = EA2235_QUEUE_FULL;
		break;
	default:
		break;
	}

	if (r != ERROR_OK)
		session_mgr_append_err_code(r, __func__);

	return r;
}
REGISTER_EVENT_HANDLER(EVENT_GPS, event_handler);
REGISTER_EVENT_HANDLER(EVENT_POWER, event_handler);

static err_code spi_transfer(const uint8_t *p_tx, const uint16_t tx_size,
		uint8_t *p_rx, uint16_t rx_size)
{
	err_code r = ERROR_OK;

	if (!me.spi_enabled)
		return EA2235_SPI_INIT;

	if (p_tx && p_rx)
		return EA2235_SPI_TRX;

	if ((p_tx && !tx_size) || (p_tx == NULL && tx_size) ||
			(p_rx && !rx_size) || (p_rx == NULL && rx_size))
		return EA2235_SPI_TRX_INVALID_BUF;

	if ((p_rx && !nrfx_is_in_ram(p_rx)) || (p_tx && !nrfx_is_in_ram(p_tx)))
		return EA2235_SPI_RAM_BUF;

	nrf_spim_tx_buffer_set(me.p_board->p_instance, p_tx, tx_size);
	nrf_spim_rx_buffer_set(me.p_board->p_instance, p_rx, rx_size);

	nrf_spim_event_clear(me.p_board->p_instance, NRF_SPIM_EVENT_END);
	me.notify_spi_handle = xTaskGetCurrentTaskHandle();

	set_cs_active();

	nrf_spim_task_trigger(me.p_board->p_instance, NRF_SPIM_TASK_START);

	if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(SPI_TRANSF_TMO_MS)) != pdTRUE) {
		nrf_spim_task_trigger(me.p_board->p_instance, NRF_SPIM_TASK_STOP);
		r = EA2235_SPI_TMO;
	}

	me.notify_spi_handle = NULL;

	set_cs_inactive();

	return r;
}

static err_code spi_setup(void)
{
	err_code r;

	if (me.spi_enabled)
		return ERROR_OK;

	/* SPI Mode 1 */
	nrf_gpio_pin_clear(me.p_board->pins.spi_clk);
	nrf_gpio_cfg_output(me.p_board->pins.spi_clk);

	nrf_gpio_pin_clear(me.p_board->pins.spi_mosi);
	nrf_gpio_cfg_output(me.p_board->pins.spi_mosi);

	nrf_gpio_cfg_input(me.p_board->pins.spi_miso, NRF_GPIO_PIN_PULLDOWN);

	set_cs_inactive();
	nrf_gpio_cfg_output(me.p_board->pins.spi_cs);

	nrf_spim_pins_set(me.p_board->p_instance, me.p_board->pins.spi_clk,
						me.p_board->pins.spi_mosi, me.p_board->pins.spi_miso);
	nrf_spim_frequency_set(me.p_board->p_instance, me.p_board->frequency);
	nrf_spim_configure(me.p_board->p_instance, NRF_SPIM_MODE_1,
		NRF_SPIM_BIT_ORDER_MSB_FIRST);

	r = twim_spim_irq_register_spim(me.p_board->p_instance, spim_irq_callback);
	ERR_CHECK(r);

	nrf_spim_int_enable(me.p_board->p_instance, NRF_SPIM_INT_END_MASK);
	nrf_spim_enable(me.p_board->p_instance);

	NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(me.p_board->p_instance),
		APP_IRQ_PRIORITY_LOW);
	NRFX_IRQ_ENABLE(nrfx_get_irq_number(me.p_board->p_instance));

	me.spi_enabled = true;

	return ERROR_OK;
}

static void spi_teardown(void)
{
	if (!me.spi_enabled)
		return;

	NRFX_IRQ_DISABLE(nrfx_get_irq_number(me.p_board->p_instance));
	nrf_spim_int_disable(me.p_board->p_instance, NRF_SPIM_ALL_INTS_MASK);
	if (!nrf_spim_event_check(me.p_board->p_instance, NRF_SPIM_EVENT_STOPPED)) {
		nrf_spim_task_trigger(me.p_board->p_instance, NRF_SPIM_TASK_STOP);
		while (!nrf_spim_event_check(me.p_board->p_instance,
			NRF_SPIM_EVENT_STOPPED)) {
		}
	}

	nrf_spim_event_clear(me.p_board->p_instance, NRF_SPIM_EVENT_END);
	nrf_spim_disable(me.p_board->p_instance);
	nrf_gpio_cfg_default(me.p_board->pins.spi_cs);
	nrf_gpio_cfg_default(me.p_board->pins.spi_clk);
	nrf_gpio_cfg_default(me.p_board->pins.spi_mosi);
	nrf_gpio_cfg_default(me.p_board->pins.spi_miso);
	me.spi_enabled = false;
}

static bool is_ok_to_send(void)
{
	err_code r;

	r = spi_transfer(NULL, 0, me.trx_buf, NMEA_OK_TO_SEND_SIZE);
	ERR_CHECK_GOTO(r, exit);

	if (!memcmp(me.trx_buf, nmea_rsp_ok_to_send, strlen(nmea_rsp_ok_to_send)))
		return true;

exit:
	return false;
}

static uint16_t calc_checksum(const uint8_t *p_cmd, uint8_t size)
{
	uint8_t checksum = 0;

	while (size) {
		checksum ^= *p_cmd;
		p_cmd++;
		size--;
	}

	return checksum;
}

static err_code gps_nmea_send_cmd(const enum nmea_msg_id msg_id,
		const int8_t payload_size)
{
	uint8_t checksum = 0;

	if ((payload_size < 0) || (payload_size >= NMEA_PAYLOAD_MAX_SIZE))
		return EA2235_NMEA_ENCODE_PAYLOAD;

	snprintf(me.nmea_header, sizeof(me.nmea_header), "$PSRF%03d", msg_id);

	/* Insert nmea protocol header with msg_id */
	memcpy(me.trx_buf, me.nmea_header, strlen(me.nmea_header));

	checksum = calc_checksum((me.trx_buf + NMEA_HEADER_START_SIZE),
			(payload_size + NMEA_HEADER_SIZE - NMEA_HEADER_START_SIZE));

	/* Append '*' char, checksum and EOM termination */
	sprintf(((char *)me.trx_buf + NMEA_HEADER_SIZE + payload_size), "*%02X\r\n",
			checksum);

	return spi_transfer(me.trx_buf, (payload_size + NMEA_HEADER_SIZE +
			NMEA_FOOTER_SIZE), NULL, 0);
}

static err_code gps_nmea_set_rate(const enum nmea_msg msg,
		const enum nmea_mode mode, const uint8_t rate)
{
	int8_t payload_size;

	if ((msg >= NUM_OF_NMEA_MSGS) || (mode >= NUM_OF_NMEA_MODES))
		return EA2235_INVALID_ARG;

	payload_size = NMEA_ENCODE_PAYLOAD(",%02d,%02d,%02d,%02d", msg, mode, rate,
			true);

	return gps_nmea_send_cmd(NMEA_MSG_ID_RATE_CONTROL, payload_size);
}

static err_code gps_nmea_cold_start(void)
{
	memcpy(me.trx_buf, nmea_cmd_cold_start, strlen(nmea_cmd_cold_start));
	return spi_transfer(me.trx_buf, strlen(nmea_cmd_cold_start), NULL, 0);
}

static err_code gps_enable(void)
{
	BaseType_t rtos_r;
	err_code r;
	uint8_t i;

	if (persistent_get_ptr_gps()->is_enabled)
		return ERROR_OK;

	r = EA2235_OK_TO_SEND;
	for (i = 0; i < MAX_NBR_OF_ENABLE_RETRIES; i++) {
		if (i)
			vTaskDelay(pdMS_TO_TICKS(ENABLE_RETRY_DELAY_MS));

		me.notify_wakeup_handle = xTaskGetCurrentTaskHandle();

		send_reset_pulse();

		rtos_r = xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(WAKEUP_TMO_MS));
		me.notify_wakeup_handle = NULL;

		if (rtos_r != pdTRUE) {
			r = EA2235_WAKEUP_TMO;
			continue;
		}

		/* Serves as "On" after "Reset" */
		send_on_off_pulse();

		if (!chip_is_active()) {
			r = EA2235_WAKEUP_NOT_ACTIVE;
			continue;
		}

		if (!is_ok_to_send())
			continue;

		r = gps_nmea_cold_start();
		if (r != ERROR_OK)
			continue;

		vTaskDelay(pdMS_TO_TICKS(GPS_COLD_START_DELAY_MS));
		if (is_ok_to_send()) {
			r = ERROR_OK;
			break;
		}
	}

	if (r == ERROR_OK)
		persistent_get_ptr_gps()->is_enabled = true;

	return r;
}

static void gps_disable(void)
{
	if (!persistent_get_ptr_gps()->is_enabled)
		return;

	/* Serves as "Off" after "On" */
	send_on_off_pulse();

	persistent_get_ptr_gps()->is_enabled = false;
}

static err_code validate_fw_version(void)
{
	uint32_t i;
	uint32_t j;
	err_code r;

	for (i = 0; i < MAX_NBR_OF_READ_SW_VERS_RETRIES; i++) {
		memcpy(me.trx_buf, nmea_cmd_read_sw_vers,
			strlen(nmea_cmd_read_sw_vers));

		r = spi_transfer(me.trx_buf, strlen(nmea_cmd_read_sw_vers), NULL, 0);
		if (r != ERROR_OK) {
			vTaskDelay(SW_VERSION_RETRY_DELAY_MS);
			continue;
		}

		/* According to the module's data sheet, the sequence for this is to
		 * first send the READ_SW command, wait for an arbitrary delay and then
		 * extract a complete NMEA packet. Unfortunately, if the readout fails
		 * the delay needs to be increased in between the retries in order to
		 * find a suitable delay, hence the add-ons below.
		 */
		vTaskDelay(pdMS_TO_TICKS(SW_VERSION_RSP_DELAY_MS +
			(i * SW_VERSION_RSP_ADD_ON_DELAY_MS)));

		r = spi_transfer(NULL, 0, me.trx_buf, sizeof(me.trx_buf));
		if (r != ERROR_OK) {
			vTaskDelay(pdMS_TO_TICKS(SW_VERSION_RETRY_DELAY_MS));
			continue;
		}

		r = EA2235_INVALID_NMEA_RSP;
		for (j = 0; j < (sizeof(me.trx_buf) - NMEA_FW_VERS_RESP_LEN); j++) {
			if (memcmp(nmea_rsp_read_sw_vers, &me.trx_buf[j],
					strlen(nmea_rsp_read_sw_vers)) != 0)
				continue;

			if (strncmp(EXPECTED_FW_VERSION_STRING,
					(char *)&me.trx_buf[j + NMEA_PACKET_FW_VERS_OFFSET],
					(EXPECTED_FW_VERSION_LEN - 1)) == 0)
				return ERROR_OK;

			LOGE("Invalid fw:%.12s, expected:%.12s",
				&me.trx_buf[j + NMEA_PACKET_FW_VERS_OFFSET],
				EXPECTED_FW_VERSION_STRING);
			return EA2235_FW_VERSION;
		}

		vTaskDelay(pdMS_TO_TICKS(SW_VERSION_RETRY_DELAY_MS));
	}

	return r;
}

static void task(void *arg)
{
	struct eventpump_param param;
	enum queue_item queue_item;
	err_code r;

	if (!persistent_get_ptr_gps()->is_verified)
		vTaskDelay(pdMS_TO_TICKS(GPS_WAKE_UP_DELAY_MS));

	r = spi_setup();
	ERR_CHECK_GOTO(r, exit);

	/* Make sure to reset, enable and verify the GPS once per battery insert,
	 * else the fix will be lost and there will be a connection delay in between
	 * every soft reset. Disable once verified.
	 */
	if (!persistent_get_ptr_gps()->is_verified) {
		r = gps_enable();
		ERR_CHECK_GOTO(r, exit);

		gps_disable();
		persistent_get_ptr_gps()->is_verified = true;
	}

	param.source = EVENT_GPS;

	while (true) {
		if (r != ERROR_OK) {
			LOGE("%s: r=0x%08lX", __func__, r);
			session_mgr_append_err_code(r, __func__);
		}
		r = ERROR_OK;

		vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));

		if (xQueueReceive(me.queue_handle, &queue_item,
				(me.started ? 0 : portMAX_DELAY)) == pdTRUE) {
			switch (queue_item) {
			case QUEUE_ITEM_SHUTDOWN_REQ:
				spi_teardown();
				gps_disable();
				goto exit;
				break;
			case QUEUE_ITEM_START_REQ:
				r = gps_a2235_start();
				if (r != ERROR_OK)
					continue;

				param.gps.event = GPS_STARTED;
				r = eventpump_post(&param);
				if (r != ERROR_OK)
					LOGE("EvtPmp STARTED");
				break;
			case QUEUE_ITEM_STOP_REQ:
				r = gps_a2235_stop();
				if (r != ERROR_OK)
					continue;

				param.gps.event = GPS_STOPPED;
				r = eventpump_post(&param);
				if (r != ERROR_OK)
					LOGE("EvtPmp STOPPED");
				break;
			case QUEUE_ITEM_WAKEUP_OFF:
				if (persistent_get_ptr_gps()->is_enabled) {
					persistent_get_ptr_gps()->is_enabled = false;

					LOGV("Restart");
					param.gps.event = GPS_RESTARTED;
					r = eventpump_post(&param);
					if (r != ERROR_OK)
						LOGE("EvtPmp RESTARTED");

					r = gps_enable();
					if (r != ERROR_OK)
						LOGE("Enable");
				}
				break;
			default:
				break;
			}

			continue;
		}

		if (!util_mutex_lock(me.mutex_handle))
			continue;

		r = spi_transfer(NULL, 0, me.trx_buf, NMEA_SINGLE_EXTRACT_SIZE);

		util_mutex_unlock(me.mutex_handle);

		if (r != ERROR_OK) {
			LOGE("spi_transfer r=0x%08lX", r);
			continue;
		}

		if (!nmea_parser_process(&me.nmea_data,
				me.trx_buf, NMEA_SINGLE_EXTRACT_SIZE)) {
			LOGE("nmea_parser");
			continue;
		}

		if (!me.gps_fixed) {
			if (me.nmea_data.is_valid) {
				param.gps.event = GPS_FIX;
				param.gps.time.year = me.nmea_data.year;
				param.gps.time.month = me.nmea_data.month;
				param.gps.time.day = me.nmea_data.date;
				param.gps.time.hour = me.nmea_data.hours;
				param.gps.time.minute = me.nmea_data.minutes;
				param.gps.time.second = me.nmea_data.seconds;

				r = eventpump_post(&param);
				if (r != ERROR_OK)
					LOGE("EvtPmp FIX");
				else
					me.gps_fixed = true;
			}
			continue;
		}

		if (!me.nmea_data.is_valid) {
			me.gps_fixed = false;
			param.gps.event = GPS_NO_SIGNAL;
			r = eventpump_post(&param);
			if (r != ERROR_OK)
				LOGE("EvtPmp NO_SIG");
			continue;
		}

		/* TODO: Investigate if these non-values are produced in the parser or
		 * provided by the GPS module (could be that the values are not read out
		 * at the correct time) and adjust accordingly.
		 */
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
			LOGE("EvtPmp PARAMS");
	}

exit:
	if (r != ERROR_OK)
		LOGE("%s: r=0x%08lX", __func__, r);

	me.p_board = NULL;
	while (true)
		vTaskDelay(portMAX_DELAY);
}

err_code gps_a2235_start(void)
{
	err_code r;

	if (me.p_board == NULL)
		return EA2235_NO_INIT;

	if (!persistent_get_ptr_gps()->is_verified)
		return EA2235_NO_VERIF;

	if (me.started)
		return ERROR_OK;

	if (!util_mutex_lock(me.mutex_handle))
		return EA2235_MUTEX;

	r = gps_enable();
	ERR_CHECK_GOTO(r, exit);

	r = validate_fw_version();
	ERR_CHECK_GOTO(r, exit);

	/* Disable NMEA GGA message */
	r = gps_nmea_set_rate(NMEA_MSG_GGA, NMEA_MODE_SET_RATE, NMEA_OUTPUT_RATE_OFF);
	ERR_CHECK_GOTO(r, exit);

	/* Disable NMEA GSV message */
	r = gps_nmea_set_rate(NMEA_MSG_GSV, NMEA_MODE_SET_RATE, NMEA_OUTPUT_RATE_OFF);
	ERR_CHECK_GOTO(r, exit);

	/* Enable 5Hz update rate on RMC message */
	r = gps_nmea_set_rate(NMEA_MSG_RMC, NMEA_MODE_5HZ_ON, NMEA_OUTPUT_RATE_OFF);
	ERR_CHECK_GOTO(r, exit);

	/* Enable 5Hz update rate on GSA message */
	r = gps_nmea_set_rate(NMEA_MSG_GSA, NMEA_MODE_5HZ_ON, NMEA_OUTPUT_RATE_OFF);
	ERR_CHECK_GOTO(r, exit);

	me.started = true;

exit:
	if (r != ERROR_OK) {
		gps_disable();
		LOGE("%s r=0x%08lX", __func__, r);
	}

	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code gps_a2235_stop(void)
{
	if (me.p_board == NULL)
		return EA2235_NO_INIT;

	if (!persistent_get_ptr_gps()->is_verified)
		return EA2235_NO_VERIF;

	if (!util_mutex_lock(me.mutex_handle))
		return EA2235_MUTEX;

	gps_disable();
	me.started = false;
	util_mutex_unlock(me.mutex_handle);

	return ERROR_OK;
}

err_code gps_a2235_init(const struct gps_a2235_board * const p_board)
{
	err_code r;

	if (me.p_board != NULL)
		return ERROR_OK;

	if ((p_board == NULL) || (p_board->p_instance == NULL))
		return EA2235_PDATA;

	r = util_validate_pins((uint8_t *)&p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	me.p_board = p_board;

	nrf_gpio_pin_set(me.p_board->pins.rst);
	nrf_gpio_cfg(me.p_board->pins.rst,
				NRF_GPIO_PIN_DIR_OUTPUT,
				NRF_GPIO_PIN_INPUT_DISCONNECT,
				NRF_GPIO_PIN_NOPULL,
				NRF_GPIO_PIN_S0D1,
				NRF_GPIO_PIN_NOSENSE);

	nrf_gpio_pin_clear(me.p_board->pins.on);
	nrf_gpio_cfg(me.p_board->pins.on,
				NRF_GPIO_PIN_DIR_OUTPUT,
				NRF_GPIO_PIN_INPUT_DISCONNECT,
				NRF_GPIO_PIN_NOPULL,
				NRF_GPIO_PIN_S0S1,
				NRF_GPIO_PIN_NOSENSE);

	me.queue_handle = CREATE_STATIC_QUEUE(TASK_NAME);
	me.mutex_handle = CREATE_STATIC_MUTEX(TASK_NAME);
	me.task_handle = CREATE_STATIC_TASK(task, TASK_NAME, TASK_PRIORITY);

	r = tasktracker_register_task(me.task_handle);
	ERR_CHECK_GOTO(r, error_exit);

	r = gpio_irq_register(p_board->pins.wakeup, GPIO_IRQ_POL_TOGGLE,
			wakeup_irq_callback);
	ERR_CHECK_GOTO(r, error_exit);

	nrf_gpio_cfg_input(p_board->pins.wakeup, NRF_GPIO_PIN_NOPULL);

	return ERROR_OK;

error_exit:
	me.p_board = NULL;
	return r;
}
