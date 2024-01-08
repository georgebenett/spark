#include <nrf_gpio.h>
#include <nrf_spim.h>
#include <nrf_spi.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <task.h>

#include "boards.h"
#include "drivers/adxl.h"
#include "drivers/gpio_irq.h"
#include "drivers/twim_spim_irq.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "freertos_tasktracker.h"
#include "persistent.h"
#include "settings.h"
#include "util.h"

#define DRV_NAME			ADXL
#define LOG_TAG				STRINGIFY(DRV_NAME)
#include "log.h"

/* Driver specifics */
#define TASK_NAME			DRV_NAME
#define TASK_PRIORITY		2
#define TASK_STACK_DEPTH	192
#define QUEUE_LEN			4
#define FIFO_SIZE			512
#define MAX_RETRY_COUNT		20
#define RETRY_DELAY_MS		50

#define DEFAULT_INACT_TIME		5
#define DEFAULT_INACT_THRESHOLD	100
#define DEFAULT_ACT_TIME		0
#define DEFAULT_ACT_THRESHOLD	150

STATIC_TASK(TASK_NAME, TASK_STACK_DEPTH);
STATIC_MUTEX(TASK_NAME);
STATIC_QUEUE_0(TASK_NAME, QUEUE_LEN);

/* ADXL362 Register Map */
#define REG_DEVID_AD		0x00
#define REG_DEVID_MST		0x01
#define REG_PARTID			0x02
#define REG_REVID			0x03
#define REG_XDATA			0x08
#define REG_YDATA			0x09
#define REG_ZDATA			0x0A
#define REG_STATUS			0x0B
#define REG_FIFO_ENTRIES_L	0x0C
#define REG_FIFO_ENTRIES_H	0x0D
#define REG_XDATA_L			0x0E
#define REG_XDATA_H			0x0F
#define REG_YDATA_L			0x10
#define REG_YDATA_H			0x11
#define REG_ZDATA_L			0x12
#define REG_ZDATA_H			0x13
#define REG_TEMP_L			0x14
#define REG_TEMP_H			0x15
#define REG_SOFT_RESET		0x1F
#define REG_THRESH_ACT_L	0x20
#define REG_THRESH_ACT_H	0x21
#define REG_TIME_ACT		0x22
#define REG_THRESH_INACT_L	0x23
#define REG_THRESH_INACT_H	0x24
#define REG_TIME_INACT_L	0x25
#define REG_TIME_INACT_H	0x26
#define REG_ACT_INACT_CTL	0x27
#define REG_FIFO_CTL		0x28
#define REG_FIFO_SAMPLES	0x29
#define REG_INTMAP1			0x2A
#define REG_INTMAP2			0x2B
#define REG_FILTER_CTL		0x2C
#define REG_POWER_CTL		0x2D
#define REG_SELF_TEST		0x2E

/* ACT_INACT_CTL */
#define LINK_LOOP(x)		(((x) & 0x3) << 4)
#define DEFA_MODE			0
#define LINK_MODE			1
#define LOOP_MODE			3
#define INACT_REF			(1 << 3)
#define INACT_EN			(1 << 2)
#define ACT_REF				(1 << 1)
#define ACT_EN				(1 << 0)

/* FILTER_CTL */
#define RANGE(x)			((x) << 6)
#define PM_2g				0
#define PM_4g				1
#define PM_8g				2
#define FULL_BW				(0)
#define HALF_BW				(1 << 4)
#define EXT_SAMPLE			(1 << 3)
#define ODR(x)				((x) & 0x7)
#define RAW_12_5HZ			(0)
#define RAW_25HZ			(0x01)
#define RAW_50HZ			(0x02)
#define RAW_100HZ			(0x03)
#define RAW_200HZ			(0x04)
#define RAW_400HZ			(0x05)

/* FIFO_CTL */
#define FIFO_AH				(1 << 3)
#define FIFO_TEMP_EN		(1 << 2)
#define FIFO_MODE(x)		((x) & 0x3)
#define FIFO_DISABLE		0
#define FIFO_OLDEST			1
#define FIFO_STREAM			2
#define FIFO_TRIG			3

/* INTMAP1/INTMAP2 */
#define INT_LOW_ACTIVE		(1 << 7)
#define INT_AWAKE_EN		(1 << 6)
#define INT_INACT_EN		(1 << 5)
#define INT_ACT_EN			(1 << 4)
#define INT_FIFO_OVERRUN_EN	(1 << 3)
#define INT_FIFO_WATERMARK_EN	(1 << 2)
#define INT_FIFO_READY_EN	(1 << 1)
#define INT_DATA_READY_EN	(1 << 0)

#define ODR_TO_HZ(x)			((125 << (x)) / 10)
#define HZ_TO_ODR(x)			(ilog2(((x) * 10) / 125))
#define CLAMP_WM(x)				(clamp_t(u8, x, MIN_FIFO_SETS, MAX_T_FIFO_SETS))
#define CLAMP_ACT(x)			(clamp_t(uint8_t, x, 1, 0xFF))
#define CLAMP_INACT(x)			(clamp_t(uint16_t, x, 1, 0xFFFF))

/* POWER_CTL */
#define EXT_CLK					(1 << 6)
#define LOW_NOISE(x)			((x) << 4)
#define WAKE_UP					(1 << 3)
#define AUTOSLEEP				(1 << 2)
#define MEASUREMENT_MODE		(1 << 1)

/* STATUS */
#define STATUS_ERR_USER_REGS	(1 << 7)
#define STATUS_AWAKE			(1 << 6)
#define STATUS_INACT			(1 << 5)
#define STATUS_ACT				(1 << 4)
#define STATUS_FIFO_OVERRUN		(1 << 3)
#define STATUS_FIFO_WATERMARK	(1 << 2)
#define STATUS_FIFO_READY		(1 << 1)
#define STATUS_DATA_READY		(1 << 0)

/* SOFT_RESET */
#define SOFT_RESET				0x52

/* DEVID */
#define DEVICE_ID				0xAD
#define PART_ID					0xF2

/* Commands */
#define CMD_FIFO				0x0D
#define CMD_READ				0x0B
#define CMD_WRITE				0x0A

/* The SPI HAL is limited to read 255 bytes at the time because of the
 * hardware register TXD.MAXCNT is limited to 8 bit (used by the DMA).
 *
 * TODO: Potentially eliminate this restriction by using EasyDMA array list.
 */
#define MAX_DMA_ARRAY_SIZE		(255)

/* The accelerometer FIFO has a maximum capacity of 512 sample words with each
 * value being represented by the following format:
 *
 * |B15|B14| - Data Type: 00: X-axis, 01: Y-axis, 10: Z-axis, 11: Temperature
 * |B13|B12| - Sign extension
 * |B11| - MSB
 * |B10|...|B0| - Data
 *
 * Word size: 2 Bytes
 * Sample set: 3 words, eg. one word per accelerometer axis (x, y, z)
 */
#define WORD_SIZE					(2)
#define WORDS_IN_SAMPLE_SET			(3)
#define SAMPLE_SET_SIZE				(WORDS_IN_SAMPLE_SET * WORD_SIZE)
#define MAX_NBR_SAMPLE_SETS			(MAX_DMA_ARRAY_SIZE / SAMPLE_SET_SIZE)
#define MAX_SAMPLE_SETS_SIZE		(MAX_NBR_SAMPLE_SETS * SAMPLE_SET_SIZE)

/* The FIFO watermark is set to below the maximum capacity for cases where the
 * nRF is delayed before the watermark interrupt is being served. This enables
 * the accelerometer to add more samples to the FIFO without overflowing it.
 */
#define FIFO_WATERMARK				(WORDS_IN_SAMPLE_SET * MAX_NBR_SAMPLE_SETS)
#define FIFO_WATERMARK_PRODTEST		(WORDS_IN_SAMPLE_SET * 6)

#if ((FIFO_WATERMARK > FIFO_SIZE) || \
	(FIFO_WATERMARK_PRODTEST > FIFO_SIZE))
#error "Driver FIFO watermark exceeds adxl362 FIFO size!!"
#endif

/* Offset for the registers used for ADXL362 Configuration */
#define CFG_REG_FIRST		REG_THRESH_ACT_L
#define CFG_REG_LAST		REG_POWER_CTL
#define CFG_REG_SIZE		(CFG_REG_LAST - CFG_REG_FIRST + 1)
#define CFG_REG_IDX(reg, idx) \
	do { \
		STATIC_ASSERT((reg >= CFG_REG_FIRST) && \
							(reg <= CFG_REG_LAST)); \
		idx = (reg - CFG_REG_FIRST); \
	} while (0)

#define DISCONNECT_GPIO(pin) \
	do { \
		if (pin < NUMBER_OF_PINS) { \
			nrf_gpio_cfg(pin, \
				NRF_GPIO_PIN_DIR_INPUT, \
				NRF_GPIO_PIN_INPUT_DISCONNECT, \
				NRF_GPIO_PIN_NOPULL, \
				NRF_GPIO_PIN_D0S1, \
				NRF_GPIO_PIN_NOSENSE); \
		} \
	} while (0)

#define MUTEX_TMO pdMS_TO_TICKS(30)
#define NOTIFY_TAKE_WAIT_TICKS pdMS_TO_TICKS(100)
#define SPI_DISABLE_ALL_INT 0xFFFFFFFF

struct fifo {
	int16_t		buf[FIFO_SIZE];
	uint16_t	samples;
	bool		is_dirty;
};

static struct {
	const struct adxl_board	*p_board;
	SemaphoreHandle_t		mutex_handle;
	QueueHandle_t			queue_handle;
	TaskHandle_t			task_handle;
	TaskHandle_t			task_to_notify;
	enum adxl_mode			mode;
	struct fifo				fifo;
	uint32_t				timestamp_active;
	bool					spi_enabled;
	bool					enabled;
	struct settings_adxl	settings;
} me;

void spim_irq_callback(void)
{
	BaseType_t yield_req;

	yield_req = pdFALSE;

	if (nrf_spim_event_check(me.p_board->p_spi, NRF_SPIM_EVENT_END)) {
		nrf_spim_event_clear(me.p_board->p_spi, NRF_SPIM_EVENT_END);
		if (me.task_to_notify != NULL)
			vTaskNotifyGiveFromISR(me.task_to_notify, &yield_req);
	}
	portYIELD_FROM_ISR(yield_req);
}

static err_code spi_setup(void)
{
	if (me.spi_enabled)
		return ERROR_OK;

	/* SPI Mode 0 */
	nrf_gpio_pin_clear(me.p_board->pins.spi_clk);

	nrf_gpio_cfg(me.p_board->pins.spi_clk,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_CONNECT,
			NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_S0S1,
			NRF_GPIO_PIN_NOSENSE);

	nrf_gpio_pin_clear(me.p_board->pins.spi_mosi);
	nrf_gpio_cfg_output(me.p_board->pins.spi_mosi);

	nrf_gpio_cfg_input(me.p_board->pins.spi_miso, NRF_GPIO_PIN_PULLDOWN);

	nrf_gpio_pin_set(me.p_board->pins.spi_cs);
	nrf_gpio_cfg_output(me.p_board->pins.spi_cs);

	nrf_spim_pins_set(me.p_board->p_spi, me.p_board->pins.spi_clk,
		me.p_board->pins.spi_mosi, me.p_board->pins.spi_miso);
	nrf_spim_frequency_set(me.p_board->p_spi, me.p_board->frequency);
	nrf_spim_configure(me.p_board->p_spi,
			NRF_SPIM_MODE_0,
			NRF_SPIM_BIT_ORDER_MSB_FIRST);

	nrf_spim_int_enable(me.p_board->p_spi, NRF_SPIM_INT_END_MASK);
	NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(me.p_board->p_spi),
		APP_IRQ_PRIORITY_LOW);
	NRFX_IRQ_PENDING_CLEAR(nrfx_get_irq_number(me.p_board->p_spi));
	NRFX_IRQ_ENABLE(nrfx_get_irq_number(me.p_board->p_spi));
	nrf_spim_enable(me.p_board->p_spi);

	me.spi_enabled = true;

	return ERROR_OK;
}

static err_code spi_teardown(void)
{
	if (!me.spi_enabled)
		return ERROR_OK;

	NRFX_IRQ_DISABLE(nrfx_get_irq_number(me.p_board->p_spi));
	NRFX_IRQ_PENDING_CLEAR(nrfx_get_irq_number(me.p_board->p_spi));
	nrf_spim_int_disable(me.p_board->p_spi, SPI_DISABLE_ALL_INT);

	if (!nrf_spim_event_check(me.p_board->p_spi, NRF_SPIM_EVENT_STOPPED)) {
		nrf_spim_task_trigger(me.p_board->p_spi, NRF_SPIM_TASK_STOP);
		while (!nrf_spim_event_check(me.p_board->p_spi,
			NRF_SPIM_EVENT_STOPPED)) {
		}
	}

	nrf_spim_event_clear(me.p_board->p_spi, NRF_SPIM_EVENT_END);
	nrf_spim_disable(me.p_board->p_spi);
	DISCONNECT_GPIO(me.p_board->pins.spi_cs);
	DISCONNECT_GPIO(me.p_board->pins.spi_clk);
	DISCONNECT_GPIO(me.p_board->pins.spi_mosi);
	DISCONNECT_GPIO(me.p_board->pins.spi_miso);

	me.spi_enabled = false;

	return ERROR_OK;
}

static err_code spi_transfer(const uint8_t *tx_buf, const uint8_t tx_size,
	uint8_t *rx_buf, uint8_t rx_size)
{
	if ((tx_buf == NULL && tx_size != 0) ||
		(rx_buf == NULL && rx_size != 0))
		return EADXL_SPI_TRANSFER;

	nrf_gpio_pin_clear(me.p_board->pins.spi_cs);

	nrf_spim_tx_buffer_set(me.p_board->p_spi, tx_buf, tx_size);
	nrf_spim_rx_buffer_set(me.p_board->p_spi, rx_buf, rx_size);

	nrf_spim_event_clear(me.p_board->p_spi, NRF_SPIM_EVENT_END);
	ulTaskNotifyTake(pdTRUE, 0);
	me.task_to_notify = xTaskGetCurrentTaskHandle();

	nrf_spim_task_trigger(me.p_board->p_spi, NRF_SPIM_TASK_START);

	if (xTaskNotifyWait(0, 0xFFFFFFFF, NULL, NOTIFY_TAKE_WAIT_TICKS) !=
			pdTRUE) {
		nrf_spim_task_trigger(me.p_board->p_spi, NRF_SPIM_TASK_STOP);
		return EADXL_TIMEOUT;
	}

	me.task_to_notify = NULL;
	nrf_gpio_pin_set(me.p_board->pins.spi_cs);

	return ERROR_OK;
}

static err_code write8(uint16_t reg, uint8_t value)
{
	uint8_t cmd[3];

	cmd[0] = CMD_WRITE;
	cmd[1] = reg;
	cmd[2] = value;

	if (!me.spi_enabled)
		return EADXL_SPI_DISABLED;

	return spi_transfer(cmd, sizeof(cmd), NULL, 0);
}

static err_code read16(uint16_t reg, uint16_t *p_value)
{
	uint8_t cmd[2];
	uint8_t buf[4];
	err_code r;

	cmd[0] = CMD_READ;
	cmd[1] = reg;

	if (!me.spi_enabled)
		return EADXL_SPI_DISABLED;

	if (p_value == NULL)
		return EADXL_INVALID_ARG;

	r = spi_transfer(cmd, sizeof(cmd), buf, sizeof(buf));
	ERR_CHECK(r);

	*p_value = buf[2] | (buf[3] << 8);
	return ERROR_OK;
}

static err_code read_fifo_status(uint8_t *p_status,
										uint16_t *p_fifo_entries)
{
	uint8_t cmd[2];
	uint8_t buf[5];
	err_code r;

	cmd[0] = CMD_READ;
	cmd[1] = REG_STATUS;

	if (p_status == NULL || p_fifo_entries == NULL)
		return EADXL_INVALID_ARG;

	r = spi_transfer(cmd, sizeof(cmd), buf, sizeof(buf));
	ERR_CHECK(r);

	*p_status = buf[2];
	*p_fifo_entries = buf[3] | (buf[4] << 8);

	return ERROR_OK;
}

static err_code read_chip_id(struct adxl_chip_id *p_id)
{
	uint8_t cmd[2];
	uint8_t buf[6];
	err_code r;

	cmd[0] = CMD_READ;
	cmd[1] = REG_DEVID_AD;

	if (p_id == NULL)
		return EADXL_INVALID_ARG;

	r = spi_transfer(cmd, sizeof(cmd), buf, sizeof(buf));
	ERR_CHECK(r);

	p_id->dev = buf[2];
	/* buf[3] contains ADI's MEMS device id which we aren't interested of */
	p_id->part = buf[4];
	p_id->rev = buf[5];

	return ERROR_OK;
}

static void power(bool value)
{
	if (me.p_board->pins.vdd != BOARD_UNUSED_PIN)
		return;

	vTaskDelay(pdMS_TO_TICKS(6));
	nrf_gpio_pin_write(me.p_board->pins.vdd, value);
	vTaskDelay(pdMS_TO_TICKS(6));
}

static void reset_hard(void)
{
	power(false);
	power(true);
}

static err_code reset_soft(void)
{
	err_code r;

	r = write8(REG_SOFT_RESET, SOFT_RESET);
	ERR_CHECK(r);

	/* SOFT_RESET takes ~500us */
	vTaskDelay(pdMS_TO_TICKS(1));

	return ERROR_OK;
}

static err_code read_fifo(uint16_t nbr_samples)
{
	uint16_t nbr_sample_sets;
	struct fifo *p_fifo;
	uint16_t nbr_bytes;
	uint8_t cmd;
	err_code r;

	p_fifo = &me.fifo;
	cmd = CMD_FIFO;

	if (p_fifo->is_dirty)
		LOGW("FIFO Buffer got overwritten before it was read");

	/* Make sure to always read out number of bytes corresponding to complete
	 * xyz sample-sets. This is in order to keep the axis alignment intact.
	 */
	nbr_sample_sets = (nbr_samples / WORDS_IN_SAMPLE_SET);
	nbr_bytes = MIN(nbr_sample_sets * SAMPLE_SET_SIZE, MAX_SAMPLE_SETS_SIZE);

	/* The first byte to be clocked in will be the FIFO read command itself
	 * hence the offset of storing this in the second byte of the fifo buffer.
	 * And also, by storing it in the second- instead of the first byte we
	 * get the desired uint16_t alignment (little endian) with sensor data
	 * starting at p_fifo->buf[1]. And, since this is a transfer, the command
	 * byte that will be clocked out needs to be added to the total sum of bytes
	 * to receive in order to get all the desired bytes.
	 */
	r = spi_transfer(&cmd, 1, ((uint8_t *)&p_fifo->buf[0]) + 1, nbr_bytes + 1);
	ERR_CHECK(r);

	p_fifo->samples = (nbr_bytes / WORD_SIZE);
	p_fifo->is_dirty = true;

	return ERROR_OK;
}

static void irq_tophalf(uint8_t pin, uint32_t value)
{
	if ((me.p_board == NULL) || !me.enabled)
		return;

	if (pin != me.p_board->pins.int1)
		return;

	xQueueSendToBackFromISR(me.queue_handle, NULL, NULL);
}

static err_code irq_bottomhalf(void)
{
	struct eventpump_param param;
	uint16_t fifo_entries;
	uint32_t t_now;
	uint8_t status;
	err_code r;

	r = ERROR_OK;
	param.source = EVENT_ACCEL;

	if (!util_mutex_lock(me.mutex_handle))
		return EADXL_MUTEX;

	/* Silently discard any potential irq that might occur while disabled */
	if (!me.enabled)
		goto out;

	r = walltime_get_uptime(&t_now, NULL);
	ERR_CHECK_GOTO(r, teardown);

	r = spi_setup();
	ERR_CHECK_GOTO(r, teardown);

	r = read_fifo_status(&status, &fifo_entries);
	ERR_CHECK_GOTO(r, teardown);

	if (status & STATUS_INACT) {
		if (me.mode == ADXL_MODE_STREAM_MOTION_DETECT) {
			r = write8(REG_FIFO_CTL,
					FIFO_MODE(FIFO_DISABLE));
			if (r != ERROR_OK) {
				LOGE("adxl_write8 r=0x%08lX", r);
				goto teardown;
			}
		}

		me.fifo.is_dirty = false;
		param.accel.event = ADXL_EVENT_STILLNESS;
		r = eventpump_post(&param);
	} else if (status & STATUS_ACT) {
		if (me.mode == ADXL_MODE_STREAM_MOTION_DETECT) {
			r = write8(REG_FIFO_CTL,
					FIFO_MODE(FIFO_STREAM) |
						(FIFO_WATERMARK > 0xFF ? FIFO_AH : 0));
			if (r != ERROR_OK) {
				LOGE("adxl_write8 r=0x%08lX", r);
				goto teardown;
			}

			me.timestamp_active = t_now;
		}

		me.fifo.is_dirty = false;
		param.accel.event = ADXL_EVENT_MOVEMENT;
		r = eventpump_post(&param);
	} else if (status & (STATUS_FIFO_READY | STATUS_FIFO_WATERMARK)) {
		if (!fifo_entries || fifo_entries < SAMPLE_SET_SIZE)
			goto teardown;

		r = read_fifo(fifo_entries);
		ERR_CHECK_GOTO(r, teardown);

		param.accel.event = ADXL_EVENT_DATA;
		r = eventpump_post(&param);
	}

teardown:
	spi_teardown();

out:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

static void task(void *arg)
{
	err_code r;

	for (; ;) {
		if (xQueueReceive(me.queue_handle, NULL, portMAX_DELAY) == pdTRUE) {
			r = irq_bottomhalf();
			if (r != ERROR_OK)
				LOGE("adxl_irq_bottomhalf r=0x%08lX", r);
		}
	}
}

static err_code config(enum adxl_mode mode)
{
	uint8_t cmd[2 + CFG_REG_SIZE];
	bool motion_detection;
	uint8_t *p_cfg_regs;
	uint8_t cfg_reg_idx;
	err_code r;

	memset(cmd, 0, sizeof(cmd));
	motion_detection = false;
	p_cfg_regs = &cmd[2];

	if (me.mode == mode)
		return ERROR_OK;

	if (!me.spi_enabled)
		return EADXL_SPI_DISABLED;

	if (me.p_board == NULL)
		return EADXL_NO_INIT;

	/* Set range (2G), filter bandwidth and sample rate (odr) */
	CFG_REG_IDX(REG_FILTER_CTL, cfg_reg_idx);
	p_cfg_regs[cfg_reg_idx] = (RANGE(PM_2g) |
								FULL_BW | ODR(me.p_board->rate));

	/* Set irq's for fifo watermark and activity/inactivity detection */
	CFG_REG_IDX(REG_INTMAP1, cfg_reg_idx);
	p_cfg_regs[cfg_reg_idx] = (INT_FIFO_WATERMARK_EN |
								INT_ACT_EN | INT_INACT_EN);

	switch (mode) {
	case ADXL_MODE_STREAM:
		CFG_REG_IDX(REG_POWER_CTL, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = MEASUREMENT_MODE;

		CFG_REG_IDX(REG_FIFO_SAMPLES, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = FIFO_WATERMARK & 0xFF;

		CFG_REG_IDX(REG_FIFO_CTL, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = FIFO_MODE(FIFO_STREAM) |
					(FIFO_WATERMARK > 0xFF ? FIFO_AH : 0);
		break;
	case ADXL_MODE_STREAM_MOTION_DETECT:
		CFG_REG_IDX(REG_POWER_CTL, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = (AUTOSLEEP | MEASUREMENT_MODE);

		CFG_REG_IDX(REG_FIFO_SAMPLES, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = FIFO_WATERMARK & 0xFF;

		CFG_REG_IDX(REG_FIFO_CTL, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = FIFO_MODE(FIFO_STREAM) |
					(FIFO_WATERMARK > 0xFF ? FIFO_AH : 0);

		motion_detection = true;
		break;
	case ADXL_MODE_MOTION_DETECT:
		if (me.p_board->wakeup_enabled) {
			CFG_REG_IDX(REG_POWER_CTL, cfg_reg_idx);
			p_cfg_regs[cfg_reg_idx] = (WAKE_UP | MEASUREMENT_MODE);
		} else {
			CFG_REG_IDX(REG_POWER_CTL, cfg_reg_idx);
			p_cfg_regs[cfg_reg_idx] = MEASUREMENT_MODE;
		}

		motion_detection = true;
		break;
	case ADXL_MODE_PRODTEST:
		CFG_REG_IDX(REG_POWER_CTL, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = MEASUREMENT_MODE;

		CFG_REG_IDX(REG_FIFO_SAMPLES, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = FIFO_WATERMARK_PRODTEST;

		CFG_REG_IDX(REG_FIFO_CTL, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = FIFO_MODE(FIFO_STREAM) |
					(FIFO_WATERMARK_PRODTEST > 0xFF ? FIFO_AH : 0);
		break;
	case ADXL_MODE_OFF:
	default:
		break;
	}

	if (motion_detection) {
		/* Inactivity Detection */
		CFG_REG_IDX(REG_TIME_INACT_L, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] =
			(me.settings.inactivity_time_s * ODR_TO_HZ(me.p_board->rate));

		CFG_REG_IDX(REG_TIME_INACT_H, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] =
			(((me.settings.inactivity_time_s *
				ODR_TO_HZ(me.p_board->rate)) & 0x700) >> 8);

		CFG_REG_IDX(REG_THRESH_INACT_L, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = me.settings.inactivity_threshold;

		CFG_REG_IDX(REG_THRESH_INACT_H, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] =
			((me.settings.inactivity_threshold & 0x700) >> 8);

		/* Activity Detection */
		CFG_REG_IDX(REG_TIME_ACT, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] =
			(me.settings.activity_time_s * ODR_TO_HZ(me.p_board->rate));

		CFG_REG_IDX(REG_THRESH_ACT_L, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = me.settings.activity_threshold;

		CFG_REG_IDX(REG_THRESH_ACT_H, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] =
			((me.settings.activity_threshold & 0x700) >> 8);

		CFG_REG_IDX(REG_ACT_INACT_CTL, cfg_reg_idx);
		p_cfg_regs[cfg_reg_idx] = (ACT_EN | ACT_REF | INACT_EN |
			INACT_REF | LINK_LOOP(LINK_MODE));
	}

	cmd[0] = CMD_WRITE;
	cmd[1] = CFG_REG_FIRST;

	r = spi_transfer(cmd, sizeof(cmd), NULL, 0);
	ERR_CHECK(r);

	me.mode = mode;

	return ERROR_OK;
}

err_code adxl_get_temperature(int16_t *temperature)
{
	bool spi_was_enabled;
	err_code r;

	spi_was_enabled = true;

	if (temperature == NULL)
		return EADXL_INVALID_ARG;

	if (!me.enabled)
		return EADXL_NOT_ENABLED;

	if (!util_mutex_lock(me.mutex_handle))
		return EADXL_MUTEX;

	if (!me.spi_enabled) {
		r = spi_setup();
		ERR_CHECK_GOTO(r, exit_fail);
		spi_was_enabled = false;
	}

	/* TODO: Add accel mode check */
	r = read16(REG_TEMP_L, (uint16_t *) temperature);

	if (!spi_was_enabled)
		spi_teardown();

exit_fail:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code adxl_get_samples(int16_t **p_samples, int *p_num_samples)
{
	struct fifo *p_fifo;
	uint16_t *p_U;
	int16_t *p_I;
	err_code r;

	p_fifo = &me.fifo;
	p_U = (uint16_t *)&p_fifo->buf[1];
	p_I = (int16_t *)p_U;
	r = ERROR_OK;

	if ((p_samples == NULL) || (p_num_samples == NULL))
		return EADXL_INVALID_ARG;

	if (!me.enabled)
		return EADXL_NOT_ENABLED;

	if (!util_mutex_lock(me.mutex_handle))
		return EADXL_MUTEX;

	if (!p_fifo->is_dirty) {
		r = EADXL_EMPTY_FIFO;
		goto exit_fail;
	}

	for (int i = 0; i < p_fifo->samples; i++) {
		if (p_U[i] & 0x800)
			p_U[i] |= 0xf000;
		else
			p_U[i] &= 0x0fff;
	}

	*p_samples = p_I;
	*p_num_samples = p_fifo->samples;
	p_fifo->is_dirty = false;

exit_fail:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code adxl_get_sample(int16_t *p_x, int16_t *p_y, int16_t *p_z)
{
	struct fifo *p_fifo;
	uint8_t last_sample_set_offset;

	p_fifo = &me.fifo;

	if (!me.enabled)
		return EADXL_NOT_ENABLED;

	/*
	 * Since the drivers is written to be FIFO-oriented this functions is fitted
	 * into this setup, eg. read out the last sample set from the lastly stored
	 * fifo interrupt.
	 */
	if (!p_fifo->samples)
		return EADXL_EMPTY_FIFO;

	if (!util_mutex_lock(me.mutex_handle))
		return EADXL_MUTEX;

	last_sample_set_offset = (p_fifo->samples - SAMPLE_SET_SIZE);

	*p_x = (int16_t)p_fifo->buf[last_sample_set_offset + 1];
	*p_y = (int16_t)p_fifo->buf[last_sample_set_offset + 2];
	*p_z = (int16_t)p_fifo->buf[last_sample_set_offset + 3];

	util_mutex_unlock(me.mutex_handle);

	return ERROR_OK;
}

err_code adxl_get_id(struct adxl_chip_id *p_id)
{
	err_code r;

	if (p_id == NULL)
		return EADXL_INVALID_ARG;

	if (!me.enabled)
		return EADXL_NOT_ENABLED;

	if (!util_mutex_lock(me.mutex_handle))
		return EADXL_MUTEX;

	r = spi_setup();
	ERR_CHECK_GOTO(r, exit_fail);

	r = read_chip_id(p_id);

	spi_teardown();

exit_fail:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code adxl_get_is_initialized(bool *p_initialized)
{
	if (p_initialized == NULL)
		return EADXL_INVALID_ARG;

	*p_initialized = (me.p_board != NULL);

	return ERROR_OK;
}

err_code adxl_enable(enum adxl_mode mode)
{
	err_code r = ERROR_OK;

	if (mode >= NBR_OF_ADXL_MODES)
		return EADXL_INVALID_ARG;

	if (me.p_board == NULL)
		return EADXL_NO_INIT;

	if (me.enabled) {
		r = adxl_disable();
		ERR_CHECK(r);
	}

	if (!util_mutex_lock(me.mutex_handle))
		return EADXL_MUTEX;

	power(true);

	r = spi_setup();
	ERR_CHECK_GOTO(r, exit_fail);

	r = reset_soft();
	ERR_CHECK_GOTO(r, exit_fail);

	r = config(mode);
	ERR_CHECK_GOTO(r, exit_fail);

	me.fifo.is_dirty = false;
	me.enabled = true;

exit_fail:
	spi_teardown();
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code adxl_disable(void)
{
	err_code r;

	if (me.p_board == NULL)
		return EADXL_NO_INIT;

	if (!me.enabled)
		return ERROR_OK;

	if (!util_mutex_lock(me.mutex_handle))
		return EADXL_MUTEX;

	r = spi_setup();
	ERR_CHECK_GOTO(r, exit);

	r = config(ADXL_MODE_OFF);
	ERR_CHECK_GOTO(r, exit);

	me.mode = ADXL_MODE_OFF;
	me.enabled = false;

	r = spi_teardown();
	ERR_CHECK_GOTO(r, exit);

	power(false);

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code adxl_deinit(void)
{
	err_code r;

	if (me.p_board == NULL)
		return EADXL_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return EADXL_MUTEX;

	r = spi_setup();
	ERR_CHECK_GOTO(r, exit);

	r = config(ADXL_MODE_OFF);
	ERR_CHECK_GOTO(r, exit);

	me.mode = ADXL_MODE_OFF;
	me.enabled = false;

	r = spi_teardown();
	ERR_CHECK_GOTO(r, exit);
	/* TODO: Unregister spim when it's implemented in spim driver. */

	r = gpio_irq_disable(me.p_board->pins.int1);
	ERR_CHECK_GOTO(r, exit);
	/* TODO: Unregister gpio irq when it's implemented in gpio
	 * irq driver.
	 */

	power(false);
	if (me.p_board->pins.vdd != BOARD_UNUSED_PIN)
		nrf_gpio_cfg_default(me.p_board->pins.vdd);

	me.p_board = NULL;

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code adxl_init(const struct adxl_board *p_board)
{
	struct adxl_chip_id chip_id;
	err_code r;
	uint8_t i;

	if (me.p_board != NULL)
		return ERROR_OK;

	if (p_board == NULL)
		return EADXL_INVALID_ARG;

	r = util_validate_pins((uint8_t *)&p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	me.p_board = p_board;

	if (me.p_board->pins.vdd != BOARD_UNUSED_PIN) {
		nrf_gpio_pin_clear(me.p_board->pins.vdd);
		nrf_gpio_cfg_output(me.p_board->pins.vdd);
	}

	r = twim_spim_irq_register_spim(me.p_board->p_spi, spim_irq_callback);
	ERR_CHECK_GOTO(r, error);

	reset_hard();

	r = spi_setup();
	ERR_CHECK_GOTO(r, error);

	r = reset_soft();
	ERR_CHECK_GOTO(r, error);

	/* Make sure we're talking to a supported device */
	for (i = 0; i < MAX_RETRY_COUNT; i++) {
		r = read_chip_id(&chip_id);
		if (r != ERROR_OK) {
			LOGE("read_chip_id, r=0x%08lX", r);
			continue;
		}

		if ((chip_id.dev == DEVICE_ID) && (chip_id.part == PART_ID))
			break;

		vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
	}

	if (i >= MAX_RETRY_COUNT) {
		r = EADXL_WRONG_ID;
		goto error;
	}

	r = spi_teardown();
	ERR_CHECK_GOTO(r, error);

	me.mode = ADXL_MODE_OFF;
	power(false);

	/* Configure interrupt pin and register an interrupt handler */
	nrf_gpio_cfg_input(me.p_board->pins.int1, NRF_GPIO_PIN_NOPULL);
	r = gpio_irq_register(me.p_board->pins.int1, GPIO_IRQ_POL_HIGH,
		irq_tophalf);
	ERR_CHECK_GOTO(r, error);

	r = gpio_irq_enable(me.p_board->pins.int1);
	ERR_CHECK_GOTO(r, error);

	r = settings_get_adxl_settings(&me.settings);
	ERR_CHECK_GOTO(r, error);

	if (me.settings.inactivity_time_s == FLASH_EMPTY_WORD)
		me.settings.inactivity_time_s = DEFAULT_INACT_TIME;

	if (me.settings.inactivity_threshold == FLASH_EMPTY_WORD)
		me.settings.inactivity_threshold = DEFAULT_INACT_THRESHOLD;

	if (me. settings.activity_time_s == FLASH_EMPTY_WORD)
		me.settings.activity_time_s = DEFAULT_ACT_TIME;

	if (me.settings.activity_threshold == FLASH_EMPTY_WORD)
		me.settings.activity_threshold = DEFAULT_ACT_THRESHOLD;

	me.mutex_handle = CREATE_STATIC_MUTEX(TASK_NAME);
	me.queue_handle = CREATE_STATIC_QUEUE_0(TASK_NAME, QUEUE_LEN);
	me.task_handle = CREATE_STATIC_TASK(task, TASK_NAME, TASK_PRIORITY);

	return tasktracker_register_task(me.task_handle);

error:
	me.p_board = NULL;

	return r;
}
