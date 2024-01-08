#include <app_util_platform.h>
#include <nrf_gpio.h>
#include <nrf_spim.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include "boards.h"
#include "drivers/can.h"
#include "drivers/gpio_irq.h"
#include "drivers/mcp25xxfd/mcp25xxfd.h"
#include "drivers/mcp25xxfd/mcp25xxfd_reg.h"
#include "drivers/mcp25xxfd/mcp25xxfd_def.h"
#include "drivers/twim_spim_irq.h"
#include "error.h"
#include "freertos_static.h"
#include "freertos_tasktracker.h"
#include "power.h"
#include "session_mgr.h"
#include "util.h"

#define DRV_NAME					MCP25
#define LOG_TAG						STRINGIFY(DRV_NAME)
#include "log.h"

#define TASK_NAME					DRV_NAME
#define TASK_PRIORITY				2
#define TASK_STACK_DEPTH			256
#define QUEUE_LEN					4

#define SPI_COMPLETION_TMO_MS		150
#define SPI_DEFAULT_BUFFER_LENGTH	96
#define SPI_MAX_BUFFER_LENGTH		256
#define SPI_DMA_MAX_LEN				255
#define MAX_MSG_SIZE				76
#define CRCBASE						0xFFFF
#define CRCUPPER					1

#define CAN_CLIENTS_MAX				5
#define CAN_RX_FIFO_CH				MCP25_CAN_FIFO_CH1
#define CAN_TX_FIFO_CH				MCP25_CAN_FIFO_CH2
#define CAN_OP_MODE_SWITCH_TIME_MS	1
#define MAX_NBR_RETRIES				5
#define RETRY_DELAY_MS				100
#define CAN_MAX_PAYLOAD_BYTES		8
#define RAM_EMPTY_BYTE				0xFF

/* FIFO size will be 1 entry greater than the config value.
 * See "MCP2518FD - External CAN FD Controller with SPI Interface"
 * ch 3.1 register 3-29.
 */
#define FIFO_SIZE_OFFSET			1

#define CS_SET_ACTIVE()				nrf_gpio_pin_clear(me.p_board->pins.spi_cs)
#define CS_SET_INACTIVE()			nrf_gpio_pin_set(me.p_board->pins.spi_cs)
#define STDBY_SET_ACTIVE()			nrf_gpio_pin_set(me.p_board->pins.stdby)
#define STDBY_SET_INACTIVE()		nrf_gpio_pin_clear(me.p_board->pins.stdby)

enum tx_attempts {
	TX_ATTEMPTS_DISABLED = 0,
	TX_ATTEMPTS_3,
	TX_ATTEMPTS_UNLIMITED,
};

enum queue_item_type {
	QUEUE_ITEM_CAN_RX_IRQ = 0,
	NBR_OF_QUEUE_ITEM_TYPES,
};

STATIC_MUTEX(DRV_NAME);
STATIC_TASK(TASK_NAME, TASK_STACK_DEPTH);
STATIC_QUEUE(DRV_NAME, QUEUE_LEN, enum queue_item_type);

/* Control Register Reset Values up to FIFOs */
static const uint32_t can_ctrl_rst_vals[] = {
	/* Address 0x000 to 0x00C */
	0x04980760, 0x003E0F0F, 0x000E0303, 0x00021000,
	/* Address 0x010 to 0x01C */
	0x00000000, 0x00000000, 0x40400040, 0x00000000,
	/* Address 0x020 to 0x02C */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Address 0x030 to 0x03C */
	0x00000000, 0x00200000, 0x00000000, 0x00000000,
	/* Address 0x040 to 0x04C */
	0x00000400, 0x00000000, 0x00000000, 0x00000000
};

/* FIFO Register Reset Values */
static const uint32_t can_fifo_rst_vals[] = {
	0x00600400, 0x00000000, 0x00000000
};

static struct {
	const struct mcp25xxfd_board   *p_board;
	SemaphoreHandle_t				mutex_handle;
	QueueHandle_t					queue_handle;
	TaskHandle_t					task_handle;
	TaskHandle_t					notify_spi_handle;
	struct can_rx_client			can_client[CAN_CLIENTS_MAX];
	uint8_t							tx_buf[SPI_MAX_BUFFER_LENGTH];
	uint8_t							rx_buf[SPI_MAX_BUFFER_LENGTH];
	uint8_t							trx_buf[CAN_MAX_PAYLOAD_BYTES];
	bool							initialized;
} me;

static void gpio_irq_callback(uint8_t gpio, uint32_t value)
{
	enum queue_item_type item;
	BaseType_t yield_req;

	yield_req = pdFALSE;

	if (me.p_board && (gpio == me.p_board->pins.int_1) && (value == 0) &&
			me.queue_handle) {
		item = QUEUE_ITEM_CAN_RX_IRQ;
		xQueueSendToBackFromISR(me.queue_handle, &item, &yield_req);
	}

	portYIELD_FROM_ISR(yield_req);
}

static void spim_irq_callback(void)
{
	BaseType_t yield_req = pdFALSE;

	if (nrf_spim_event_check(NRF_SPIM2, NRF_SPIM_EVENT_END)) {
		nrf_spim_event_clear(NRF_SPIM2, NRF_SPIM_EVENT_END);
		if (me.notify_spi_handle != NULL)
			vTaskNotifyGiveFromISR(me.notify_spi_handle, &yield_req);
	}

	portYIELD_FROM_ISR(yield_req);
}

static err_code spi_transfer(const uint8_t *p_tx, uint8_t * const p_rx,
	const uint16_t trx_size)
{
	const uint8_t *p_tx_buf;
	uint16_t trx_bytes_left;
	uint8_t trx_chunk;
	uint8_t *p_rx_buf;
	err_code r;

	me.notify_spi_handle = xTaskGetCurrentTaskHandle();
	trx_bytes_left = trx_size;
	p_tx_buf = p_tx;
	p_rx_buf = p_rx;
	r = ERROR_OK;

	CS_SET_ACTIVE();

	do {
		trx_chunk = MIN(SPI_DMA_MAX_LEN, trx_bytes_left);

		nrf_spim_tx_buffer_set(me.p_board->p_instance, p_tx_buf,
			(p_tx_buf ? trx_bytes_left : 0));
		nrf_spim_rx_buffer_set(me.p_board->p_instance, p_rx_buf,
			(p_rx_buf ? trx_bytes_left : 0));

		nrf_spim_event_clear(me.p_board->p_instance, NRF_SPIM_EVENT_END);
		nrf_spim_task_trigger(me.p_board->p_instance, NRF_SPIM_TASK_START);

		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
				pdMS_TO_TICKS(SPI_COMPLETION_TMO_MS)) != pdTRUE) {
			nrf_spim_task_trigger(me.p_board->p_instance, NRF_SPIM_TASK_STOP);
			r = EMCP25XXFD_SPI_TMO;
			break;
		}

		if (p_tx_buf)
			p_tx_buf += trx_chunk;
		if (p_rx_buf)
			p_rx_buf += trx_chunk;
		trx_bytes_left -= trx_chunk;
	} while (trx_bytes_left > 0);

	CS_SET_INACTIVE();

	me.notify_spi_handle = NULL;
	return r;
}

static err_code spi_init(void)
{
	err_code r;

	CS_SET_INACTIVE();
	nrf_gpio_cfg_output(me.p_board->pins.spi_cs);

	nrf_gpio_cfg_output(me.p_board->pins.spi_clk);
	nrf_gpio_cfg_output(me.p_board->pins.spi_mosi);
	nrf_gpio_cfg_input(me.p_board->pins.spi_miso, NRF_GPIO_PIN_NOPULL);

	nrf_spim_pins_set(me.p_board->p_instance, me.p_board->pins.spi_clk,
		me.p_board->pins.spi_mosi, me.p_board->pins.spi_miso);

	nrf_spim_frequency_set(me.p_board->p_instance, me.p_board->frequency);

	/* CLK Idle High (active low) & output data phase on trailing CLK flank */
	nrf_spim_configure(me.p_board->p_instance,
		NRF_SPIM_MODE_3, NRF_SPIM_BIT_ORDER_MSB_FIRST);

	r = twim_spim_irq_register_spim(me.p_board->p_instance, spim_irq_callback);
	ERR_CHECK(r);

	nrf_spim_int_enable(me.p_board->p_instance, NRF_SPIM_INT_END_MASK);

	nrf_spim_enable(me.p_board->p_instance);

	NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(me.p_board->p_instance),
			APP_IRQ_PRIORITY_LOW);
	NRFX_IRQ_ENABLE(nrfx_get_irq_number(me.p_board->p_instance));

	return ERROR_OK;
}

static uint32_t dlc_to_data_bytes(enum mcp25_can_dlc dlc)
{
	uint32_t bytes_in_obj;

	/* TODO: Investigate if needed, else remove. */
	__NOP();
	__NOP();

	if (dlc < MCP25_CAN_DLC_12) {
		bytes_in_obj = dlc;
	} else {
		switch (dlc) {
		case MCP25_CAN_DLC_12:
			bytes_in_obj = 12;
			break;
		case MCP25_CAN_DLC_16:
			bytes_in_obj = 16;
			break;
		case MCP25_CAN_DLC_20:
			bytes_in_obj = 20;
			break;
		case MCP25_CAN_DLC_24:
			bytes_in_obj = 24;
			break;
		case MCP25_CAN_DLC_32:
			bytes_in_obj = 32;
			break;
		case MCP25_CAN_DLC_48:
			bytes_in_obj = 48;
			break;
		case MCP25_CAN_DLC_64:
			bytes_in_obj = 64;
			break;
		default:
			break;
		}
	}

	return bytes_in_obj;
}

static err_code sw_reset(void)
{
	me.tx_buf[0] = (MCP25_INST_RESET << 4);
	me.tx_buf[1] = 0;

	return spi_transfer(me.tx_buf, NULL, 2);
}

static err_code read_byte(uint16_t address, uint8_t *p_byte)
{
	uint16_t spi_trx_size;
	err_code r;

	if (!p_byte)
		return EMCP25XXFD_INVALID_ARG;

	spi_trx_size = 3;
	memset(me.tx_buf, 0, spi_trx_size);
	me.tx_buf[0] = ((MCP25_INST_READ << 4) + ((address >> 8) & 0xF));
	me.tx_buf[1] = (address & 0xFF);

	r = spi_transfer(me.tx_buf, me.rx_buf, spi_trx_size);
	ERR_CHECK(r);

	*p_byte = me.rx_buf[2];
	return ERROR_OK;
}

static err_code write_byte(uint16_t address, uint8_t byte)
{
	me.tx_buf[0] = ((MCP25_INST_WRITE << 4) + ((address >> 8) & 0xF));
	me.tx_buf[1] = (address & 0xFF);
	me.tx_buf[2] = byte;

	return spi_transfer(me.tx_buf, NULL, 3);
}

static err_code read_word(uint16_t address, uint32_t *p_word)
{
	uint16_t spi_trx_size;
	uint32_t temp;
	err_code r;
	uint8_t i;

	if (!p_word)
		return EMCP25XXFD_INVALID_ARG;

	spi_trx_size = 6;
	memset(me.tx_buf, 0, spi_trx_size);
	me.tx_buf[0] = ((MCP25_INST_READ << 4) + ((address >> 8) & 0xF));
	me.tx_buf[1] = (address & 0xFF);

	r = spi_transfer(me.tx_buf, me.rx_buf, spi_trx_size);
	ERR_CHECK(r);

	/* TODO: Replace with *p_word = *(uint32_t*)&me.rx_buf[2]; */
	*p_word = 0;
	for (i = 2; i < spi_trx_size; i++) {
		temp = (uint32_t)me.rx_buf[i];
		*p_word += (temp << ((i - 2) * 8));
	}

	return ERROR_OK;
}

static err_code write_word(uint16_t address, uint32_t word)
{
	uint8_t i;

	me.tx_buf[0] = ((MCP25_INST_WRITE << 4) + ((address >> 8) & 0xF));
	me.tx_buf[1] = (address & 0xFF);

	/* TODO: Use memcpy instead */
	for (i = 0; i < 4; i++)
		me.tx_buf[i + 2] = ((word >> (i * 8)) & 0xFF);

	return spi_transfer(me.tx_buf, NULL, 6);
}

static err_code read_halfword(uint16_t address, uint16_t *p_word)
{
	uint16_t spi_trx_size;
	uint32_t temp;
	err_code r;
	uint8_t i;

	if (!p_word)
		return EMCP25XXFD_INVALID_ARG;

	spi_trx_size = 4;
	memset(me.tx_buf, 0, spi_trx_size);
	me.tx_buf[0] = ((MCP25_INST_READ << 4) + ((address >> 8) & 0xF));
	me.tx_buf[1] = (address & 0xFF);

	r = spi_transfer(me.tx_buf, me.rx_buf, spi_trx_size);
	ERR_CHECK(r);

	/* TODO: Replace with *p_word = *(uint32_t*)&me.rx_buf[2]; */
	*p_word = 0;
	for (i = 2; i < spi_trx_size; i++) {
		temp = (uint32_t)me.rx_buf[i];
		*p_word += (temp << ((i - 2) * 8));
	}

	return ERROR_OK;
}

static err_code write_halfword(uint16_t address, uint16_t word)
{
	uint8_t i;

	me.tx_buf[0] = ((MCP25_INST_WRITE << 4) + ((address >> 8) & 0xF));
	me.tx_buf[1] = (address & 0xFF);

	/* TODO: Use memcpy instead */
	for (i = 0; i < 2; i++)
		me.tx_buf[i + 2] = ((word >> (i * 8)) & 0xFF);

	return spi_transfer(me.tx_buf, NULL, 4);
}

static err_code read_byte_array(uint16_t address, uint8_t *p_buf,
	uint16_t size)
{
	uint16_t spi_trx_size;
	err_code r;

	if (!p_buf || !size)
		return EMCP25XXFD_INVALID_ARG;

	spi_trx_size = (size + 2);
	if (spi_trx_size > SPI_MAX_BUFFER_LENGTH)
		return EMCP25XXFD_BUF_OVERSIZE;

	memset(me.tx_buf, 0, spi_trx_size);
	me.tx_buf[0] = ((MCP25_INST_READ << 4) + ((address >> 8) & 0xF));
	me.tx_buf[1] = (address & 0xFF);

	r = spi_transfer(me.tx_buf, me.rx_buf, spi_trx_size);
	ERR_CHECK(r);

	memcpy(p_buf, &me.rx_buf[2], size);
	return ERROR_OK;
}

static err_code write_byte_array(uint16_t address, uint8_t *p_buf,
	uint16_t size)
{
	uint16_t spi_trx_size;

	if (!p_buf || !size)
		return EMCP25XXFD_INVALID_ARG;

	spi_trx_size = (size + 2);
	if (spi_trx_size > SPI_MAX_BUFFER_LENGTH)
		return EMCP25XXFD_BUF_OVERSIZE;

	me.tx_buf[0] = ((MCP25_INST_WRITE << 4) + ((address >> 8) & 0xF));
	me.tx_buf[1] = (address & 0xFF);
	memcpy(&me.tx_buf[2], p_buf, size);

	return spi_transfer(me.tx_buf, NULL, spi_trx_size);
}

static err_code read_word_array(uint16_t address, uint32_t *p_buf,
	uint16_t size)
{
	uint16_t spi_trx_size;
	union mcp25_word w;
	err_code r;
	uint16_t i;
	uint16_t j;
	uint16_t n;

	if (!p_buf || !size)
		return EMCP25XXFD_INVALID_ARG;

	spi_trx_size = (size * 4 + 2);
	if (spi_trx_size > SPI_MAX_BUFFER_LENGTH)
		return EMCP25XXFD_BUF_OVERSIZE;

	memset(me.tx_buf, 0, spi_trx_size);
	me.tx_buf[0] = (MCP25_INST_READ << 4) + ((address >> 8) & 0xF);
	me.tx_buf[1] = address & 0xFF;

	r = spi_transfer(me.tx_buf, me.rx_buf, spi_trx_size);
	ERR_CHECK(r);

	/* Convert byte-array to word-array */
	n = 2;
	for (i = 0; i < size; i++) {
		w.word = 0;
		for (j = 0; j < 4; j++, n++)
			w.byte[j] = me.rx_buf[n];
		p_buf[i] = w.word;
	}

	return ERROR_OK;
}

/* TODO: Go through all function headers and apply const directive to all
 * where applicable.
 */
static err_code mcp25_tx_channel_configure(enum mcp25_can_fifo_channel channel,
	struct mcp25_can_tx_fifo_cfg *p_cfg)
{
	union mcp25_reg_cififocon reg;
	uint16_t a;

	reg.word = can_fifo_rst_vals[0];

	reg.tx_bf.tx_enable = 1;
	reg.tx_bf.fifo_size = (p_cfg->fifo_size - FIFO_SIZE_OFFSET);
	reg.tx_bf.payload_size = p_cfg->payload_size;
	reg.tx_bf.tx_attempts = p_cfg->tx_attempts;
	reg.tx_bf.tx_prio = p_cfg->tx_prio;
	reg.tx_bf.rtr_enable = p_cfg->rtr_enable;

	a = (MCP25_REG_CiFIFOCON + (channel * CiFIFO_OFFSET));
	return write_word(a, reg.word);
}

static err_code mcp25_tx_channel_configure_obj_rst(
	struct mcp25_can_tx_fifo_cfg *p_cfg)
{
	union mcp25_reg_cififocon reg;

	reg.word = can_fifo_rst_vals[0];

	p_cfg->rtr_enable = reg.tx_bf.rtr_enable;
	p_cfg->tx_prio = reg.tx_bf.tx_prio;
	p_cfg->tx_attempts = reg.tx_bf.tx_attempts;
	p_cfg->fifo_size = reg.tx_bf.fifo_size;
	p_cfg->payload_size = reg.tx_bf.payload_size;
	return ERROR_OK;
}

static err_code mcp25_tx_channel_upd(enum mcp25_can_fifo_channel channel,
	bool flush)
{
	union mcp25_reg_cififocon reg;
	uint16_t a;

	reg.word = 0;
	reg.tx_bf.uinc = 1;
	if (flush)
		reg.tx_bf.tx_req = 1;

	a = (MCP25_REG_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1);
	return write_byte(a, reg.byte[1]);
}

static err_code mcp25_tx_channel_load(enum mcp25_can_fifo_channel channel,
	union mcp25_can_tx_msgobj *p_obj, const uint8_t * const p_tx,
	const uint32_t tx_size, bool flush)
{
	__attribute__((unused)) union mcp25_reg_cififosta reg_fifosta;
	union mcp25_reg_cififocon reg_fifocon;
	union mcp25_reg_cififoua reg_fifoua;
	uint8_t tx_buf[MAX_MSG_SIZE];
	uint32_t bytes_in_obj;
	uint32_t fifo_reg[3];
	uint16_t a;
	uint16_t n;
	uint8_t i;
	uint8_t j;
	err_code r;

	a = (MCP25_REG_CiFIFOCON + (channel * CiFIFO_OFFSET));
	r = read_word_array(a, fifo_reg, ARRAY_SIZE(fifo_reg));
	ERR_CHECK(r);

	/* Check that it is a transmit buffer */
	reg_fifocon.word = fifo_reg[0];
	if (!reg_fifocon.tx_bf.tx_enable)
		return EMCP25XXFD_TX_NOT_ENABLED;

	/* Check that DLC is big enough for data */
	bytes_in_obj = dlc_to_data_bytes((enum mcp25_can_dlc) p_obj->bf.ctrl.DLC);
	if (bytes_in_obj < tx_size)
		return EMCP25XXFD_TX_DLC_TOO_SMALL;

	reg_fifosta.word = fifo_reg[1];
	reg_fifoua.word = fifo_reg[2];

#ifdef USERADDRESS_TIMES_FOUR
	a = (4 * reg_fifoua.bf.usr_addr);
#else
	a = reg_fifoua.bf.usr_addr;
#endif
	a += MCP25_RAMADDR_START;

	/* TODO: use memcpy instead */
	tx_buf[0] = p_obj->byte[0];
	tx_buf[1] = p_obj->byte[1];
	tx_buf[2] = p_obj->byte[2];
	tx_buf[3] = p_obj->byte[3];
	tx_buf[4] = p_obj->byte[4];
	tx_buf[5] = p_obj->byte[5];
	tx_buf[6] = p_obj->byte[6];
	tx_buf[7] = p_obj->byte[7];

	/* TODO: use memcpy instead */
	for (i = 0; i < tx_size; i++)
		tx_buf[i + 8] = p_tx[i];

	/* Make sure to write a multiple of 4 bytes to RAM */
	/* TODO: Verify that the byte off-set calculcations below are correct,
	 * adjust where needed.
	 */
	n = 0;
	if (tx_size % 4) {
		/* Need to add bytes */
		n = (4 - (tx_size % 4));
		i = (tx_size + 8);

		for (j = 0; j < n; j++)
			tx_buf[i + 8 + j] = 0;
	}

	r = write_byte_array(a, tx_buf, tx_size + 8 + n);
	ERR_CHECK(r);

	return mcp25_tx_channel_upd(channel, flush);
}

static err_code mcp25_tx_channel_evt_get(enum mcp25_can_fifo_channel channel,
	enum mcp25_can_tx_fifo_event *p_flags)
{
	union mcp25_reg_cififosta reg;
	err_code r;
	uint16_t a;

	reg.word = 0;
	a = (MCP25_REG_CiFIFOSTA + (channel * CiFIFO_OFFSET));
	r = read_byte(a, &reg.byte[0]);
	ERR_CHECK(r);

	*p_flags = (enum mcp25_can_tx_fifo_event)
		(reg.byte[0] & MCP25_CAN_TX_FIFO_ALL_EVENTS);
	return ERROR_OK;
}

static err_code mcp25_tx_channel_evt_enable(enum mcp25_can_fifo_channel channel,
	enum mcp25_can_tx_fifo_event flags)
{
	union mcp25_reg_cififocon reg;
	uint16_t a;
	err_code r;

	reg.word = 0;
	a = (MCP25_REG_CiFIFOCON + (channel * CiFIFO_OFFSET));
	r = read_byte(a, &reg.byte[0]);
	ERR_CHECK(r);

	reg.byte[0] |= (flags & MCP25_CAN_TX_FIFO_ALL_EVENTS);
	return write_byte(a, reg.byte[0]);
}

static err_code mcp25_rx_channel_configure(enum mcp25_can_fifo_channel channel,
	struct mcp25_can_rx_fifo_cfg *p_cfg)
{
	union mcp25_reg_cififocon reg;
	uint16_t a;

	if (channel == MCP25_CAN_TXQUEUE_CH0)
		return EMCP25XXFD_RX_CAN_TXQUEUE_CH0;

	/* Setup FIFO */
	reg.word = can_fifo_rst_vals[0];

	reg.rx_bf.tx_enable = 0;
	reg.rx_bf.fifo_size = (p_cfg->fifo_size - FIFO_SIZE_OFFSET);
	reg.rx_bf.payload_size = p_cfg->payload_size;
	reg.rx_bf.rx_ts_enable = p_cfg->rx_ts_enable;

	a = (MCP25_REG_CiFIFOCON + (channel * CiFIFO_OFFSET));
	return write_word(a, reg.word);
}

static err_code mcp25_rx_channel_cfg_obj_rst(
	struct mcp25_can_rx_fifo_cfg *p_cfg)
{
	union mcp25_reg_cififocon reg;

	reg.word = can_fifo_rst_vals[0];

	p_cfg->fifo_size = reg.rx_bf.fifo_size;
	p_cfg->payload_size = reg.rx_bf.payload_size;
	p_cfg->rx_ts_enable = reg.rx_bf.rx_ts_enable;
	return ERROR_OK;
}

static err_code mcp25_rx_channel_upd(enum mcp25_can_fifo_channel channel)
{
	union mcp25_reg_cififocon reg;
	uint16_t a;

	reg.word = 0;
	reg.rx_bf.uinc = 1;

	a = (MCP25_REG_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1);
	return write_byte(a, reg.byte[1]);
}

static err_code mcp25_rx_msg_get(enum mcp25_can_fifo_channel channel,
	union mcp25_can_rx_msgobj *p_obj, uint8_t *p_rx, uint8_t rx_bytes)
{
	__attribute__((unused)) union mcp25_reg_cififosta reg_fifosta;
	union mcp25_reg_cififocon reg_fifocon;
	union mcp25_reg_cififoua reg_fifoua;
	uint8_t ba[MAX_MSG_SIZE];
	uint32_t fifo_reg[3];
	union mcp25_word w;
	uint8_t n;
	uint8_t i;
	uint16_t a;
	err_code r;

	a = (MCP25_REG_CiFIFOCON + (channel * CiFIFO_OFFSET));
	r = read_word_array(a, fifo_reg, ARRAY_SIZE(fifo_reg));
	ERR_CHECK(r);

	reg_fifocon.word = fifo_reg[0];
	if (reg_fifocon.tx_bf.tx_enable)
		return EMCP25XXFD_RX_NOT_ENABLED;

	reg_fifosta.word = fifo_reg[1];
	reg_fifoua.word = fifo_reg[2];

#ifdef USERADDRESS_TIMES_FOUR
	a = (4 * reg_fifoua.bf.usr_addr);
#else
	a = reg_fifoua.bf.usr_addr;
#endif
	a += MCP25_RAMADDR_START;

	/* Number of bytes to read plus 8 bytes header */
	n = (rx_bytes + 8);

	/* Four bvtes for the time stamp. */
	if (reg_fifocon.rx_bf.rx_ts_enable)
		n += 4;

	/* Make sure we read a multiple of 4 bytes from RAM */
	if (n % 4)
		n = (n + 4 - (n % 4));

	if (n > MAX_MSG_SIZE)
		n = MAX_MSG_SIZE;

	r = read_byte_array(a, ba, n);
	ERR_CHECK(r);

	/* Assign message header */
	w.byte[0] = ba[0];
	w.byte[1] = ba[1];
	w.byte[2] = ba[2];
	w.byte[3] = ba[3];
	p_obj->word[0] = w.word;

	w.byte[0] = ba[4];
	w.byte[1] = ba[5];
	w.byte[2] = ba[6];
	w.byte[3] = ba[7];
	p_obj->word[1] = w.word;

	if (reg_fifocon.rx_bf.rx_ts_enable) {
		w.byte[0] = ba[8];
		w.byte[1] = ba[9];
		w.byte[2] = ba[10];
		w.byte[3] = ba[11];
		p_obj->word[2] = w.word;

		for (i = 0; i < rx_bytes; i++)
			p_rx[i] = ba[i + 12];
	} else {
		p_obj->word[2] = 0;
		for (i = 0; i < rx_bytes; i++)
			p_rx[i] = ba[i + 8];
	}

	return mcp25_rx_channel_upd(channel);
}

static err_code mcp25_rx_channel_evt_get(enum mcp25_can_fifo_channel channel,
	enum mcp25_can_rx_fifo_event *p_flags)
{
	union mcp25_reg_cififosta reg;
	uint16_t a;
	err_code r;

	if (channel == MCP25_CAN_TXQUEUE_CH0)
		return EMCP25XXFD_RX_CAN_TXQUEUE_CH0;

	reg.word = 0;
	a = (MCP25_REG_CiFIFOSTA + (channel * CiFIFO_OFFSET));
	r = read_byte(a, &reg.byte[0]);
	ERR_CHECK(r);

	*p_flags = (enum mcp25_can_rx_fifo_event)
		(reg.byte[0] & MCP25_CAN_RX_FIFO_ALL_EVENTS);
	return ERROR_OK;
}

static err_code mcp25_rx_channel_evt_enable(enum mcp25_can_fifo_channel channel,
	enum mcp25_can_rx_fifo_event flags)
{
	union mcp25_reg_cififocon reg;
	uint16_t a;
	err_code r;

	if (channel == MCP25_CAN_TXQUEUE_CH0)
		return EMCP25XXFD_RX_CAN_TXQUEUE_CH0;

	reg.word = 0;
	a = (MCP25_REG_CiFIFOCON + (channel * CiFIFO_OFFSET));
	r = read_byte(a, &reg.byte[0]);
	ERR_CHECK(r);

	reg.byte[0] |= (flags & MCP25_CAN_RX_FIFO_ALL_EVENTS);
	return write_byte(a, reg.byte[0]);
}

static err_code mcp25_filter_obj_configure(enum mcp25_can_filter filter,
	struct mcp25_can_filterobj_id *p_id)
{
	union mcp25_reg_cifltobj f_obj;
	uint16_t a;

	f_obj.word = 0;
	f_obj.bf = *p_id;

	a = (MCP25_REG_CiFLTOBJ + (filter * CiFILTER_OFFSET));
	return write_word(a, f_obj.word);
}

static err_code mcp25_filter_msk_configure(enum mcp25_can_filter filter,
	struct mcp25_can_maskobj_id *p_msk)
{
	union mcp25_reg_cimask m_obj;
	uint16_t a;

	m_obj.word = 0;
	m_obj.bf = *p_msk;

	a = (MCP25_REG_CiMASK + (filter * CiFILTER_OFFSET));
	return write_word(a, m_obj.word);
}

static err_code mcp25_filter_to_fifo_link(enum mcp25_can_filter filter,
	enum mcp25_can_fifo_channel channel, bool enable)
{
	union mcp25_reg_cifltcon_byte f_ctrl;
	uint16_t a;

	f_ctrl.bf.enable = enable;
	f_ctrl.bf.buf_ptr = channel;

	a = (MCP25_REG_CiFLTCON + filter);
	return write_byte(a, f_ctrl.byte);
}

static err_code mcp25_filter_disable(enum mcp25_can_filter filter)
{
	union mcp25_reg_cifltcon_byte f_ctrl;
	uint16_t a;
	err_code r;

	a = (MCP25_REG_CiFLTCON + filter);
	r = read_byte(a, &f_ctrl.byte);
	ERR_CHECK(r);

	f_ctrl.bf.enable = 0;
	return write_byte(a, f_ctrl.byte);
}

static err_code mcp25_module_evt_enable(enum mcp25_can_module_event flags)
{
	union mcp25_reg_ciint_enable reg;
	err_code r;

	reg.word = 0;
	r = read_halfword(MCP25_REG_CiINTENABLE, &reg.word);
	ERR_CHECK(r);

	reg.word |= (flags & MCP25_CAN_ALL_EVENTS);
	return write_halfword(MCP25_REG_CiINTENABLE, reg.word);
}

static err_code mcp25_ecc_enable(void)
{
	err_code r;
	uint8_t d;

	r = read_byte(MCP25_REG_ECCCON, &d);
	ERR_CHECK(r);

	d |= 0x01;
	return write_byte(MCP25_REG_ECCCON, d);
}

static err_code mcp25_init_ram(uint8_t d)
{
	uint8_t txd[SPI_DEFAULT_BUFFER_LENGTH];
	uint16_t a;
	uint32_t i;
	err_code r;

	for (i = 0; i < SPI_DEFAULT_BUFFER_LENGTH; i++)
		txd[i] = d;

	a = MCP25_RAMADDR_START;
	for (i = 0; i < (MCP25_RAM_SIZE / SPI_DEFAULT_BUFFER_LENGTH); i++) {
		r = write_byte_array(a, txd, SPI_DEFAULT_BUFFER_LENGTH);
		ERR_CHECK(r);
		a += SPI_DEFAULT_BUFFER_LENGTH;
	}

	return ERROR_OK;
}

static err_code mcp25_bit_time_cfg_nom_40MHz(
	enum mcp25_can_bittime_setup bit_time)
{
	union mcp25_reg_cinbtcfg reg;

	reg.word = can_ctrl_rst_vals[MCP25_REG_CiNBTCFG / 4];

	switch (bit_time) {
	case MCP25_CAN_500K_1M:
	case MCP25_CAN_500K_2M:
	case MCP25_CAN_500K_3M:
	case MCP25_CAN_500K_4M:
	case MCP25_CAN_500K_5M:
	case MCP25_CAN_500K_6M7:
	case MCP25_CAN_500K_8M:
	case MCP25_CAN_500K_10M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 62;
		reg.bf.tseg2 = 15;
		reg.bf.sjw = 15;
		break;
	case MCP25_CAN_250K_500K:
	case MCP25_CAN_250K_833K:
	case MCP25_CAN_250K_1M:
	case MCP25_CAN_250K_1M5:
	case MCP25_CAN_250K_2M:
	case MCP25_CAN_250K_3M:
	case MCP25_CAN_250K_4M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 126;
		reg.bf.tseg2 = 31;
		reg.bf.sjw = 31;
		break;
	case MCP25_CAN_1000K_4M:
	case MCP25_CAN_1000K_8M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 30;
		reg.bf.tseg2 = 7;
		reg.bf.sjw = 7;
		break;
	case MCP25_CAN_125K_500K:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 254;
		reg.bf.tseg2 = 63;
		reg.bf.sjw = 63;
		break;
	default:
		return EMCP25XXFD_CAN_BIT_TIME_NOM;
	}

	return write_word(MCP25_REG_CiNBTCFG, reg.word);
}

static err_code mcp25_bit_time_cfg_data_40MHz(
	enum mcp25_can_bittime_setup bit_time,
	enum mcp25_can_ssp_mode ssp_mode)
{
	union mcp25_reg_cidbtcfg reg_dbtcfg;
	union mcp25_reg_citdc reg_tdc;
	err_code r;

	reg_dbtcfg.word = can_ctrl_rst_vals[MCP25_REG_CiDBTCFG / 4];
	reg_tdc.word = 0;
	/* Configure Bit time and sample point */
	reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_AUTO;

	switch (bit_time) {
	case MCP25_CAN_250K_1M:
	case MCP25_CAN_500K_1M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 30;
		reg_dbtcfg.bf.tseg2 = 7;
		reg_dbtcfg.bf.sjw = 7;
		reg_tdc.bf.tdc_offset = 31;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_2M:
	case MCP25_CAN_500K_2M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 14;
		reg_dbtcfg.bf.tseg2 = 3;
		reg_dbtcfg.bf.sjw = 3;
		reg_tdc.bf.tdc_offset = 15;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_3M:
	case MCP25_CAN_500K_3M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 8;
		reg_dbtcfg.bf.tseg2 = 2;
		reg_dbtcfg.bf.sjw = 2;
		reg_tdc.bf.tdc_offset = 9;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_4M:
	case MCP25_CAN_500K_4M:
	case MCP25_CAN_1000K_4M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 6;
		reg_dbtcfg.bf.tseg2 = 1;
		reg_dbtcfg.bf.sjw = 1;
		reg_tdc.bf.tdc_offset = 7;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_500K_5M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 4;
		reg_dbtcfg.bf.tseg2 = 1;
		reg_dbtcfg.bf.sjw = 1;
		reg_tdc.bf.tdc_offset = 5;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_500K_6M7:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 3;
		reg_dbtcfg.bf.tseg2 = 0;
		reg_dbtcfg.bf.sjw = 0;
		reg_tdc.bf.tdc_offset = 4;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_500K_8M:
	case MCP25_CAN_1000K_8M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 2;
		reg_dbtcfg.bf.tseg2 = 0;
		reg_dbtcfg.bf.sjw = 0;
		reg_tdc.bf.tdc_offset = 3;
		reg_tdc.bf.tdc_val = 1;
		break;
	case MCP25_CAN_500K_10M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 1;
		reg_dbtcfg.bf.tseg2 = 0;
		reg_dbtcfg.bf.sjw = 0;
		reg_tdc.bf.tdc_offset = 2;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_500K:
	case MCP25_CAN_125K_500K:
		reg_dbtcfg.bf.brp = 1;
		reg_dbtcfg.bf.tseg1 = 30;
		reg_dbtcfg.bf.tseg2 = 7;
		reg_dbtcfg.bf.sjw = 7;
		reg_tdc.bf.tdc_offset = 31;
		reg_tdc.bf.tdc_val = 0;
		reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_OFF;
		break;
	case MCP25_CAN_250K_833K:
		reg_dbtcfg.bf.brp = 1;
		reg_dbtcfg.bf.tseg1 = 17;
		reg_dbtcfg.bf.tseg2 = 4;
		reg_dbtcfg.bf.sjw = 4;
		reg_tdc.bf.tdc_offset = 18;
		reg_tdc.bf.tdc_val = 0;
		reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_OFF;
		break;
	case MCP25_CAN_250K_1M5:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 18;
		reg_dbtcfg.bf.tseg2 = 5;
		reg_dbtcfg.bf.sjw = 5;
		reg_tdc.bf.tdc_offset = 19;
		reg_tdc.bf.tdc_val = 0;
		break;
	default:
		return EMCP25XXFD_CAN_BIT_TIME_DAT;
	}

	r = write_word(MCP25_REG_CiDBTCFG, reg_dbtcfg.word);
	ERR_CHECK(r);

/* TODO: Remove or support via board structure */
#ifdef REV_A
	reg_tdc.bf.tdc_offset = 0;
	reg_tdc.bf.tdc_val = 0;
#endif

	return write_word(MCP25_REG_CiTDC, reg_tdc.word);
}

static err_code mcp25_bit_time_cfg_nom_20MHz(
	enum mcp25_can_bittime_setup bit_time)
{
	union mcp25_reg_cinbtcfg reg;

	reg.word = can_ctrl_rst_vals[MCP25_REG_CiNBTCFG / 4];

	switch (bit_time) {
	case MCP25_CAN_500K_1M:
	case MCP25_CAN_500K_2M:
	case MCP25_CAN_500K_4M:
	case MCP25_CAN_500K_5M:
	case MCP25_CAN_500K_6M7:
	case MCP25_CAN_500K_8M:
	case MCP25_CAN_500K_10M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 30;
		reg.bf.tseg2 = 7;
		reg.bf.sjw = 7;
		break;
	case MCP25_CAN_250K_500K:
	case MCP25_CAN_250K_833K:
	case MCP25_CAN_250K_1M:
	case MCP25_CAN_250K_1M5:
	case MCP25_CAN_250K_2M:
	case MCP25_CAN_250K_3M:
	case MCP25_CAN_250K_4M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 62;
		reg.bf.tseg2 = 15;
		reg.bf.sjw = 15;
		break;
	case MCP25_CAN_1000K_4M:
	case MCP25_CAN_1000K_8M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 14;
		reg.bf.tseg2 = 3;
		reg.bf.sjw = 3;
		break;
	case MCP25_CAN_125K_500K:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 126;
		reg.bf.tseg2 = 31;
		reg.bf.sjw = 31;
		break;
	default:
		return EMCP25XXFD_CAN_BIT_TIME_NOM;
	}

	return write_word(MCP25_REG_CiNBTCFG, reg.word);
}

static err_code mcp25_bit_time_cfg_data_20MHz(
	enum mcp25_can_bittime_setup bit_time,
	enum mcp25_can_ssp_mode ssp_mode)
{
	union mcp25_reg_cidbtcfg reg_dbtcfg;
	union mcp25_reg_citdc reg_tdc;
	err_code r;

	reg_dbtcfg.word = can_ctrl_rst_vals[MCP25_REG_CiDBTCFG / 4];
	reg_tdc.word = 0;
	/* Configure Bit time and sample point */
	reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_AUTO;

	switch (bit_time) {
	case MCP25_CAN_250K_1M:
	case MCP25_CAN_500K_1M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 14;
		reg_dbtcfg.bf.tseg2 = 3;
		reg_dbtcfg.bf.sjw = 3;
		reg_tdc.bf.tdc_offset = 15;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_2M:
	case MCP25_CAN_500K_2M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 6;
		reg_dbtcfg.bf.tseg2 = 1;
		reg_dbtcfg.bf.sjw = 1;
		reg_tdc.bf.tdc_offset = 7;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_4M:
	case MCP25_CAN_500K_4M:
	case MCP25_CAN_1000K_4M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 2;
		reg_dbtcfg.bf.tseg2 = 0;
		reg_dbtcfg.bf.sjw = 0;
		reg_tdc.bf.tdc_offset = 3;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_500K_5M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 1;
		reg_dbtcfg.bf.tseg2 = 0;
		reg_dbtcfg.bf.sjw = 0;
		reg_tdc.bf.tdc_offset = 2;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_500K:
	case MCP25_CAN_125K_500K:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 30;
		reg_dbtcfg.bf.tseg2 = 7;
		reg_dbtcfg.bf.sjw = 7;
		reg_tdc.bf.tdc_offset = 31;
		reg_tdc.bf.tdc_val = 0;
		reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_OFF;
		break;
	case MCP25_CAN_250K_833K:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 17;
		reg_dbtcfg.bf.tseg2 = 4;
		reg_dbtcfg.bf.sjw = 4;
		reg_tdc.bf.tdc_offset = 18;
		reg_tdc.bf.tdc_val = 0;
		reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_OFF;
		break;
	case MCP25_CAN_250K_1M5:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 8;
		reg_dbtcfg.bf.tseg2 = 2;
		reg_dbtcfg.bf.sjw = 2;
		reg_tdc.bf.tdc_offset = 9;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_3M:
	case MCP25_CAN_500K_6M7:
	case MCP25_CAN_500K_8M:
	case MCP25_CAN_500K_10M:
	case MCP25_CAN_1000K_8M:
		return EMCP25XXFD_CAN_BIT_TIME_DAT_INV;
	default:
		return EMCP25XXFD_CAN_BIT_TIME_DAT;
	}

	r = write_word(MCP25_REG_CiDBTCFG, reg_dbtcfg.word);
	ERR_CHECK(r);

#ifdef REV_A
	reg_tdc.bf.tdc_offset = 0;
	reg_tdc.bf.tdc_val = 0;
#endif

	return write_word(MCP25_REG_CiTDC, reg_tdc.word);
}

static err_code mcp25_bit_time_cfg_nom_10MHz(
	enum mcp25_can_bittime_setup bit_time)
{
	union mcp25_reg_cinbtcfg reg;

	reg.word = can_ctrl_rst_vals[MCP25_REG_CiNBTCFG / 4];

	switch (bit_time) {
	case MCP25_CAN_500K_1M:
	case MCP25_CAN_500K_2M:
	case MCP25_CAN_500K_4M:
	case MCP25_CAN_500K_5M:
	case MCP25_CAN_500K_6M7:
	case MCP25_CAN_500K_8M:
	case MCP25_CAN_500K_10M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 14;
		reg.bf.tseg2 = 3;
		reg.bf.sjw = 3;
		break;
	case MCP25_CAN_250K_500K:
	case MCP25_CAN_250K_833K:
	case MCP25_CAN_250K_1M:
	case MCP25_CAN_250K_1M5:
	case MCP25_CAN_250K_2M:
	case MCP25_CAN_250K_3M:
	case MCP25_CAN_250K_4M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 30;
		reg.bf.tseg2 = 7;
		reg.bf.sjw = 7;
		break;
	case MCP25_CAN_1000K_4M:
	case MCP25_CAN_1000K_8M:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 7;
		reg.bf.tseg2 = 2;
		reg.bf.sjw = 2;
		break;
	case MCP25_CAN_125K_500K:
		reg.bf.brp = 0;
		reg.bf.tseg1 = 62;
		reg.bf.tseg2 = 15;
		reg.bf.sjw = 15;
		break;
	default:
		return EMCP25XXFD_CAN_BIT_TIME_NOM;
	}

	return write_word(MCP25_REG_CiNBTCFG, reg.word);
}

static err_code mcp25_bit_time_cfg_data_10MHz(
	enum mcp25_can_bittime_setup bit_time,
	enum mcp25_can_ssp_mode ssp_mode)
{
	union mcp25_reg_cidbtcfg reg_dbtcfg;
	union mcp25_reg_citdc reg_tdc;
	err_code r;

	reg_dbtcfg.word = can_ctrl_rst_vals[MCP25_REG_CiDBTCFG / 4];
	reg_tdc.word = 0;
	/* Configure Bit time and sample point */
	reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_AUTO;

	switch (bit_time) {
	case MCP25_CAN_250K_1M:
	case MCP25_CAN_500K_1M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 6;
		reg_dbtcfg.bf.tseg2 = 1;
		reg_dbtcfg.bf.sjw = 1;
		reg_tdc.bf.tdc_offset = 7;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_2M:
	case MCP25_CAN_500K_2M:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 2;
		reg_dbtcfg.bf.tseg2 = 0;
		reg_dbtcfg.bf.sjw = 0;
		reg_tdc.bf.tdc_offset = 3;
		reg_tdc.bf.tdc_val = 0;
		break;
	case MCP25_CAN_250K_500K:
	case MCP25_CAN_125K_500K:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 14;
		reg_dbtcfg.bf.tseg2 = 3;
		reg_dbtcfg.bf.sjw = 3;
		reg_tdc.bf.tdc_offset = 15;
		reg_tdc.bf.tdc_val = 0;
		reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_OFF;
		break;
	case MCP25_CAN_250K_833K:
		reg_dbtcfg.bf.brp = 0;
		reg_dbtcfg.bf.tseg1 = 7;
		reg_dbtcfg.bf.tseg2 = 2;
		reg_dbtcfg.bf.sjw = 2;
		reg_tdc.bf.tdc_offset = 8;
		reg_tdc.bf.tdc_val = 0;
		reg_tdc.bf.tdc_mode = MCP25_CAN_SSP_MODE_OFF;
		break;
	case MCP25_CAN_250K_1M5:
	case MCP25_CAN_250K_3M:
	case MCP25_CAN_250K_4M:
	case MCP25_CAN_500K_4M:
	case MCP25_CAN_500K_5M:
	case MCP25_CAN_500K_6M7:
	case MCP25_CAN_500K_8M:
	case MCP25_CAN_500K_10M:
	case MCP25_CAN_1000K_4M:
	case MCP25_CAN_1000K_8M:
		return EMCP25XXFD_CAN_BIT_TIME_DAT_INV;
	default:
		return EMCP25XXFD_CAN_BIT_TIME_DAT;
	}

	r = write_word(MCP25_REG_CiDBTCFG, reg_dbtcfg.word);
	ERR_CHECK(r);

#ifdef REV_A
	reg_tdc.bf.tdc_offset = 0;
	reg_tdc.bf.tdc_val = 0;
#endif

	return write_word(MCP25_REG_CiTDC, reg_tdc.word);
}

static err_code mcp25_bit_time_cfg(enum mcp25_can_bittime_setup bit_time,
	enum mcp25_can_ssp_mode ssp_mode, enum mcp25_can_sysclk_speed clk)
{
	err_code r;

	switch (clk) {
	case MCP25_CAN_SYSCLK_40M:
		r = mcp25_bit_time_cfg_nom_40MHz(bit_time);
		ERR_CHECK(r);
		r = mcp25_bit_time_cfg_data_40MHz(bit_time, ssp_mode);
		break;
	case MCP25_CAN_SYSCLK_20M:
		r = mcp25_bit_time_cfg_nom_20MHz(bit_time);
		ERR_CHECK(r);
		r = mcp25_bit_time_cfg_data_20MHz(bit_time, ssp_mode);
		break;
	case MCP25_CAN_SYSCLK_10M:
		r = mcp25_bit_time_cfg_nom_10MHz(bit_time);
		ERR_CHECK(r);
		r = mcp25_bit_time_cfg_data_10MHz(bit_time, ssp_mode);
		break;
	default:
		r = EMCP25XXFD_CAN_SYS_CLK;
		break;
	}

	return r;
}

static err_code mcp25_gpio_mode_cfg(enum mcp25_gpio_pin_mode gpio0,
	enum mcp25_gpio_pin_mode gpio1)
{
	union mcp25_reg_iocon iocon;
	err_code r;

	iocon.word = 0;

	r = read_byte((MCP25_REG_IOCON + 3), &iocon.byte[3]);
	ERR_CHECK(r);

	iocon.bf.pin_mode0 = gpio0;
	iocon.bf.pin_mode1 = gpio1;
	return write_byte((MCP25_REG_IOCON + 3), iocon.byte[3]);
}

static err_code mcp25_set_op_mode(enum mcp25_can_operation_mode mode)
{
	uint8_t d;
	err_code r;

	if (mode >= NUM_CAN_OPERATION_MODES)
		return EMCP25XXFD_INVALID_MODE;

	r = read_byte(MCP25_REG_CiCON + 3, &d);
	ERR_CHECK(r);

	d &= ~0x07;
	d |=  mode;
	return write_byte(MCP25_REG_CiCON + 3, d);
}

static err_code mcp25_get_op_mode(enum mcp25_can_operation_mode *p_mode)
{
	err_code r;
	uint8_t d;

	r = read_byte(MCP25_REG_CiCON + 2, &d);
	ERR_CHECK(r);

	/* Get Opmode bits & decode */
	d = (d >> 5) & 0x7;

	if (d >= NUM_CAN_OPERATION_MODES)
		return EMCP25XXFD_INVALID_MODE;

	*p_mode = d;
	return ERROR_OK;
}

static err_code mcp25c_cfg_obj_rst(struct mcp25_can_cfg *p_cfg)
{
	union mcp25_reg_cicon reg;

	reg.word = can_ctrl_rst_vals[MCP25_REG_CiCON / 4];

	p_cfg->dnet_filter_cnt = reg.bf.dnet_filter_cnt;
	p_cfg->iso_crc_enable = reg.bf.iso_crc_enable;
	p_cfg->prot_except_evt_disable = reg.bf.prot_except_evt_disable;
	p_cfg->wakeup_filter_enable = reg.bf.wakeup_filter_enable;
	p_cfg->wakeup_filter_time = reg.bf.wakeup_filter_time;
	p_cfg->bit_rate_switch_disable = reg.bf.bit_rate_switch_disable;
	p_cfg->restrict_re_tx_attempts = reg.bf.restrict_re_tx_attempts;
	p_cfg->esi_in_gateway_mode = reg.bf.esi_in_gateway_mode;
	p_cfg->sys_err_to_listen_only = reg.bf.sys_err_to_listen_only;
	p_cfg->store_in_tef = reg.bf.store_in_tef;
	p_cfg->txq_enable = reg.bf.txq_enable;
	p_cfg->tx_bw_sharing = reg.bf.tx_bw_sharing;
	return ERROR_OK;
}

static err_code mcp25c_cfg(struct mcp25_can_cfg *p_cfg)
{
	union mcp25_reg_cicon reg;

	reg.word = can_ctrl_rst_vals[MCP25_REG_CiCON / 4];

	reg.bf.dnet_filter_cnt = p_cfg->dnet_filter_cnt;
	reg.bf.iso_crc_enable = p_cfg->iso_crc_enable;
	reg.bf.prot_except_evt_disable = p_cfg->prot_except_evt_disable;
	reg.bf.wakeup_filter_enable = p_cfg->wakeup_filter_enable;
	reg.bf.wakeup_filter_time = p_cfg->wakeup_filter_time;
	reg.bf.bit_rate_switch_disable = p_cfg->bit_rate_switch_disable;
	reg.bf.restrict_re_tx_attempts = p_cfg->restrict_re_tx_attempts;
	reg.bf.esi_in_gateway_mode = p_cfg->esi_in_gateway_mode;
	reg.bf.sys_err_to_listen_only = p_cfg->sys_err_to_listen_only;
	reg.bf.store_in_tef = p_cfg->store_in_tef;
	reg.bf.txq_enable = p_cfg->txq_enable;
	reg.bf.tx_bw_sharing = p_cfg->tx_bw_sharing;
	return write_word(MCP25_REG_CiCON, reg.word);
}

static err_code mcp25c_can_init(const struct mcp25xxfd_can_cfg * const p_can_cfg,
		const int8_t stdby_pin)
{
	struct mcp25_can_filterobj_id filter;
	struct mcp25_can_tx_fifo_cfg tx_cfg;
	struct mcp25_can_rx_fifo_cfg rx_cfg;
	enum mcp25_can_operation_mode mode;
	struct mcp25_can_maskobj_id mask;
	struct mcp25_can_cfg config;
	err_code r;

	if (stdby_pin != BOARD_UNUSED_PIN)
		STDBY_SET_ACTIVE();

	r = mcp25_set_op_mode(MCP25_CAN_CONFIGURATION_MODE);
	ERR_CHECK(r);

	vTaskDelay(pdMS_TO_TICKS(CAN_OP_MODE_SWITCH_TIME_MS));

	r = mcp25_get_op_mode(&mode);
	ERR_CHECK(r);

	if (mode != MCP25_CAN_CONFIGURATION_MODE)
		return EMCP25XXFD_SET_MODE;

	r = mcp25_ecc_enable();
	ERR_CHECK(r);

	r = mcp25_init_ram(RAM_EMPTY_BYTE);
	ERR_CHECK(r);

	r = mcp25c_cfg_obj_rst(&config);
	ERR_CHECK(r);

	memset(&config, 0, sizeof(config));
	config.iso_crc_enable = 1;
	config.restrict_re_tx_attempts = 1;
	r = mcp25c_cfg(&config);
	ERR_CHECK(r);

	/* Setup TX FIFO */
	r = mcp25_tx_channel_configure_obj_rst(&tx_cfg);
	ERR_CHECK(r);

	if (p_can_cfg->tx_fifo_size > MCP25_CAN_FIFO_MAX_SIZE)
		return EMCP25XXFD_FIFO_SIZE;

	tx_cfg.tx_prio = 1;
	tx_cfg.tx_attempts = TX_ATTEMPTS_3;
	tx_cfg.fifo_size = p_can_cfg->tx_fifo_size;
	tx_cfg.payload_size = MCP25_CAN_PLSIZE_8;
	r = mcp25_tx_channel_configure(CAN_TX_FIFO_CH, &tx_cfg);
	ERR_CHECK(r);

	/* Setup RX FIFO */
	r = mcp25_rx_channel_cfg_obj_rst(&rx_cfg);
	ERR_CHECK(r);

	if (p_can_cfg->rx_fifo_size > MCP25_CAN_FIFO_MAX_SIZE)
		return EMCP25XXFD_FIFO_SIZE;

	rx_cfg.rx_ts_enable = 0;
	rx_cfg.fifo_size = p_can_cfg->rx_fifo_size;
	rx_cfg.payload_size = MCP25_CAN_PLSIZE_8;
	r = mcp25_rx_channel_configure(CAN_RX_FIFO_CH, &rx_cfg);
	ERR_CHECK(r);

	r = mcp25_filter_disable(MCP25_CAN_FILTER0);
	ERR_CHECK(r);

	/* Setup RX Filter */
	memset(&filter, 0, sizeof(filter));
	r = mcp25_filter_obj_configure(MCP25_CAN_FILTER0, &filter);
	ERR_CHECK(r);

	/* Setup RX Mask */
	memset(&mask, 0, sizeof(mask));
	r = mcp25_filter_msk_configure(MCP25_CAN_FILTER0, &mask);
	ERR_CHECK(r);

	r = mcp25_filter_to_fifo_link(MCP25_CAN_FILTER0, CAN_RX_FIFO_CH, true);
	ERR_CHECK(r);

	r = mcp25_bit_time_cfg(p_can_cfg->bit_time, MCP25_CAN_SSP_MODE_AUTO,
		p_can_cfg->sysclk);
	ERR_CHECK(r);

	r = mcp25_gpio_mode_cfg(MCP25_GPIO_MODE_INT, MCP25_GPIO_MODE_INT);
	ERR_CHECK(r);
	r = mcp25_tx_channel_evt_enable(CAN_TX_FIFO_CH,
		MCP25_CAN_TX_FIFO_NOT_FULL_EVENT);
	ERR_CHECK(r);
	r = mcp25_rx_channel_evt_enable(CAN_RX_FIFO_CH,
		MCP25_CAN_RX_FIFO_NOT_EMPTY_EVENT);
	ERR_CHECK(r);
	r = mcp25_module_evt_enable(MCP25_CAN_TX_EVENT | MCP25_CAN_RX_EVENT);
	ERR_CHECK(r);

	r = mcp25_set_op_mode(MCP25_CAN_NORMAL_MODE);
	ERR_CHECK(r);

	vTaskDelay(pdMS_TO_TICKS(CAN_OP_MODE_SWITCH_TIME_MS));

	r = mcp25_get_op_mode(&mode);
	ERR_CHECK(r);

	if (mode != MCP25_CAN_NORMAL_MODE)
		return EMCP25XXFD_SET_MODE;

	if (stdby_pin != BOARD_UNUSED_PIN)
		STDBY_SET_INACTIVE();

	return r;
}

static err_code process_can_rx_data(void)
{
	enum mcp25_can_rx_fifo_event flags;
	struct can_rx_client *p_can_clnt;
	union mcp25_can_rx_msgobj rx_obj;
	uint8_t client_idx;
	err_code r;
	uint8_t i;

	r = mcp25_rx_channel_evt_get(CAN_RX_FIFO_CH, &flags);
	ERR_CHECK(r);

	while (flags & MCP25_CAN_RX_FIFO_NOT_EMPTY_EVENT) {
		if (flags & MCP25_CAN_RX_FIFO_FULL_EVENT) {
			session_mgr_append_err_code(EMCP25XXFD_RX_FIFO_FULL, __func__);
			LOGE("Rx FIFO full");
		}

		r = mcp25_rx_msg_get(CAN_RX_FIFO_CH, &rx_obj, me.trx_buf,
			sizeof(me.trx_buf));
		ERR_CHECK(r);

		for (client_idx = 0; client_idx < CAN_CLIENTS_MAX; client_idx++) {
			p_can_clnt = &me.can_client[client_idx];

			if (!p_can_clnt->cb)
				continue;

			for (i = 0; i < p_can_clnt->nbr_ids; i++) {
				if ((p_can_clnt->can_id[i].use_eid && rx_obj.bf.ctrl.IDE &&
						(p_can_clnt->can_id[i].eid == rx_obj.bf.id.EID) &&
						(p_can_clnt->can_id[i].sid == rx_obj.bf.id.SID)) ||
						(!p_can_clnt->can_id[i].use_eid &&
						!rx_obj.bf.ctrl.IDE &&
						(p_can_clnt->can_id[i].sid == rx_obj.bf.id.SID))) {
					p_can_clnt->cb(p_can_clnt->can_id[i], me.trx_buf,
						rx_obj.bf.ctrl.DLC);
					break;
				}
			}
		}

		r = mcp25_rx_channel_evt_get(CAN_RX_FIFO_CH, &flags);
		ERR_CHECK(r);
	}

	return r;
}

static void task(void *arg)
{
	enum queue_item_type item;
	err_code r;

	while (1) {
		if (xQueueReceive(me.queue_handle, &item, portMAX_DELAY) == pdFALSE)
			continue;	/* Should never happen */

		switch (item) {
		case QUEUE_ITEM_CAN_RX_IRQ:
			if (!util_mutex_lock(me.mutex_handle)) {
				session_mgr_append_err_code(EMCP25XXFD_MUTEX, __func__);
				break;
			}

			r = process_can_rx_data();
			util_mutex_unlock(me.mutex_handle);
			if (r != ERROR_OK) {
				session_mgr_append_err_code(r, __func__);
				LOGE("can rcv r = 0x%08lX", r);
			}
			break;
		default:
			break;
		}
	}
}

err_code mcp25xxfd_unregister_can_rx_client(const uint32_t handle)
{
	uint8_t i;

	if (!me.initialized)
		return EMCP25XXFD_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return EMCP25XXFD_MUTEX;

	for (i = 0; i < CAN_CLIENTS_MAX; i++) {
		if (handle != (uint32_t)&me.can_client[i])
			continue;

		memset(&me.can_client[i], 0, sizeof(struct can_rx_client));
		break;
	}

	util_mutex_unlock(me.mutex_handle);

	if (i >= CAN_CLIENTS_MAX)
		return EMCP25XXFD_INVALID_CAN_CLIENT_HANDLE;

	return ERROR_OK;
}

err_code mcp25xxfd_register_can_rx_client(uint32_t * const p_handle,
		const struct can_rx_client * const p_client)
{
	uint8_t i;

	if (!me.initialized)
		return EMCP25XXFD_NO_INIT;

	if (!p_handle || !p_client || !p_client->nbr_ids || !p_client->cb)
		return EMCP25XXFD_INVALID_ARG;

	if (p_client->nbr_ids > CAN_CLIENT_MAX_NBR_IDS)
		return EMCP25XXFD_CAN_CLIENT_STD_IDS_OVERFLOW;

	if (!util_mutex_lock(me.mutex_handle))
		return EMCP25XXFD_MUTEX;

	for (i = 0; i < CAN_CLIENTS_MAX; i++) {
		if (me.can_client[i].cb)
			continue;

		me.can_client[i] = *p_client;
		break;
	}

	util_mutex_unlock(me.mutex_handle);

	if (i >= CAN_CLIENTS_MAX)
		return EMCP25XXFD_CAN_CLIENT_OVERFLOW;

	return ERROR_OK;
}

err_code mcp25xxfd_send_can_frame(const struct can_id can_id,
	const uint8_t * const p_buf, const uint8_t buf_size)
{
	enum mcp25_can_tx_fifo_event flags;
	union mcp25_can_tx_msgobj tx_obj;
	err_code r;

	if (!me.initialized)
		return EMCP25XXFD_NO_INIT;

	if (!p_buf || !buf_size || (buf_size > CAN_MAX_PAYLOAD_BYTES))
		return EMCP25XXFD_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EMCP25XXFD_MUTEX;

	r = mcp25_tx_channel_evt_get(CAN_TX_FIFO_CH, &flags);
	ERR_CHECK_GOTO(r, exit);

	if (!(flags & MCP25_CAN_TX_FIFO_NOT_FULL_EVENT)) {
		r = EMCP25XXFD_TX_FIFO_FULL;
		session_mgr_append_err_code(r, __func__);
		goto exit;
	}

	memset(&tx_obj, 0, sizeof(tx_obj));

	if (can_id.use_eid) {
		tx_obj.bf.id.EID = can_id.eid;
		tx_obj.bf.ctrl.IDE = 1;
	}
	tx_obj.bf.id.SID = can_id.sid;
	tx_obj.bf.ctrl.DLC = buf_size;

	r = mcp25_tx_channel_load(CAN_TX_FIFO_CH, &tx_obj, p_buf, buf_size, true);

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code mcp25xxfd_deinit(void)
{
	if (!me.initialized)
		return ERROR_OK;

	if (!util_mutex_lock(me.mutex_handle))
		return EMCP25XXFD_MUTEX;

	(void)power_set_regulator_can_enable_state(false);

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

	me.initialized = false;
	util_mutex_unlock(me.mutex_handle);

	return ERROR_OK;
}

err_code mcp25xxfd_init(const struct mcp25xxfd_board * const p_board)
{
	union mcp25_reg_devid devid;
	err_code r;
	uint8_t i;

	if (me.initialized)
		return ERROR_OK;

	if ((p_board == NULL) || (p_board->p_instance == NULL))
		return EMCP25XXFD_PDATA;

	r = util_validate_pins((uint8_t *)&p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	if (p_board->pins.int_op != BOARD_UNUSED_PIN)
		nrf_gpio_cfg(p_board->pins.int_op,
					NRF_GPIO_PIN_DIR_INPUT,
					NRF_GPIO_PIN_INPUT_DISCONNECT,
					NRF_GPIO_PIN_NOPULL,
					NRF_GPIO_PIN_D0S1,
					NRF_GPIO_PIN_NOSENSE);

	if (p_board->pins.int_0 != BOARD_UNUSED_PIN)
		nrf_gpio_cfg(p_board->pins.int_0,
					NRF_GPIO_PIN_DIR_INPUT,
					NRF_GPIO_PIN_INPUT_DISCONNECT,
					NRF_GPIO_PIN_NOPULL,
					NRF_GPIO_PIN_D0S1,
					NRF_GPIO_PIN_NOSENSE);

	if (p_board->pins.int_1 != BOARD_UNUSED_PIN) {
		nrf_gpio_cfg(p_board->pins.int_1,
					NRF_GPIO_PIN_DIR_INPUT,
					NRF_GPIO_PIN_INPUT_DISCONNECT,
					NRF_GPIO_PIN_NOPULL,
					NRF_GPIO_PIN_D0S1,
					NRF_GPIO_PIN_NOSENSE);

		r = gpio_irq_register(p_board->pins.int_1, GPIO_IRQ_POL_LOW,
				gpio_irq_callback);
		ERR_CHECK(r);
	}

	if (p_board->pins.stdby != BOARD_UNUSED_PIN)
		nrf_gpio_cfg(p_board->pins.stdby,
					NRF_GPIO_PIN_DIR_OUTPUT,
					NRF_GPIO_PIN_INPUT_DISCONNECT,
					NRF_GPIO_PIN_NOPULL,
					NRF_GPIO_PIN_S0S1,
					NRF_GPIO_PIN_NOSENSE);

	me.p_board = p_board;

	r = spi_init();
	ERR_CHECK(r);

	r = EMCP25XXFD_RESET;
	for (i = 0; i < MAX_NBR_RETRIES; i++) {
		vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));

		r = sw_reset();
		if (r != ERROR_OK)
			continue;

		r = read_word(MCP25_REG_DEVID, &devid.word);
		if (r != ERROR_OK)
			continue;

		/* TODO: Investigate why dev id is zero */
		LOGI("DEV_ID:0x%08lX", devid.word);
		break;
	}
	ERR_CHECK(r);

	me.queue_handle = CREATE_STATIC_QUEUE(DRV_NAME);
	me.mutex_handle = CREATE_STATIC_MUTEX(DRV_NAME);
	me.task_handle = CREATE_STATIC_TASK(task, TASK_NAME, TASK_PRIORITY);

	r = tasktracker_register_task(me.task_handle);
	ERR_CHECK(r);

	/* Transceiver power must be enabled at this point because the mcp25 requires an
	 * idle-state on the CAN RX line during can init to succesfully join the bus.
	 */
	r = power_set_regulator_can_enable_state(true);
	ERR_CHECK(r);

	for (i = 0; i < MAX_NBR_RETRIES; i++) {
		if (!util_mutex_lock(me.mutex_handle)) {
			r = EMCP25XXFD_MUTEX;
			break;
		}

		r = mcp25c_can_init(&me.p_board->can_cfg, me.p_board->pins.stdby);

		util_mutex_unlock(me.mutex_handle);

		if (r == ERROR_OK)
			break;

		vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
	}
	if (r != ERROR_OK) {
		(void)power_set_regulator_can_enable_state(false);
		return r;
	}

	me.initialized = true;

	return r;
}
