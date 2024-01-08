#include <nrf_gpio.h>
#include <string.h>
#include <app_util.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "drivers/w25n0x.h"
#include "drivers/twim_spim_irq.h"
#include "freertos_static.h"
#include "util.h"

#define DRV_NAME						W25N0X
#define LOG_TAG							STRINGIFY(DRV_NAME)
#include "log.h"

#define MUTEX_NAME						DRV_NAME
#define SPI_COMPLETION_TMO_MS			150
#define FLASH_BUSY_TMO_MS				100
#define FLASH_BUSY_POLL_INTERVAL_MS		1
#define SPI_DMA_MAX_LEN					255

#define W25N01_JEDEC_ID					0x21AAEF
#define W25M02_JEDEC_ID					0x21ABEF
#define W25N02_JEDEC_ID					0x22AAEF
#define W25N04_JEDEC_ID					0x23AAEF

#define WRITE_TO_ALL_BLOCKS_ENABLED		0
#define FAST_READ_CMD_SIZE				4
#define PROGRAM_DATA_LOAD_CMD_SIZE		3

#define BIT_SET							0x1
#define PAD_FIELD_SIZE					1

#define UINT24_PTR(val)					(&((uint8_t *)&(val))[1])

#define CS_SET_ACTIVE()		nrf_gpio_pin_clear(me.p_board->pins.spi_cs)
#define CS_SET_INACTIVE()	nrf_gpio_pin_set(me.p_board->pins.spi_cs)

enum opcode {
	OPCODE_DEVICE_RESET			= 0xFF,
	OPCODE_READ_JEDEC_ID		= 0x9F,
	OPCODE_READ_STATUS_REG		= 0x05,
	OPCODE_WRITE_STATUS_REG		= 0x01,
	OPCODE_WRITE_ENABLE			= 0x06,
	OPCODE_WRITE_DISABLE		= 0x04,
	OPCODE_BLOCK_ERASE			= 0xD8,
	OPCODE_PROGRAM_EXECUTE		= 0x10,
	OPCODE_PAGE_DATA_READ		= 0x13,
	OPCODE_PROGRAM_DATA_LOAD	= 0x02,
	OPCODE_FAST_READ			= 0x0B,
};

struct instr {
	enum opcode opcode;
	uint8_t padding_0;
	uint8_t opcode_add_len;
	uint8_t resp_len;
};

enum instr_idx {
	DEVICE_RESET = 0,
	JEDEC_ID,
	READ_STATUS_REGISTER,
	WRITE_STATUS_REGISTER,
	WRITE_ENABLE,
	WRITE_DISABLE,
	BLOCK_ERASE,
	PROGRAM_EXECUTE,
	PAGE_DATA_READ,
};

enum register_type {
	PROTECTION_REG		= 0xA0,
	CONFIGURATION_REG	= 0xB0,
	STATUS_REG			= 0xC0,
};

union status_reg {
	struct {
		uint8_t busy			: 1;
		uint8_t write_enable	: 1;
		uint8_t erase_failure	: 1;
		uint8_t program_failure	: 1;
		uint8_t ecc_0			: 1;
		uint8_t ecc_1			: 1;
		uint8_t bbm_lut_full	: 1;
		uint8_t reserved		: 1;
	} fields;

	uint8_t word;
};

union conf_reg {
	struct {
		uint8_t reserved			: 3;
		uint8_t buffer_mode			: 1;
		uint8_t ecc_enable			: 1;
		uint8_t prot_reg_lock		: 1;
		uint8_t enter_otp_mode		: 1;
		uint8_t otp_data_pages_lock	: 1;
	} fields;

	uint8_t word;
};

union prot_reg {
	struct {
		uint8_t srp1	: 1;
		uint8_t wp_e	: 1;
		uint8_t tb		: 1;
		uint8_t bp0		: 1;
		uint8_t bp1		: 1;
		uint8_t bp2		: 1;
		uint8_t bp3		: 1;
		uint8_t srp0	: 1;
	} fields;

	uint8_t word;
};

STATIC_MUTEX(MUTEX_NAME);

static struct {
	const struct w25n0x_board	*p_board;
	SemaphoreHandle_t			mutex_handle;
	TaskHandle_t				req_handle;
	uint8_t						rw_buffer[W25N0X_PAGE_SIZE +
									W25N0X_SPARE_SIZE_MAX +
									FAST_READ_CMD_SIZE];
	union status_reg			status_reg;
} me;

/* Reference: 8.1.3 Instruction Set Table 2 in W25N01 rev.L spec */
const struct instr instr_list[] = {
		[DEVICE_RESET] = {
				.opcode = OPCODE_DEVICE_RESET
		},
		[JEDEC_ID] = {
				.opcode = OPCODE_READ_JEDEC_ID,
				.padding_0 = PAD_FIELD_SIZE,
				.resp_len = 3
		},
		[READ_STATUS_REGISTER] = {
				.opcode = OPCODE_READ_STATUS_REG,
				.opcode_add_len = 1,
				.resp_len = 1
		},
		[WRITE_STATUS_REGISTER] = {
				.opcode = OPCODE_WRITE_STATUS_REG,
				.opcode_add_len = 2
		},
		[WRITE_ENABLE] = {
				.opcode = OPCODE_WRITE_ENABLE
		},
		[WRITE_DISABLE] = {
				.opcode = OPCODE_WRITE_DISABLE
		},
		[BLOCK_ERASE] = {
				.opcode = OPCODE_BLOCK_ERASE,
				.opcode_add_len = 3
		},
		[PROGRAM_EXECUTE] = {
				.opcode = OPCODE_PROGRAM_EXECUTE,
				.opcode_add_len = 3
		},
		[PAGE_DATA_READ] = {
				.opcode = OPCODE_PAGE_DATA_READ,
				.opcode_add_len = 3
		},
};

static const uint16_t nbr_of_blocks_by_variant[NBR_OF_W25N0X_VARIANTS] = {
	[W25N01] = 1024, /* 1 Gb, or 128 MB */
	[W25M02] = 2048, /* 2 Gb, or 256 MB */
	[W25N02] = 2048, /* 2 Gb, or 256 MB */
	[W25N04] = 4096, /* 4 Gb, or 512 MB */
};

static const uint8_t spare_size_by_variant[NBR_OF_W25N0X_VARIANTS] = {
	[W25N01] = W25N0X_SPARE_SIZE,
	[W25M02] = W25N0X_SPARE_SIZE,
	[W25N02] = W25N0X_SPARE_SIZE_MAX,
	[W25N04] = W25N0X_SPARE_SIZE_MAX,
};

static err_code wait_until_ready(void);

static uint16_t get_nbr_of_blocks(void)
{
	return nbr_of_blocks_by_variant[me.p_board->variant];
}

static uint32_t get_total_size(void)
{
	return get_nbr_of_blocks() * W25N0X_BLOCK_SIZE;
}

static void spim_irq_callback(void)
{
	BaseType_t yield_req = pdFALSE;

	if (nrf_spim_event_check(me.p_board->p_instance, NRF_SPIM_EVENT_END)) {
		nrf_spim_event_clear(me.p_board->p_instance, NRF_SPIM_EVENT_END);

		if (me.req_handle != NULL)
			vTaskNotifyGiveFromISR(me.req_handle, &yield_req);
	}

	portYIELD_FROM_ISR(yield_req);
}

static err_code spi_transfer(const uint8_t *const p_tx, const uint16_t tx_size,
		uint8_t *const p_rx, const uint16_t rx_size)
{
	uint16_t bytes_left_to_write = tx_size;
	uint16_t bytes_left_to_read = rx_size;
	const uint8_t *p_current_tx = p_tx;
	uint8_t *p_current_rx = p_rx;
	uint8_t chunk_read;
	uint8_t chunk_write;
	err_code r;

	me.req_handle = xTaskGetCurrentTaskHandle();
	r = ERROR_OK;

	CS_SET_ACTIVE();

	do {
		chunk_read = MIN(SPI_DMA_MAX_LEN,
				bytes_left_to_read);

		chunk_write = MIN(SPI_DMA_MAX_LEN,
				bytes_left_to_write);

		nrf_spim_tx_buffer_set(me.p_board->p_instance, p_current_tx,
			chunk_write);
		nrf_spim_rx_buffer_set(me.p_board->p_instance, p_current_rx,
			chunk_read);

		nrf_spim_event_clear(me.p_board->p_instance, NRF_SPIM_EVENT_END);
		nrf_spim_task_trigger(me.p_board->p_instance, NRF_SPIM_TASK_START);

		if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
				pdMS_TO_TICKS(SPI_COMPLETION_TMO_MS)) != pdTRUE) {
			nrf_spim_task_trigger(me.p_board->p_instance, NRF_SPIM_TASK_STOP);
			r = EW25N0X_SPI_NO_RESP;
			break;
		}

		p_current_tx += chunk_write;
		p_current_rx += chunk_read;
		bytes_left_to_read -= chunk_read;
		bytes_left_to_write -= chunk_write;

	} while (bytes_left_to_read > 0 ||
			bytes_left_to_write > 0);

	CS_SET_INACTIVE();

	me.req_handle = NULL;
	return r;
}

static err_code spi_init(void)
{
	err_code r;

	nrf_gpio_cfg_output(me.p_board->pins.spi_clk);
	nrf_gpio_cfg_output(me.p_board->pins.io0);
	nrf_gpio_cfg_input(me.p_board->pins.io1, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_output(me.p_board->pins.spi_cs);

	nrf_spim_pins_set(me.p_board->p_instance, me.p_board->pins.spi_clk,
			me.p_board->pins.io0, me.p_board->pins.io1);

	nrf_spim_frequency_set(me.p_board->p_instance, me.p_board->frequency);

	nrf_spim_configure(me.p_board->p_instance, NRF_SPIM_MODE_0,
		NRF_SPIM_BIT_ORDER_MSB_FIRST);

	r = twim_spim_irq_register_spim(me.p_board->p_instance, spim_irq_callback);
	ERR_CHECK(r);

	nrf_spim_int_enable(me.p_board->p_instance, NRF_SPIM_INT_END_MASK);

	nrf_spim_enable(me.p_board->p_instance);

	NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(me.p_board->p_instance),
			APP_IRQ_PRIORITY_LOW);
	NRFX_IRQ_ENABLE(nrfx_get_irq_number(me.p_board->p_instance));

	return ERROR_OK;
}

static err_code send_instr(const enum instr_idx instr,
		const uint8_t *const p_instr_payload, uint8_t *const p_instr_response,
		const uint8_t resp_buf_size)
{
	const struct instr *p_instr = &instr_list[instr];
	uint8_t tx_len;
	err_code r;

	if ((p_instr->opcode_add_len > 0 && p_instr_payload == NULL) ||
			(p_instr->resp_len > 0 && (p_instr_response == NULL ||
					resp_buf_size < p_instr->resp_len)))
		return EW25N0X_INSTR_HANDLING;

	tx_len = FIELD_SIZE(instr, opcode) + p_instr->padding_0 +
			p_instr->opcode_add_len + p_instr->resp_len;

	me.rw_buffer[0] = p_instr->opcode;

	memcpy(&me.rw_buffer[FIELD_SIZE(instr, opcode) + p_instr->padding_0],
			p_instr_payload, p_instr->opcode_add_len);

	r = spi_transfer(me.rw_buffer, tx_len, me.rw_buffer, tx_len);
	ERR_CHECK(r);

	memcpy(p_instr_response, &me.rw_buffer[FIELD_SIZE(instr, opcode) +
		p_instr->padding_0 + p_instr->opcode_add_len], p_instr->resp_len);

	if (instr == DEVICE_RESET || instr == PAGE_DATA_READ ||
			instr == PROGRAM_EXECUTE || instr == BLOCK_ERASE)
		r = wait_until_ready();

	return r;
}

static bool is_busy(void)
{
	const enum register_type reg_type = STATUS_REG;

	me.status_reg.fields.busy = BIT_SET;

	/* Skipping return value. In case of failure here the caller function
	 * returns is_busy set to true which is safe
	 */
	(void)send_instr(READ_STATUS_REGISTER, &reg_type, &me.status_reg.word,
				sizeof(me.status_reg));

	return (me.status_reg.fields.busy == BIT_SET);
}

static err_code wait_until_ready(void)
{
	uint8_t i;

	for (i = FLASH_BUSY_POLL_INTERVAL_MS; is_busy(); i++) {
		if (i == FLASH_BUSY_TMO_MS)
			return EW25N0X_FLASH_BUSY;

		vTaskDelay(pdMS_TO_TICKS(FLASH_BUSY_POLL_INTERVAL_MS));
	}

	return ERROR_OK;
}

static err_code verify_chip_id(uint32_t expected_id)
{
	uint32_t jedec_id;
	err_code r;

	r = send_instr(JEDEC_ID, NULL, (uint8_t *)&jedec_id, sizeof(jedec_id));
	ERR_CHECK(r);

	if (jedec_id != expected_id) {
		LOGE("Unexpected JEDEC ID: 0x%X", (unsigned int)jedec_id);
		return EW25N0X_UNEXPECTED_JEDEC_ID;
	}

	return ERROR_OK;
}

static err_code program_execute(const uint32_t page_address)
{
	const enum register_type reg_type = STATUS_REG;
	union status_reg status_reg;
	uint32_t page_address_be;
	err_code r;

	(void)uint32_big_encode(page_address, (uint8_t *)&page_address_be);

	r = send_instr(PROGRAM_EXECUTE, UINT24_PTR(page_address_be), NULL, 0);
	ERR_CHECK(r);

	r = send_instr(READ_STATUS_REGISTER, &reg_type, &status_reg.word,
			sizeof(status_reg));
	ERR_CHECK(r);

	if (status_reg.fields.program_failure == BIT_SET)
		return EW25N0X_PROGRAM;

	return ERROR_OK;
}

err_code w25n0x_get_nbr_of_blocks(uint16_t * const p_nbr_of_blocks)
{
	if (me.p_board == NULL)
		return EW25N0X_NO_INIT;

	if (p_nbr_of_blocks == NULL)
		return EW25N0X_INVALID_ARG;

	*p_nbr_of_blocks = get_nbr_of_blocks();
	return ERROR_OK;
}

err_code w25n0x_write(const uint8_t *const p_tx_buffer,
		const uint32_t tx_buffer_length, const uint32_t dest_address)
{
	uint16_t current_column_address_be;
	const uint8_t *p_current_tx_buffer;
	uint16_t current_column_address;
	uint32_t current_page_address;
	uint32_t bytes_left_to_write;
	bool start_new_page_write;
	uint16_t chunk_size;
	err_code r;

	if (me.p_board == NULL)
		return EW25N0X_NO_INIT;

	if (p_tx_buffer == NULL || tx_buffer_length == 0 ||
			((dest_address + tx_buffer_length) > get_total_size()) ||
			((dest_address + tx_buffer_length) < dest_address))
		return EW25N0X_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EW25N0X_MUTEX;

	current_page_address = (dest_address / W25N0X_PAGE_SIZE);
	current_column_address = (dest_address % W25N0X_PAGE_SIZE);
	p_current_tx_buffer = p_tx_buffer;
	bytes_left_to_write = tx_buffer_length;
	start_new_page_write = true;

	do {
		if (start_new_page_write) {
			r = send_instr(WRITE_ENABLE, NULL, NULL, 0);
			ERR_CHECK_GOTO(r, exit);

			start_new_page_write = false;
		}

		chunk_size = MIN(bytes_left_to_write,
				(W25N0X_PAGE_SIZE - current_column_address));

		(void)uint16_big_encode(current_column_address,
				(uint8_t *)&current_column_address_be);

		me.rw_buffer[0] = OPCODE_PROGRAM_DATA_LOAD;
		memcpy(&me.rw_buffer[1], &current_column_address_be,
				sizeof(current_column_address_be));
		memcpy(&me.rw_buffer[PROGRAM_DATA_LOAD_CMD_SIZE], p_current_tx_buffer,
				chunk_size);

		r = spi_transfer(me.rw_buffer, chunk_size + PROGRAM_DATA_LOAD_CMD_SIZE,
				NULL, 0);
		ERR_CHECK_GOTO(r, exit);

		bytes_left_to_write -= chunk_size;
		p_current_tx_buffer += chunk_size;
		current_column_address += chunk_size;

		if (bytes_left_to_write == 0 ||
				current_column_address == W25N0X_PAGE_SIZE) {

			r = program_execute(current_page_address);
			ERR_CHECK_GOTO(r, exit);

			current_column_address = 0;
			current_page_address++;
			start_new_page_write = true;
		}
	} while (bytes_left_to_write > 0);

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code w25n0x_write_page(const uint8_t *const p_page_data,
	const uint32_t page_data_len, struct w25n0x_spare_data * const p_spare_data,
	const uint32_t block_number, const uint32_t page_number)
{
	uint16_t column_address_be;
	err_code r;

	if (me.p_board == NULL)
		return EW25N0X_NO_INIT;

	if (!p_page_data || !p_spare_data)
		return EW25N0X_INVALID_ARG;

	if ((page_data_len == 0) || (page_data_len > W25N0X_PAGE_SIZE))
		return EW25N0X_INVALID_ARG;

	if (block_number >= get_nbr_of_blocks())
		return EW25N0X_INVALID_ARG;

	if (page_number >= W25N0X_PAGES_IN_BLOCK)
		return EW25N0X_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EW25N0X_MUTEX;

	r = send_instr(WRITE_ENABLE, NULL, NULL, 0);
	ERR_CHECK_GOTO(r, exit);

	(void)uint16_big_encode(0, (uint8_t *)&column_address_be);

	me.rw_buffer[0] = OPCODE_PROGRAM_DATA_LOAD;
	memcpy(&me.rw_buffer[1], &column_address_be, sizeof(column_address_be));
	memcpy(&me.rw_buffer[PROGRAM_DATA_LOAD_CMD_SIZE], p_page_data,
		page_data_len);
	memset(&me.rw_buffer[PROGRAM_DATA_LOAD_CMD_SIZE + page_data_len],
		W25N0X_EMPTY_BYTE, (W25N0X_PAGE_SIZE - page_data_len));
	memcpy(&me.rw_buffer[PROGRAM_DATA_LOAD_CMD_SIZE + W25N0X_PAGE_SIZE],
		p_spare_data, W25N0X_SPARE_SIZE);

	r = spi_transfer(me.rw_buffer,
		(PROGRAM_DATA_LOAD_CMD_SIZE + W25N0X_PAGE_SIZE + W25N0X_SPARE_SIZE),
		NULL, 0);
	ERR_CHECK_GOTO(r, exit);

	r = program_execute((block_number * W25N0X_PAGES_IN_BLOCK) + page_number);
	ERR_CHECK_GOTO(r, exit);

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;

}

err_code w25n0x_write_page_spare(const struct w25n0x_spare_data * const p_data,
	const uint16_t block_number, const uint8_t page_number)
{
	uint16_t column_address_be;
	err_code r;

	if (me.p_board == NULL)
		return EW25N0X_NO_INIT;

	if (block_number >= get_nbr_of_blocks())
		return EW25N0X_INVALID_ARG;

	if (page_number >= W25N0X_PAGES_IN_BLOCK)
		return EW25N0X_INVALID_ARG;

	if (!p_data)
		return EW25N0X_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EW25N0X_MUTEX;

	r = send_instr(WRITE_ENABLE, NULL, NULL, 0);
	ERR_CHECK_GOTO(r, exit);

	(void)uint16_big_encode(W25N0X_PAGE_SIZE, (uint8_t *)&column_address_be);

	me.rw_buffer[0] = OPCODE_PROGRAM_DATA_LOAD;
	memcpy(&me.rw_buffer[1], &column_address_be, sizeof(column_address_be));
	memcpy(&me.rw_buffer[PROGRAM_DATA_LOAD_CMD_SIZE], p_data, sizeof(*p_data));

	r = spi_transfer(me.rw_buffer,
		(PROGRAM_DATA_LOAD_CMD_SIZE + sizeof(*p_data)), NULL, 0);
	ERR_CHECK_GOTO(r, exit);

	r = program_execute((block_number * W25N0X_PAGES_IN_BLOCK) + page_number);
	ERR_CHECK_GOTO(r, exit);

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code w25n0x_read(uint8_t *const p_rx_buffer,
		const uint32_t rx_buffer_length, const uint32_t src_address)
{
	uint32_t current_page_address;
	uint32_t current_page_address_be;
	uint16_t current_column_address;
	uint16_t current_column_address_be;
	uint32_t bytes_left_to_read;
	uint8_t *p_current_rx_buffer;
	bool start_new_page_read;
	uint16_t chunk_size;
	err_code r;

	if (me.p_board == NULL)
		return EW25N0X_NO_INIT;

	if (p_rx_buffer == NULL || rx_buffer_length == 0 ||
			((src_address + rx_buffer_length) > get_total_size()) ||
			((src_address + rx_buffer_length) < src_address))
		return EW25N0X_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EW25N0X_MUTEX;

	current_page_address = (src_address / W25N0X_PAGE_SIZE);
	current_column_address = (src_address % W25N0X_PAGE_SIZE);
	p_current_rx_buffer = p_rx_buffer;
	bytes_left_to_read = rx_buffer_length;
	start_new_page_read = true;

	do {
		if (start_new_page_read) {
			(void)uint32_big_encode(current_page_address,
					(uint8_t *)&current_page_address_be);
			r = send_instr(PAGE_DATA_READ, UINT24_PTR(current_page_address_be),
				NULL, 0);
			ERR_CHECK_GOTO(r, exit);

			start_new_page_read = false;
		}

		chunk_size = MIN(bytes_left_to_read,
				(W25N0X_PAGE_SIZE - current_column_address));

		(void)uint16_big_encode(current_column_address,
						(uint8_t *)&current_column_address_be);

		me.rw_buffer[0] = OPCODE_FAST_READ;
		memcpy(&me.rw_buffer[1], &current_column_address_be,
				sizeof(current_column_address_be));

		r = spi_transfer(me.rw_buffer, FAST_READ_CMD_SIZE, me.rw_buffer,
				chunk_size + FAST_READ_CMD_SIZE);
		ERR_CHECK_GOTO(r, exit);

		memcpy(p_current_rx_buffer, &me.rw_buffer[FAST_READ_CMD_SIZE],
				chunk_size);

		bytes_left_to_read -= chunk_size;
		p_current_rx_buffer += chunk_size;
		current_column_address += chunk_size;

		if (current_column_address == W25N0X_PAGE_SIZE) {
			current_column_address = 0;
			current_page_address++;
			start_new_page_read = true;
		}
	} while (bytes_left_to_read > 0);

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code w25n0x_read_page_spare(struct w25n0x_spare_data * const p_data,
	bool * const p_ecc_ok, const uint16_t block_number,
	const uint8_t page_number)
{
	uint16_t column_address_be;
	uint32_t page_address_be;
	uint8_t spare_size;
	err_code r;

	if (me.p_board == NULL)
		return EW25N0X_NO_INIT;

	if (block_number >= get_nbr_of_blocks())
		return EW25N0X_INVALID_ARG;

	if (page_number >= W25N0X_PAGES_IN_BLOCK)
		return EW25N0X_INVALID_ARG;

	if (!p_data || !p_ecc_ok)
		return EW25N0X_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EW25N0X_MUTEX;

	(void)uint32_big_encode(((block_number * W25N0X_PAGES_IN_BLOCK)
		+ page_number), (uint8_t *)&page_address_be);
	r = send_instr(PAGE_DATA_READ, UINT24_PTR(page_address_be), NULL, 0);
	ERR_CHECK_GOTO(r, exit);

	*p_ecc_ok = (me.status_reg.fields.ecc_1 == 0);

	(void)uint16_big_encode(W25N0X_PAGE_SIZE, (uint8_t *)&column_address_be);
	me.rw_buffer[0] = OPCODE_FAST_READ;
	memcpy(&me.rw_buffer[1], &column_address_be, sizeof(column_address_be));

	/* For some reason, if we don't read all the available data here then
	 * subsequent reads will return corrupted data.
	 */
	spare_size = spare_size_by_variant[me.p_board->variant];

	r = spi_transfer(me.rw_buffer, FAST_READ_CMD_SIZE, me.rw_buffer,
		(FAST_READ_CMD_SIZE + spare_size));
	ERR_CHECK_GOTO(r, exit);

	memcpy(p_data, &me.rw_buffer[FAST_READ_CMD_SIZE], sizeof(*p_data));

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code w25n0x_block_erase(const uint16_t block_number)
{
	const enum register_type reg_type = STATUS_REG;
	union status_reg status_reg;
	uint32_t page_address_be;
	err_code r;

	if (me.p_board == NULL)
		return EW25N0X_NO_INIT;

	if (block_number >= get_nbr_of_blocks())
		return EW25N0X_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EW25N0X_MUTEX;

	(void)uint32_big_encode(block_number * W25N0X_PAGES_IN_BLOCK,
					(uint8_t *)&page_address_be);

	r = send_instr(WRITE_ENABLE, NULL, NULL, 0);
		ERR_CHECK_GOTO(r, exit);

	r = send_instr(BLOCK_ERASE, UINT24_PTR(page_address_be), NULL, 0);
	ERR_CHECK_GOTO(r, exit);

	r = send_instr(READ_STATUS_REGISTER, &reg_type, &status_reg.word,
			sizeof(status_reg));
	ERR_CHECK_GOTO(r, exit);

	if (status_reg.fields.erase_failure == BIT_SET)
		r = EW25N0X_ERASE;

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code w25n0x_deinit(void)
{
	if (!me.p_board)
		return ERROR_OK;

	if (!util_mutex_lock(me.mutex_handle))
		return EW25N0X_MUTEX;

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
	nrf_gpio_cfg_default(me.p_board->pins.io0);
	nrf_gpio_cfg_default(me.p_board->pins.io1);
	nrf_gpio_cfg_default(me.p_board->pins.io2);
	nrf_gpio_cfg_default(me.p_board->pins.io3);

	me.p_board = NULL;
	util_mutex_unlock(me.mutex_handle);

	return ERROR_OK;
}

err_code w25n0x_init(const struct w25n0x_board *const p_board)
{
	uint8_t write_status_reg_payload[2];
	union conf_reg conf_reg;
	union prot_reg prot_reg;
	uint32_t jedec_id;
	err_code r;

	if (me.p_board != NULL)
		return ERROR_OK;

	if ((p_board == NULL) || (p_board->p_instance == NULL))
		return EW25N0X_PDATA;

	switch (p_board->variant) {
	case W25N01:
		jedec_id = W25N01_JEDEC_ID;
		break;
	case W25M02:
		jedec_id = W25M02_JEDEC_ID;
		break;
	case W25N02:
		jedec_id = W25N02_JEDEC_ID;
		break;
	case W25N04:
		jedec_id = W25N04_JEDEC_ID;
		break;
	default:
		return EW25N0X_UNKNOWN_VARIANT;
	}

	r = util_validate_pins((uint8_t *)&p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	me.p_board = p_board;

	/* IO2 and IO3 must be outputs with state HIGH for normal SPI usage */
	nrf_gpio_cfg_output(me.p_board->pins.io2);
	nrf_gpio_cfg_output(me.p_board->pins.io3);
	nrf_gpio_pin_set(me.p_board->pins.io2);
	nrf_gpio_pin_set(me.p_board->pins.io3);

	spi_init();

	r = send_instr(DEVICE_RESET, NULL, NULL, 0);
	ERR_CHECK_GOTO(r, exit);

	r = verify_chip_id(jedec_id);
	ERR_CHECK_GOTO(r, exit);

	conf_reg.word = 0;
	conf_reg.fields.buffer_mode = BIT_SET;
	conf_reg.fields.ecc_enable = BIT_SET;

	write_status_reg_payload[0] = CONFIGURATION_REG;
	write_status_reg_payload[1] = conf_reg.word;

	r = send_instr(WRITE_STATUS_REGISTER, write_status_reg_payload, NULL, 0);
	ERR_CHECK_GOTO(r, exit);

	prot_reg.word = WRITE_TO_ALL_BLOCKS_ENABLED;

	write_status_reg_payload[0] = PROTECTION_REG;
	write_status_reg_payload[1] = prot_reg.word;

	r = send_instr(WRITE_STATUS_REGISTER, write_status_reg_payload, NULL, 0);

exit:
	if (r == ERROR_OK)
		me.mutex_handle = CREATE_STATIC_MUTEX(MUTEX_NAME);
	else
		me.p_board = NULL;

	return r;
}
