#pragma once

#include <stdbool.h>
#include <nrf_spim.h>

#include "error.h"

#define EW25N0X_NO_INIT				(EW25N0X_BASE + 0x00)
#define EW25N0X_PDATA				(EW25N0X_BASE + 0x01)
#define EW25N0X_INVALID_ARG			(EW25N0X_BASE + 0x02)
#define EW25N0X_MUTEX				(EW25N0X_BASE + 0x03)
#define EW25N0X_FLASH_BUSY			(EW25N0X_BASE + 0x04)
#define EW25N0X_SPI_NO_RESP			(EW25N0X_BASE + 0x05)
#define EW25N0X_UNEXPECTED_JEDEC_ID	(EW25N0X_BASE + 0x06)
#define EW25N0X_INSTR_HANDLING		(EW25N0X_BASE + 0x07)
#define EW25N0X_ERASE				(EW25N0X_BASE + 0x08)
#define EW25N0X_PROGRAM				(EW25N0X_BASE + 0x09)
#define EW25N0X_UNKNOWN_VARIANT		(EW25N0X_BASE + 0x0A)

#define W25N0X_PAGE_SIZE				(uint16_t)(2048)
#define W25N0X_MAX_NBR_WRITES_PER_PAGE	(uint8_t)(4)
#define W25N0X_PAGES_IN_BLOCK			(uint8_t)(64)
#define W25N0X_BLOCK_SIZE				(uint32_t)(W25N0X_PAGE_SIZE * \
													W25N0X_PAGES_IN_BLOCK)
#define W25N0X_SPARE_SIZE				64
#define W25N0X_SPARE_SIZE_MAX			128
#define W25N0X_EMPTY_BYTE				0xFF
#define W25N0X_WRITE_PAGE_TIME_MS		1
#define W25N0X_ERASE_BLOCK_TIME_MS		10
#define W25N0X_WRITE_PAGE_TMO_MS		(W25N0X_WRITE_PAGE_TIME_MS * 10)
#define W25N0X_ERASE_BLOCK_TMO_MS		(W25N0X_ERASE_BLOCK_TIME_MS * 5)

struct w25n0x_pins {
	uint8_t spi_cs;
	uint8_t spi_clk;
	uint8_t io0;
	uint8_t io1;
	uint8_t io2;
	uint8_t io3;
} __packed;

enum w25n0x_variant {
	W25N01 = 0,
	W25M02,
	W25N02,
	W25N04,
	NBR_OF_W25N0X_VARIANTS,
};

struct w25n0x_spare_data {
	uint8_t	data[W25N0X_SPARE_SIZE];
};

struct w25n0x_board {
	enum w25n0x_variant variant;
	struct w25n0x_pins pins;
	nrf_spim_frequency_t frequency;
	NRF_SPIM_Type *p_instance;
};

#if (FEAT_HW_W25N0X == 1)

err_code w25n0x_get_nbr_of_blocks(uint16_t * const p_nbr_of_blocks);
err_code w25n0x_write(const uint8_t *const p_tx_buffer,
		const uint32_t tx_buffer_length, const uint32_t dest_address);
err_code w25n0x_write_page(const uint8_t * const p_page_data,
	const uint32_t page_data_len, struct w25n0x_spare_data * const p_spare_data,
	const uint32_t block_number, const uint32_t page_number);
err_code w25n0x_write_page_spare(const struct w25n0x_spare_data * const p_data,
	const uint16_t block_number, const uint8_t page_number);
err_code w25n0x_read(uint8_t *const p_rx_buffer,
		const uint32_t rx_buffer_length, const uint32_t src_address);
err_code w25n0x_read_page_spare(struct w25n0x_spare_data * const p_data,
	bool * const p_ecc_ok, const uint16_t block_number,
	const uint8_t page_number);
err_code w25n0x_block_erase(const uint16_t block_number);
err_code w25n0x_deinit(void);
err_code w25n0x_init(const struct w25n0x_board *const p_board);

#else

__STATIC_INLINE
err_code w25n0x_get_nbr_of_blocks(uint16_t * const p_nbr_of_blocks)
	{ return EW25N0X_NO_INIT; }
__STATIC_INLINE
err_code w25n0x_write(const uint8_t *const p_tx_buffer,
		const uint32_t tx_buffer_length, const uint32_t dest_address)
	{ return EW25N0X_NO_INIT; }
__STATIC_INLINE
err_code w25n0x_write_page(const uint8_t * const p_page_data,
	const uint32_t page_data_len, struct w25n0x_spare_data * const p_spare_data,
	const uint32_t block_number, const uint32_t page_number)
	{ return EW25N0X_NO_INIT; }
__STATIC_INLINE
err_code w25n0x_write_page_spare(const struct w25n0x_spare_data * const p_data,
	const uint16_t block_number, const uint8_t page_number)
	{ return EW25N0X_NO_INIT; }
__STATIC_INLINE
err_code w25n0x_read(uint8_t *const p_rx_buffer,
		const uint32_t rx_buffer_length, const uint32_t src_address)
	{ return EW25N0X_NO_INIT; }
__STATIC_INLINE
err_code w25n0x_read_page_spare(struct w25n0x_spare_data * const p_data,
	bool * const p_ecc_ok, const uint16_t block_number,
	const uint8_t page_number)
	{ return EW25N0X_NO_INIT; }
__STATIC_INLINE
err_code w25n0x_block_erase(const uint16_t block_number)
	{ return EW25N0X_NO_INIT; }
__STATIC_INLINE
err_code w25n0x_deinit(void)
	{ return ERROR_OK; }
__STATIC_INLINE
err_code w25n0x_init(const struct w25n0x_board *const p_board)
	{ return ERROR_OK; }

#endif
