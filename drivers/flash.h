#pragma once

#include <app_util.h>
#include <nrf_gzp.h>

#include "error.h"
#include "fota_types.h"
#include "session_wrtr.h"
#include "vedder/vesc.h"
#include "walltime.h"

#define EFLASH_NO_INIT					(EFLASH_BASE + 0x00)
#define EFLASH_INVALID_ARG				(EFLASH_BASE + 0x01)
#define EFLASH_ILLEGAL_PAGE				(EFLASH_BASE + 0x02)
#define EFLASH_MUTEX					(EFLASH_BASE + 0x03)
#define EFLASH_OPERATION_TIMEOUT		(EFLASH_BASE + 0x04)
#define EFLASH_WRITE					(EFLASH_BASE + 0x05)
#define EFLASH_ERASE					(EFLASH_BASE + 0x06)
#define EFLASH_OP_REQ_FAIL				(EFLASH_BASE + 0x07)
#define EFLASH_ADDR_ALIGN				(EFLASH_BASE + 0x08)
#define EFLASH_ADDR_NOT_EMPTY			(EFLASH_BASE + 0x09)

#define FLASH_WORD_SIZE					(uint8_t)(0x04)
#define FLASH_EMPTY_BYTE				(uint8_t)(0xFF)
#define FLASH_EMPTY_WORD				(uint32_t)(0xFFFFFFFF)

#define FLASH_FOTA_INFO_NBR_WORDS		(sizeof(struct flash_fota_info) / \
	FLASH_WORD_SIZE)
#define FLASH_VESC_TOT_USG_NBR_WORDS	(sizeof(struct vesc_tot_usg) / \
	FLASH_WORD_SIZE)

/* Padding replaces legacy flash_gzll_params for backwards compatibility. */
#define FLASH_VESC_TOT_USG_PADDING		12
#define FLASH_VESC_TOT_USG_MAX_ENTRIES \
	((FLASH_PAGE_SIZE - FLASH_VESC_TOT_USG_PADDING) / \
	sizeof(struct vesc_tot_usg))

enum flash_fota_code_bank {
	FLASH_FOTA_CODE_BANK0 = 0,
	FLASH_FOTA_CODE_BANK1,
	NBR_FOTA_CODE_BANKS
};

struct flash_fota_info {
	uint32_t					fota_magic;
	struct fota_bin_hdr			bin_header;
	uint8_t						padding;
} __packed;

STATIC_ASSERT_MSG((sizeof(struct flash_fota_info) % FLASH_WORD_SIZE == 0),
	"flash_fota_info must be word aligned.");

struct flash_crash_info {
	uint32_t magic;
	uint32_t type;
	uint32_t size;
	uint32_t crc;
} __packed;

STATIC_ASSERT_MSG((sizeof(struct flash_crash_info) % FLASH_WORD_SIZE == 0),
	"flash_crash_info must be word aligned.");

struct flash_vesc_tot_usg_page {
	uint8_t				padding[FLASH_VESC_TOT_USG_PADDING];
	struct vesc_tot_usg	tot_usg[FLASH_VESC_TOT_USG_MAX_ENTRIES];
} __packed;

STATIC_ASSERT_MSG((sizeof(struct flash_vesc_tot_usg_page) % FLASH_WORD_SIZE ==
	0), "flash_vesc_tot_usg_page must be word aligned.");
STATIC_ASSERT_MSG((sizeof(struct flash_vesc_tot_usg_page) <= FLASH_PAGE_SIZE),
	"flash_vesc_tot_usg_page must be fit into one flash page.");

uint16_t flash_get_vesc_tot_usg_page(void);
uint32_t *flash_get_vesc_tot_usg_ptr(void);
uint32_t *flash_get_crash_dump_ptr(void);
err_code flash_get_crash_info_state(bool * const p_has_crashed);
uint32_t *flash_get_crash_info_ptr(void);
err_code flash_clear_crash_info(void);
uint16_t flash_get_settings_page(void);
uint32_t *flash_get_settings_ptr(void);
uint16_t flash_get_code_bank0_page_start(void);
uint16_t flash_get_code_bank1_page_start(void);
uint32_t *flash_get_code_bank0_ptr(void);
uint32_t *flash_get_code_bank1_ptr(void);
uint32_t flash_get_code_bank_size(void);
uint16_t flash_get_fota_info_page(void);
uint32_t *flash_get_fota_info_ptr(void);
err_code flash_write_words(const uint32_t * const p_addr,
	const uint32_t * const p_data, const uint32_t nbr_words);
err_code flash_write_page(const uint16_t page_nbr, const void * const p_data);
err_code flash_erase_page(const uint16_t page_nbr);
err_code flash_init(void);
