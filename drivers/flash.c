#include <app_config.h>
#include <nrf_sdh_soc.h>
#include <nrf_sdh.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "drivers/flash.h"
#include "freertos_static.h"
#include "persistent.h"
#include "util.h"

#define DRV_NAME					FLASH
#define LOG_TAG						STRINGIFY(DRV_NAME)
#include "log.h"

#define OPERATION_TMO_MS			400
#define OPERATION_RETRY_NBR			5
#define OPERATION_RETRY_WAIT_MS		1

#define CRASH_INFO_MAGIC_NBR		((uint32_t)&_CRASH_INFO_MAGIC)

#define START_ADDR(REGION)			((uint32_t)&_ ## REGION ## _START)
#define REGION_SIZE(REGION)			((uint32_t)&_ ## REGION ## _SIZE)

#define START_PAGE(REGION)			\
	((START_ADDR(REGION) - START_ADDR(FLASH)) / FLASH_PAGE_SIZE)
#define END_PAGE(REGION)			\
	(START_PAGE(REGION) + (REGION_SIZE(REGION) / FLASH_PAGE_SIZE) - 1)

STATIC_MUTEX(DRV_NAME);

enum flash_op_response {
	FLASH_OP_SUCCESS = 0,
	FLASH_OP_FAILURE,
};

static struct {
	volatile enum flash_op_response	op_req_rsp;
	SemaphoreHandle_t				mutex_handle;
	TaskHandle_t					req_handle;
	bool							initialized;
} me;

static void on_soc_evt(uint32_t evt_id, void *p_context)
{
	if (!me.initialized)
		return;

	if (evt_id == NRF_EVT_FLASH_OPERATION_SUCCESS ||
			evt_id == NRF_EVT_FLASH_OPERATION_ERROR) {
		if (me.req_handle != NULL) {
			me.op_req_rsp = (evt_id == NRF_EVT_FLASH_OPERATION_SUCCESS) ?
				FLASH_OP_SUCCESS : FLASH_OP_FAILURE;
			xTaskNotifyGive(me.req_handle);
		}
	}
}
NRF_SDH_SOC_OBSERVER(flash_soc_obs, APP_SOC_OBSERVER_PRIO, on_soc_evt, NULL);

static err_code wait_flash_op_response(void)
{
	me.req_handle = xTaskGetCurrentTaskHandle();
	if (xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL,
			pdMS_TO_TICKS(OPERATION_TMO_MS)) != pdTRUE) {
		me.req_handle = NULL;
		return EFLASH_OPERATION_TIMEOUT;
	}

	me.req_handle = NULL;

	return (me.op_req_rsp == FLASH_OP_SUCCESS ? ERROR_OK : EFLASH_OP_REQ_FAIL);
}

static bool is_legal_page(const uint16_t page_nbr)
{
	const bool is_code_bank0 = ((page_nbr >= START_PAGE(CODE_BANK0)) &&
		(page_nbr <= END_PAGE(CODE_BANK0)));
	const bool is_code_bank1 = ((page_nbr >= START_PAGE(CODE_BANK1)) &&
		(page_nbr <= END_PAGE(CODE_BANK1)));
	const bool is_fota_info_page = (page_nbr == START_PAGE(FOTA_INFO));
	const bool is_crash_info_page = (page_nbr == START_PAGE(CRASH_INFO));
	const bool is_settings_page = (page_nbr == START_PAGE(SETTINGS));
	const bool is_vesc_tot_usg_page = (page_nbr == START_PAGE(VESC_TOT_USG));

	return (is_code_bank0 || is_code_bank1 || is_fota_info_page ||
		is_crash_info_page || is_settings_page || is_vesc_tot_usg_page);
}

uint16_t flash_get_vesc_tot_usg_page(void)
{
	return START_PAGE(VESC_TOT_USG);
}

uint32_t *flash_get_vesc_tot_usg_ptr(void)
{
	return (uint32_t *)START_ADDR(VESC_TOT_USG);
}

uint32_t *flash_get_crash_dump_ptr(void)
{
	return (uint32_t *)START_ADDR(CRASH);
}

err_code flash_get_crash_info_state(bool * const p_has_crashed)
{
	struct flash_crash_info *p_crash_info =
		(struct flash_crash_info *)START_ADDR(CRASH_INFO);

	if (!me.initialized)
		return EFLASH_NO_INIT;

	if (p_has_crashed == NULL)
		return EFLASH_INVALID_ARG;

	*p_has_crashed =
		(p_crash_info->magic == CRASH_INFO_MAGIC_NBR ? true : false);

	return ERROR_OK;

}

uint32_t *flash_get_crash_info_ptr(void)
{
	return (uint32_t *)START_ADDR(CRASH_INFO);
}

err_code flash_clear_crash_info(void)
{
	bool has_crashed;
	err_code r;

	r = flash_get_crash_info_state(&has_crashed);
	ERR_CHECK(r);

	if (!has_crashed)
		return ERROR_OK;

	return flash_erase_page(START_PAGE(CRASH_INFO));
}

uint16_t flash_get_settings_page(void)
{
	return START_PAGE(SETTINGS);
}

uint32_t *flash_get_settings_ptr(void)
{
	return (uint32_t *)START_ADDR(SETTINGS);
}

uint16_t flash_get_code_bank0_page_start(void)
{
	return START_PAGE(CODE_BANK0);
}

uint16_t flash_get_code_bank1_page_start(void)
{
	return START_PAGE(CODE_BANK1);
}

uint32_t *flash_get_code_bank0_ptr(void)
{
	return (uint32_t *)START_ADDR(CODE_BANK0);
}

uint32_t *flash_get_code_bank1_ptr(void)
{
	return (uint32_t *)START_ADDR(CODE_BANK1);
}

uint32_t flash_get_code_bank_size(void)
{
	return REGION_SIZE(CODE_BANK0);
}

uint16_t flash_get_fota_info_page(void)
{
	return START_PAGE(FOTA_INFO);
}

uint32_t *flash_get_fota_info_ptr(void)
{
	return (uint32_t *)START_ADDR(FOTA_INFO);
}

err_code flash_write_words(const uint32_t * const p_addr,
	const uint32_t * const p_data, const uint32_t nbr_words)
{
	err_code r;
	uint8_t i;

	if (!me.initialized)
		return EFLASH_NO_INIT;

	/* The sd_flash_write() API requires that both the flash address to write to
	 * and the source address to copy data from to be Flash-word-aligned.
	 */
	if ((((uint32_t)p_addr % FLASH_WORD_SIZE) > 0) ||
			(((uint32_t)p_data % FLASH_WORD_SIZE) > 0))
		return EFLASH_ADDR_ALIGN;

	if (p_data == NULL)
		return EFLASH_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EFLASH_MUTEX;

	for (i = 0; i <= OPERATION_RETRY_NBR; i++) {
		if (i == OPERATION_RETRY_NBR) {
			r = EFLASH_WRITE;
			goto exit;
		}
		if (sd_flash_write((uint32_t *)p_addr, p_data, nbr_words) ==
				NRF_SUCCESS)
			break;
		vTaskDelay(pdMS_TO_TICKS(OPERATION_RETRY_WAIT_MS));
	}

	/* If Soft Device is not enabled no event will be generated and
	 * sd_flash_write call will return NRF_SUCCESS when the write
	 * operation is completed.
	 */
	if (!nrf_sdh_is_enabled()) {
		r = ERROR_OK;
		goto exit;
	}

	r = wait_flash_op_response();

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code flash_write_page(const uint16_t page_nbr, const void * const p_data)
{
	err_code r;
	uint8_t i;

	if (!me.initialized)
		return EFLASH_NO_INIT;

	if (p_data == NULL)
		return EFLASH_INVALID_ARG;

	if (!is_legal_page(page_nbr))
		return EFLASH_ILLEGAL_PAGE;

	if (!util_mutex_lock(me.mutex_handle))
		return EFLASH_MUTEX;

	for (i = 0; i <= OPERATION_RETRY_NBR; i++) {
		if (i == OPERATION_RETRY_NBR) {
			r = EFLASH_WRITE;
			goto exit;
		}
		if (sd_flash_write((uint32_t *)(page_nbr * FLASH_PAGE_SIZE),
				p_data, (FLASH_PAGE_SIZE / sizeof(uint32_t))) == NRF_SUCCESS)
			break;
		vTaskDelay(pdMS_TO_TICKS(OPERATION_RETRY_WAIT_MS));
	}

	/* If Soft Device is not enabled no event will be generated and
	 * sd_flash_write call will return NRF_SUCCESS when the write
	 * operation is completed.
	 */
	if (!nrf_sdh_is_enabled()) {
		r = ERROR_OK;
		goto exit;
	}

	r = wait_flash_op_response();

exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code flash_erase_page(const uint16_t page_nbr)
{
	err_code r;
	uint8_t i;

	if (!me.initialized)
		return EFLASH_NO_INIT;

	if (!is_legal_page(page_nbr))
		return EFLASH_ILLEGAL_PAGE;

	if (!util_mutex_lock(me.mutex_handle))
		return EFLASH_MUTEX;

	for (i = 0; i <= OPERATION_RETRY_NBR; i++) {
		if (i == OPERATION_RETRY_NBR) {
			r = EFLASH_WRITE;
			goto exit;
		}
		if (sd_flash_page_erase(page_nbr) == NRF_SUCCESS)
			break;
		vTaskDelay(pdMS_TO_TICKS(OPERATION_RETRY_WAIT_MS));
	}

	/* If Soft Device is not enabled no event will be generated and
	 * sd_flash_page_erase call will return NRF_SUCCESS when the erase
	 * operation is completed.
	 */
	if (!nrf_sdh_is_enabled()) {
		r = ERROR_OK;
		goto exit;
	}

	r = wait_flash_op_response();
exit:
	util_mutex_unlock(me.mutex_handle);

	return r;
}

err_code flash_init(void)
{
	err_code r;

	/* Cross-check for both the FLASH_PAGE_SIZE macro and the linker script
	 * symbol _FLASH_PAGE_SIZE to fit the NRF_FICR->CODEPAGESIZE register val.
	 */
	configASSERT(NRF_FICR->CODEPAGESIZE == FLASH_PAGE_SIZE);
	configASSERT((uint32_t)&_FLASH_PAGE_SIZE == FLASH_PAGE_SIZE);

	if (me.initialized)
		return ERROR_OK;

	me.mutex_handle = CREATE_STATIC_MUTEX(DRV_NAME);
	me.initialized = true;

	return r;
}
