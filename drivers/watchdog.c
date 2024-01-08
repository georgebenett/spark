#include <FreeRTOS.h>
#include <task.h>

#include <nrf_drv_common.h>
#include <nrf_rtc.h>
#include <nrf_wdt.h>

#include "errorhandler.h"
#include "freertos_static.h"
#include "freertos_tasktracker.h"
#include "persistent.h"
#include "watchdog.h"

#define DRV_NAME			WDOG
#define LOG_TAG				STRINGIFY(DRV_NAME)
#include "log.h"

#define RELOAD_TIME_MS		30000
#define FEED_INTERVAL_MS	20000
#define MS_IN_SEC			1000
#define MS_TO_WDG_TICKS(x)	(((uint64_t)(x) * (LFCLK_FREQ_HZ)) / (MS_IN_SEC))

#define TASK_NAME			DRV_NAME
#define TASK_PRIORITY		1
#define TASK_STACK_DEPTH	196

STATIC_TASK(TASK_NAME, TASK_STACK_DEPTH);

static struct {
	TaskHandle_t	task_handle;
	bool			initialized;
} me;

STATIC_ASSERT_MSG((WDT_CONFIG_IRQ_PRIORITY == 0),
	"Ensure the watchdog interrupt has the highest possible priority.");

/* We only have Two 32khz clockcycles before the Watchdog issues the RESET,
 * this means we can only store the absolute necessary things!
 * See section "Watchdog reset" in the corresponding nRF52 Product Spec.
 */
void WDT_IRQHandlerC(uint32_t *hardfault_args, uint32_t lr)
{
	struct persistent_diag * const p_diag = persistent_get_ptr_diag_data();
	struct hardfault_info * const p_hardfault_info =
		&persistent_get_ptr_crash_info()->hardfault;
	uint8_t offset;

	NRF_POWER->GPREGRET = CRASH_TYPE_WATCHDOG;

	offset = 0;
	/* On IRQ entry LR is set to a special EXC_RETURN value.
	 * If FPU Context was used, EXC_RETURN bit 4 will be set to 0.
	 * This means 18 extra FPU registers have been pushed to stack.
	 * To get the previous stack pointer, we need to offset it.
	 */
	if ((lr & (1 << 4)) == 0)
		offset += 18;

	p_hardfault_info->r0  = hardfault_args[0];
	p_hardfault_info->r1  = hardfault_args[1];
	p_hardfault_info->r2  = hardfault_args[2];
	p_hardfault_info->r3  = hardfault_args[3];
	p_hardfault_info->r12 = hardfault_args[4];
	p_hardfault_info->lr  = hardfault_args[5];
	p_hardfault_info->pc  = hardfault_args[6];
	p_hardfault_info->psr = hardfault_args[7];
	p_hardfault_info->sp  = (uint32_t)&hardfault_args[offset + 8];

	p_diag->crash_type = CRASH_TYPE_WATCHDOG;
	p_diag->has_crashed = true;
}

/* The prototype shows it is a naked function -
 * in effect this is just an assembly function.
 */
void WDT_IRQHandler(void) __attribute__((naked));

void WDT_IRQHandler(void)
{
	/*
	 * Get the appropriate stack pointer, depending on our mode,
	 * and use it as the parameter to the C handler. This function
	 * will never return
	 */
	__ASM volatile(
		"MOVS	R0, #4\n"
		"MOV	R1, LR\n"
		"TST	R0, R1\n"
		"BEQ	_MSP\n"
		"MRS	R0, PSP\n"
		"B		WDT_IRQHandlerC\n"
		"_MSP:\n"
		"MRS	R0, MSP\n"
		"B		WDT_IRQHandlerC\n"
	);
}

static void task(void *arg)
{
	TickType_t tick_count;

	UNUSED_PARAMETER(arg);

	tick_count = xTaskGetTickCount();
	for (;;) {
		watchdog_feed();
		vTaskDelayUntil(&tick_count, pdMS_TO_TICKS(FEED_INTERVAL_MS));
	}
}

err_code watchdog_feed(void)
{
	if (!me.initialized)
		return EWDT_NO_INIT;

	nrf_wdt_reload_request_set((nrf_wdt_rr_register_t)(NRF_WDT_RR0));
	return ERROR_OK;
}

err_code watchdog_init(void)
{
	uint64_t reload_value_ticks;
	err_code r;

	if (me.initialized)
		return ERROR_OK;

	reload_value_ticks = MS_TO_WDG_TICKS(RELOAD_TIME_MS);
	if (reload_value_ticks > UINT32_MAX)
		return EWDT_RELOAD_VAL;

	nrf_wdt_behaviour_set(NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT);
	nrf_wdt_reload_value_set(reload_value_ticks);

	NRFX_IRQ_PRIORITY_SET(WDT_IRQn, WDT_CONFIG_IRQ_PRIORITY);
	NRFX_IRQ_ENABLE(WDT_IRQn);

	nrf_wdt_reload_request_enable((nrf_wdt_rr_register_t)NRF_WDT_RR0);

	nrf_wdt_int_enable(NRF_WDT_INT_TIMEOUT_MASK);
	nrf_wdt_task_trigger(NRF_WDT_TASK_START);

	me.task_handle = CREATE_STATIC_TASK(task, TASK_NAME, TASK_PRIORITY);

	r = tasktracker_register_task(me.task_handle);
	ERR_CHECK(r);

	me.initialized = true;

	return ERROR_OK;
}
