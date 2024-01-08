#pragma once

#include <nrf_saadc.h>

#include <app_util_platform.h>
#include "error.h"

#define EADC_NO_INIT				(EADC_BASE + 0x00)
#define EADC_INVALID_PARAM			(EADC_BASE + 0x01)
#define EADC_CHANNEL_OVERFLOW		(EADC_BASE + 0x02)
#define EADC_SAMPLE_TMO				(EADC_BASE + 0x03)
#define EADC_HANDLE					(EADC_BASE + 0x04)
#define EADC_MUTEX					(EADC_BASE + 0x05)
#define EADC_INVALID_PIN			(EADC_BASE + 0x06)
#define EADC_INPUT_BUSY				(EADC_BASE + 0x07)

#if (FEAT_HW_ADC == 1)

err_code adc_perform_conversion(const uint32_t handle,
	int16_t * const p_result);
err_code adc_register(const uint8_t pin, uint32_t * const p_handle);
err_code adc_init(void);

#else

__STATIC_INLINE
err_code adc_perform_conversion(const uint32_t handle, int16_t * const p_result)
	{ return EADC_NO_INIT; }
__STATIC_INLINE
err_code adc_register(const uint8_t pin, uint32_t * const p_handle)
	{ return EADC_NO_INIT; }
__STATIC_INLINE
err_code adc_init(void)
	{ return ERROR_OK; }

#endif
