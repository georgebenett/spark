#pragma once

#include <nrf_saadc.h>

#include "error.h"

#define ERHCBAT_NO_INIT				(ERHCBAT_BASE + 0x00)
#define ERHCBAT_INVALID_ARG			(ERHCBAT_BASE + 0x01)
#define ERHCBAT_TIMER_RESET			(ERHCBAT_BASE + 0x02)

enum rhc_bat_charge_state {
	RHC_BAT_CHARGE_STATE_ON = 0,
	RHC_BAT_CHARGE_STATE_OFF,
	NBR_RHC_BAT_CHARGE_STATES
};

struct rhc_bat_pins {
	int8_t				soc;
	int8_t				soc_meas_enable;
	int8_t				charge_enable_1;
	int8_t				charge_enable_2;
	int8_t				charge_status;
};

struct rhc_bat_board {
	struct rhc_bat_pins	pins;
};

#if (FEAT_HW_RHC_BAT == 1)

err_code rhc_bat_get_soc(uint8_t * const p_soc);
err_code rhc_bat_soc_meas_enable(void);
err_code rhc_bat_soc_meas_disable(void);
err_code rhc_bat_enable_charging(void);
err_code rhc_bat_disable_charging(void);
err_code rhc_bat_init(const struct rhc_bat_board * const p_board);

#else

__STATIC_INLINE
err_code rhc_bat_get_soc(uint8_t * const p_soc)
	{ return ERHCBAT_NO_INIT; }
__STATIC_INLINE
err_code rhc_bat_soc_meas_enable(void)
	{ return ERHCBAT_NO_INIT; }
__STATIC_INLINE
err_code rhc_bat_soc_meas_disable(void)
	{ return ERHCBAT_NO_INIT; }
__STATIC_INLINE
err_code rhc_bat_enable_charging(void)
	{ return ERHCBAT_NO_INIT; }
__STATIC_INLINE
err_code rhc_bat_disable_charging(void)
	{ return ERHCBAT_NO_INIT; }
__STATIC_INLINE
err_code rhc_bat_init(const struct rhc_bat_board * const p_board)
	{ return ERROR_OK; }

#endif
