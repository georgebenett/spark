#pragma once

#include <nrfx_power.h>
#include <nrf_lpcomp.h>

#include "error.h"

#define EPOWER_NO_INIT					(EPOWER_BASE + 0x00)
#define EPOWER_INVALID_ARG				(EPOWER_BASE + 0x01)
#define EPOWER_ON_OFF_IRQ_CB_IN_USE		(EPOWER_BASE + 0x02)
#define EPOWER_MUTEX					(EPOWER_BASE + 0x03)
#define EPOWER_REG_EN_PIN_NOT_PRESENT	(EPOWER_BASE + 0x04)
#define EPOWER_REG_5V_HANDLE			(EPOWER_BASE + 0x05)

enum power_batt_eject_type {
	POWER_BATT_EJECT_POFWARN,
	POWER_BATT_EJECT_EXT_PIN,
	NUM_POWER_BATT_EJECT_TYPES,
};

enum power_event {
	POWER_OFF = 0,
	POWER_ON,
	POWER_EXT_EVT_PIN_HIGH,
	POWER_EXT_PIN_EVT_LOW,
	NBR_OF_POWER_EVENTS,
};

struct power_pins {
	/* The plug detect 1 pin is not used in SW but if its connected on the HW
	 * it must be properly initiated.
	 */
	int8_t	plug_detect1;
	int8_t	ext_evt;
	int8_t	regulator_enable;
	int8_t	regulator_5v_enable;
	int8_t	regulator_24v_enable;
	int8_t	regulator_can_enable;
} __packed;

struct power_board {
	enum power_batt_eject_type	batt_eject_type;
	nrfx_power_config_t			config;
	nrf_lpcomp_input_t			lpcomp_input;
	struct power_pins			pins;
};

typedef void (*power_event_cb)(const enum power_event event);

err_code power_register_on_off_irq_cb(const power_event_cb cb);
err_code power_read_ext_evt_pin(bool * const p_level);
err_code power_get_regulator_enable_state(bool * const p_state);
err_code power_set_regulator_enable_state(const bool state);
err_code power_get_regulator_5v_enable_state(bool * const p_state);
err_code power_set_regulator_5v_enable_state(const bool state,
	const uint8_t handle);
err_code power_register_regulator_5v(uint8_t * const p_handle);
err_code power_get_regulator_24v_enable_state(bool * const p_state);
err_code power_set_regulator_24v_enable_state(const bool state);
err_code power_set_wakeup_on_ext_evt_pin(const bool level);
err_code power_get_reset_reason(uint32_t * const p_reason);
err_code power_get_regulator_can_enable_state(bool * const p_state);
err_code power_set_regulator_can_enable_state(const bool state);
bool power_is_initiated(void);
err_code power_init(const struct power_board * const p_board);
