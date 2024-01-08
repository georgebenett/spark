#pragma once

#include <stdint.h>

#include "error.h"
#include "vedder/vesc.h"

#define ESIXP_BMS_NO_INIT					(ESIXP_BMS_BASE + 0x00)
#define ESIXP_BMS_INVALID_ARG				(ESIXP_BMS_BASE + 0x01)
#define ESIXP_BMS_MUTEX						(ESIXP_BMS_BASE + 0x02)
#define ESIXP_BMS_NO_MAC					(ESIXP_BMS_BASE + 0x03)
#define ESIXP_BMS_NO_FW_VERS				(ESIXP_BMS_BASE + 0x04)
#define ESIXP_BMS_INVALID_LEN				(ESIXP_BMS_BASE + 0x05)
#define ESIXP_BMS_BATT_TYPE					(ESIXP_BMS_BASE + 0x06)
#define ESIXP_BMS_BATT_TYPE_NOT_LISTED		(ESIXP_BMS_BASE + 0x07)
#define ESIXP_BMS_INVALID_LOW_CELL_V		(ESIXP_BMS_BASE + 0x08)
#define ESIXP_BMS_TIMER_START				(ESIXP_BMS_BASE + 0x09)
#define ESIXP_BMS_CELL_VOLT_OUT_OF_RANGE	(ESIXP_BMS_BASE + 0x10)
#define ESIXP_BMS_NOT_STARTED				(ESIXP_BMS_BASE + 0x11)
#define ESIXP_BMS_NO_CAN					(ESIXP_BMS_BASE + 0x12)
#define ESIXP_GET_MCCONF					(ESIXP_BMS_BASE + 0x13)
#define ESIXP_SET_MCCONF					(ESIXP_BMS_BASE + 0x14)

#define SIXPACK_BMS_FW_VERS_LEN			8
#define SIXP_BMS_FW_VERS_STRLEN			(SIXPACK_BMS_FW_VERS_LEN + 1)
#define SIXP_BMS_MAX_END_SOC_VAL		100
#define SIXP_BMS_MAX_CUTOFF_VAL			600

/* Current battery supplier, Sixpack, provides the CAN frame "STD_ID_16" which
 * holds the field "battery_type", informing its host of the battery type id.
 * This field will either be kept as delivered by Sixpack or dynamically set by
 * Radinn and is then referred to as a Soft Battery type. The Soft Battery types
 * have a matrix stored in the Rider Params flash which will be used for setting
 * the appropriate battery capabilities in the VESC firmware.
 */
#define SIXP_BMS_TYPE_FIRST_SOFT		0x20
#define SIXP_BMS_TYPE_LAST_SOFT			0x32
#define SIXP_BMS_TYPE_XL_XL				SIXP_BMS_TYPE_FIRST_SOFT
#define SIXP_BMS_TYPE_XL_L				0x21
#define SIXP_BMS_TYPE_XL_S				0x22
#define SIXP_BMS_TYPE_L_L				0x23
#define SIXP_BMS_TYPE_NO_CAN			0x2F
#define SIXP_BMS_TYPE_LOCKED			0x30
#define SIXP_BMS_TYPE_L_NOT_ONBOARDED	0x31
#define SIXP_BMS_TYPE_XL_NOT_ONBOARDED	SIXP_BMS_TYPE_LAST_SOFT
#define SIXP_BMS_TYPE_DEFAULT_L			0x33
#define SIXP_BMS_TYPE_DEFAULT_XL		0x45
/* Default Battery type value marking that no listed type has been detected and
 * that default cut-off start & stop will be used.
 */
#define SIXP_BMS_TYPE_DEFAULT			0xAA

/* The Battery type id range goes from 0x20 to 0x32 (soft batteries), and then
 * additionally the two pre-defined Sixpack id:s.
 */
#define SIXP_BMS_NBR_OF_BATT_TYPES		21

#define SIXP_BMS_TEMP_CUR_RTIO_ARR_SIZE		20

struct sixp_bms_batt_type {
	uint8_t					id;
	uint8_t					soc_zero_pos;
	struct vesc_batt_cutoff	cutoff;
	uint8_t					padding[2];
} __packed;

struct sixp_bms_safety_settings {
	float		low_cell_cutoff_thres;
	float		low_cell_motor_curr_limit;
	float		low_cell_ramp_down_ratio;
	float		fully_charged_cell_thres;
	uint32_t	bms_precharge_time_s;
	uint8_t		temperature_cutoff_start;
	float		temperature_cutoff_curr_ratio[SIXP_BMS_TEMP_CUR_RTIO_ARR_SIZE];
	uint8_t		padding[3];
} __packed;

struct sixp_bms_type_id {
	uint8_t	bms;
	uint8_t soft;
};

struct sixp_bms_can_id_10 {
	uint8_t		status;
	uint8_t		warning;
	uint8_t		int_resistance;
	uint8_t		total_capacity;
	uint16_t	current;
	uint16_t	total_voltage;
} __packed;

struct sixp_bms_can_id_11 {
	uint16_t	lowest_cell_voltage;
	uint8_t		lowest_cell_voltage_addr;
	uint16_t	highest_cell_voltage;
	uint8_t		highest_cell_voltage_addr;
	uint8_t		highest_int_resistance;
	uint8_t		highest_int_resistance_addr;
} __packed;

struct sixp_bms_can_id_12 {
	int8_t	temp_cell_1;
	int8_t	temp_cell_2;
	int8_t	temp_cell_3;
	int8_t	temp_ext_1;
	int8_t	temp_ext_2;
	int8_t	temp_ext_3;
	int8_t	temp_cpu;
	uint8_t	humidity;
} __packed;

struct sixp_bms_can_id_13 {
	uint64_t	mac;
} __packed;

struct sixp_bms_can_id_15 {
	uint8_t	fw_vers[SIXP_BMS_FW_VERS_STRLEN];
} __packed;

struct sixp_bms_can_id_16 {
	uint16_t	discharge_cycles;
	uint8_t		battery_type;
	uint8_t		reserved;
	uint16_t	total_charge_time;
	uint16_t	balance_status;
} __packed;

struct sixp_bms_can_data {
	struct sixp_bms_can_id_10	id_10;
	struct sixp_bms_can_id_11	id_11;
	struct sixp_bms_can_id_12	id_12;
	struct sixp_bms_can_id_13	id_13;
	struct sixp_bms_can_id_15	id_15;
	struct sixp_bms_can_id_16	id_16;
} __packed;

enum sixp_bms_event {
	SIXP_BMS_STARTED = 0,
	SIXP_BMS_NEW_TEMP_CUT_RATIO,
	SIXP_BMS_NEW_LOW_CELL_RATIO,
	NUM_SIXP_BMS_EVENTS
};

#if (FEAT_HW_SIXP_BMS == 1)

err_code sixp_bms_get_safety_settings(struct sixp_bms_safety_settings
										*const p_settings);
err_code sixp_bms_set_safety_settings(const struct sixp_bms_safety_settings
										*const p_settings);
err_code sixp_bms_get_soc(uint8_t *const p_soc);
err_code sixp_bms_get_can_params(struct sixp_bms_can_data *const p_params);
err_code sixp_bms_get_bms_mac(uint64_t *const p_mac);
err_code sixp_bms_get_fw_vers(char *const p_fw_vers, const uint8_t len);
err_code sixp_bms_type_id(struct sixp_bms_type_id *const p_id);
err_code sixp_bms_get_degradation(uint8_t *const p_degr);
err_code sixp_bms_init(void);

#else

__STATIC_INLINE
err_code sixp_bms_get_safety_settings(struct sixp_bms_safety_settings
										*const p_settings)
	{ return ESIXP_BMS_NO_INIT; }
__STATIC_INLINE
err_code sixp_bms_set_safety_settings(const struct sixp_bms_safety_settings
										*const p_settings)
	{ return ESIXP_BMS_NO_INIT; }
__STATIC_INLINE
err_code sixp_bms_get_soc(uint8_t *const p_soc)
	{ return ESIXP_BMS_NO_INIT; }
__STATIC_INLINE
err_code sixp_bms_get_can_params(struct sixp_bms_can_data *const p_params)
	{ return ESIXP_BMS_NO_INIT; }
__STATIC_INLINE
err_code sixp_bms_get_bms_mac(uint64_t *const p_mac)
	{ return ESIXP_BMS_NO_INIT; }
__STATIC_INLINE
err_code sixp_bms_get_fw_vers(char *const p_fw_vers, const uint8_t len)
	{ return ESIXP_BMS_NO_INIT; }
__STATIC_INLINE
err_code sixp_bms_type_id(struct sixp_bms_type_id *const p_id)
	{ return ESIXP_BMS_NO_INIT; }
__STATIC_INLINE
err_code sixp_bms_get_degradation(uint8_t *const p_degr)
	{ return ESIXP_BMS_NO_INIT; }
__STATIC_INLINE
err_code sixp_bms_init(void)
	{ return ERROR_OK; }

#endif
