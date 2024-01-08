#include <app_config.h>
#include <string.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>

#include "drivers/can.h"
#include "drivers/flash.h"
#include "drivers/sixp_bms.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "freertos_tasktracker.h"
#include "persistent.h"
#include "session_mgr.h"
#include "settings.h"
#include "util.h"
#include "vedder/vesc.h"

#define MODULE_NAME								SIXP
#define LOG_TAG									STRINGIFY(MODULE_NAME)
#include "log.h"

#define MAX_NBR_CAN_CLIENT_REG_RETRIES			5
#define CAN_CLIENT_REG_RETRY_DELAY_MS			300
#define MAX_NBR_BMS_CAN_PRESENCE_CHECKS			10
#define CAN_PRESENCE_DELAY_MS					1000
#define PRECHARGE_TMR_INT_MS					1000
#define PROC_INT_DELAY_MS						1000
#define MCCONF_RETRY_DELAY_MS					78
#define MCCONF_MAX_SET_RETRIES					5

#define SOC_LEVEL_CRITICAL_LOW					10
#define MAX_CAPACITY_PERCENT					100

/* TODO: Add to remote settings handling when in place. */
#define TEMPERATURE_HYSTERESIS					3
#define TEMPERATURE_DEFAULT_CUTOFF_START		50
#define DEFAULT_PERCHARGE_TIME_S				10
#define LOW_CELL_RAMP_INIT_CURR_RATIO			(float)(1.00)
#define LOW_CELL_DEFAULT_RAMP_END_RATIO			(float)(0.20)
#define LOW_CELL_DEFAULT_RAMP_STEP_RATIO		(float)(0.10)
#define LOW_CELL_DEFAULT_CUTOFF_THRES			(float)(3.00)
#define FULLY_CHARGED_CELL_DEFAULT_THRES		(float)(4.05)
#define MAX_CELL_VOLTAGE						(float)(4.25)

/* Scales for converting fixed to float value, total-voltage with one- and
 * cell-voltage with three decimals resolution.
 */
#define BMS_TOTAL_VOLTAGE_SCALE					10
#define BMS_CELL_VOLTAGE_SCALE					(float)1000.0

#define TASK_PRIORITY							2
#define TASK_STACK_DEPTH						384

STATIC_TASK(MODULE_NAME, TASK_STACK_DEPTH);
STATIC_MUTEX(MODULE_NAME);

enum state {
	PCH_WAIT = 0,
	RUNNING,
	NBR_OF_STATES,
};

enum bms_can_id {
	BMS_CAN_ID_10	= 0x10,
	BMS_CAN_ID_11	= 0x11,
	BMS_CAN_ID_12	= 0x12,
	BMS_CAN_ID_13	= 0x13,
	BMS_CAN_ID_14	= 0x14,
	BMS_CAN_ID_15	= 0x15,
	BMS_CAN_ID_16	= 0x16
};

static const struct can_id can_id_table[] = {
	{
		.sid = BMS_CAN_ID_10,
		.eid = 0,
		.use_eid = false,
	},
	{
		.sid = BMS_CAN_ID_11,
		.eid = 0,
		.use_eid = false,
	},
	{
		.sid = BMS_CAN_ID_12,
		.eid = 0,
		.use_eid = false,
	},
	{
		.sid = BMS_CAN_ID_13,
		.eid = 0,
		.use_eid = false,
	},
	{
		.sid = BMS_CAN_ID_15,
		.eid = 0,
		.use_eid = false,
	},
	{
		.sid = BMS_CAN_ID_16,
		.eid = 0,
		.use_eid = false,
	},
};

STATIC_ASSERT_MSG((ARRAY_SIZE(can_id_table) <=
	CAN_CLIENT_MAX_NBR_IDS), "can_id_table too large.");

/* Default battery temperature Cut-Off ratio table provided by RnD with a ratio
 * going from 0.00 to 0.99 (0 - 99% deduction) and where each entry represents
 * a temperature. The start index is mapped towards temperature cut-off start
 * and each index follows One-Degree-Celsius deviations.
 */
static const float temp_cutoff_curr_ratio[SIXP_BMS_TEMP_CUR_RTIO_ARR_SIZE] = {
	1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.95, 0.90, 0.85, 0.80,
	0.75, 0.66, 0.54, 0.39, 0.21, 0.01, 0.01, 0.01, 0.01, 0.01,
};

static struct {
	struct persistent_sixp_bms		*p;
	TaskHandle_t					task_handle;
	SemaphoreHandle_t				mutex_handle;
	struct sixp_bms_safety_settings	settings;
	struct sixp_bms_batt_type		type;
	struct sixp_bms_can_data		can_msg;
	uint32_t						can_cb_handle;
	uint8_t							temperature_hysteresis_cnt;
	uint8_t							temperature_curr_ratio_idx;
	float							low_cell_motor_curr_ratio;
	uint8_t							degr_percent;
	enum state						state;
} me;

__STATIC_INLINE uint8_t remap_soc(uint8_t const soc, uint8_t const soc_min,
							uint8_t soc_max)
{
	float remapped;

	if (soc < soc_min)
		return 0;

	if (soc > MAX_CAPACITY_PERCENT)
		return MAX_CAPACITY_PERCENT;

	if (soc > soc_max)
		soc_max = soc;

	remapped = ((soc - soc_min) * (MAX_CAPACITY_PERCENT /
		(float)(soc_max - soc_min)));

	return (uint8_t)round(remapped);
}

static err_code event_handler(const struct eventpump_param *const p_event)
{
	if (p_event->vesc.event != VESC_EVT_VERIFIED)
		return ERROR_OK;

	/* Make sure that the correct battery cut-off settings are written to the
	 * VESC in case it reboots for a reason (crash or other cause).
	 */
	if (me.type.id && (me.type.id != SIXP_BMS_TYPE_DEFAULT))
		return vesc_set_battery_cutoff(VESC_BATT_CUTOFF_TOTAL_V, false,
			&me.type.cutoff);

	return ERROR_OK;
}
REGISTER_EVENT_HANDLER(EVENT_VESC, event_handler);

static void can_rx_callback(const struct can_id can_id,
	const uint8_t * const p_buf, const uint8_t size)
{
	err_code r;

	if (!p_buf || !size)
		return;

	r = ERROR_OK;
	switch (can_id.sid) {
	case BMS_CAN_ID_10:
		if (sizeof(me.can_msg.id_10) == size)
			memcpy(&me.can_msg.id_10, p_buf, size);
		break;
	case BMS_CAN_ID_11:
		if (sizeof(me.can_msg.id_11) == size)
			memcpy(&me.can_msg.id_11, p_buf, size);
		break;
	case BMS_CAN_ID_12:
		if (sizeof(me.can_msg.id_12) == size)
			memcpy(&me.can_msg.id_12, p_buf, size);
		break;
	case BMS_CAN_ID_13:
		if (!me.can_msg.id_13.mac && (sizeof(me.can_msg.id_13) == size))
			memcpy(&me.can_msg.id_13, p_buf, size);
		break;
	case BMS_CAN_ID_15:
		if (me.can_msg.id_15.fw_vers[0] || !p_buf[0] ||
				(sizeof(me.can_msg.id_15) < size))
			break;

		memcpy(&me.can_msg.id_15, p_buf, size);
		break;
	case BMS_CAN_ID_16:
		if (sizeof(me.can_msg.id_16) == size)
			memcpy(&me.can_msg.id_16, p_buf, size);
		break;
	default:
		break;
	}

	if (r != ERROR_OK) {
		LOGE("%s r=0x%08lX", __func__, r);
		session_mgr_append_err_code(r, __func__);
	}
}

static void process_batt_cell_volt_cutoff(void)
{
	struct eventpump_param param;
	struct vesc_mcconf mcconf;
	float cell_voltage;
	float curr_ratio;
	err_code r;
	uint8_t i;

	r = ERROR_OK;

	if (!me.p->low_cell_detected) {
		cell_voltage =
		(me.can_msg.id_11.lowest_cell_voltage / BMS_CELL_VOLTAGE_SCALE);

		if ((cell_voltage == 0) || (cell_voltage > MAX_CELL_VOLTAGE))
			return;

		if (cell_voltage > me.settings.low_cell_cutoff_thres)
			return;

		/* Set in persistent RAM in order to be active throughout the whole
		 * session (e.g. agnostic to system resets).
		 */
		me.p->low_cell_detected = true;
		me.low_cell_motor_curr_ratio = LOW_CELL_RAMP_INIT_CURR_RATIO;
		return;
	}

	/* Once low-cell has been detected, ramp down motor current to minimum. */
	curr_ratio =
	(me.low_cell_motor_curr_ratio - me.settings.low_cell_ramp_down_ratio);

	if (curr_ratio < me.settings.low_cell_motor_curr_limit)
		curr_ratio = me.settings.low_cell_motor_curr_limit;

	if (me.low_cell_motor_curr_ratio == curr_ratio)
		return;

	r = vesc_get_mcconf_temp(VESC_MCCONF_ENTITY_CACHE, &mcconf);
	if (r != ERROR_OK) {
		LOGE("volt_cutoff get_mcconf r=0x%08lX", r);
		session_mgr_append_err_code(ESIXP_GET_MCCONF, __func__);
		session_mgr_append_err_code(r, __func__);
		return;
	}

	mcconf.motor_curr_ratio.max = curr_ratio;
	for (i = 0; i < MCCONF_MAX_SET_RETRIES; i++) {
		r = vesc_set_mcconf_temp(&mcconf, false);
		if (r == ERROR_OK)
			break;

		LOGE("volt_cutoff set_mcconf r=0x%08lX", r);
		session_mgr_append_err_code(ESIXP_SET_MCCONF, __func__);
		session_mgr_append_err_code(r, __func__);
		vTaskDelay(pdMS_TO_TICKS(MCCONF_RETRY_DELAY_MS << i));
	}
	if (r != ERROR_OK)
		return;

	me.low_cell_motor_curr_ratio = curr_ratio;
	param.source = EVENT_SIXP_BMS;
	param.sixp_bms.event = SIXP_BMS_NEW_LOW_CELL_RATIO;
	param.sixp_bms.ratio = (curr_ratio * 100);
	r = eventpump_post(&param);
	if (r != ERROR_OK) {
		LOGE("volt_cutoff eventpump r=0x%08lX", r);
		session_mgr_append_err_code(r, __func__);
	}
}

static void process_batt_temperature_cutoff(void)
{
	struct eventpump_param param;
	struct vesc_mcconf mcconf;
	int8_t batt_temperature;
	float motor_input_curr;
	uint8_t idx;
	err_code r;

	/* Make sure that we act upon the highest temperature */
	batt_temperature = me.can_msg.id_12.temp_cell_1;
	if (me.can_msg.id_12.temp_cell_2 > batt_temperature)
		batt_temperature = me.can_msg.id_12.temp_cell_2;
	if (me.can_msg.id_12.temp_cell_3 > batt_temperature)
		batt_temperature = me.can_msg.id_12.temp_cell_3;
	/* According to sixp_bms temp_ext_2 is temp_cell_4 */
	if (me.can_msg.id_12.temp_ext_2 > batt_temperature)
		batt_temperature = me.can_msg.id_12.temp_ext_2;

	if (batt_temperature < me.settings.temperature_cutoff_start)
		return;

	idx = (batt_temperature - me.settings.temperature_cutoff_start);
	if (idx >= ARRAY_SIZE(me.settings.temperature_cutoff_curr_ratio))
		idx = (ARRAY_SIZE(me.settings.temperature_cutoff_curr_ratio) - 1);

	/* Added inertia to make sure that the change in temperature is stable
	 * before writing a new battery-current limit to vesc.
	 */
	if (idx != me.temperature_curr_ratio_idx)
		me.temperature_hysteresis_cnt++;
	else
		me.temperature_hysteresis_cnt = 0;

	if (me.temperature_hysteresis_cnt < TEMPERATURE_HYSTERESIS)
		return;

	me.temperature_hysteresis_cnt = 0;
	me.temperature_curr_ratio_idx = idx;

	r = vesc_get_mcconf_temp(VESC_MCCONF_ENTITY_CACHE, &mcconf);
	if (r != ERROR_OK) {
		LOGE("temp_cutoff get_mcconf r=0x%08lX", r);
		session_mgr_append_err_code(r, __func__);
		return;
	}

	/* The motor input current shall be set as a deduction ratio relative to the
	 * base input current stored in the flash rider params.
	 */
	r = settings_get_motor_input_curr_max(&motor_input_curr);
	if (r != ERROR_OK) {
		LOGE("temp_cutoff get_motor_input_curr_max r=0x%08lX", r);
		session_mgr_append_err_code(r, __func__);
		return;
	}

	mcconf.input_curr.max =
	(me.settings.temperature_cutoff_curr_ratio[idx] * motor_input_curr);
	r = vesc_set_mcconf_temp(&mcconf, false);
	if (r != ERROR_OK) {
		LOGE("temp_cutoff set_mcconf r=0x%08lX", r);
		session_mgr_append_err_code(r, __func__);
		return;
	}

	param.source = EVENT_SIXP_BMS;
	param.sixp_bms.event = SIXP_BMS_NEW_TEMP_CUT_RATIO;
	param.sixp_bms.ratio =
		(me.settings.temperature_cutoff_curr_ratio[idx] * 100);
	r = eventpump_post(&param);
	if (r != ERROR_OK) {
		LOGE("temp_cutoff eventpump r=0x%08lX", r);
		session_mgr_append_err_code(r, __func__);
	}
}

static err_code set_battery_type_params(void)
{
	struct sixp_bms_batt_type batt_table[SIXP_BMS_NBR_OF_BATT_TYPES];
	uint8_t bms_batt_type;
	err_code r;
	uint8_t i;

	bms_batt_type = me.can_msg.id_16.battery_type;
	me.type.id = SIXP_BMS_TYPE_DEFAULT;
	r = ERROR_OK;

	if (!me.can_msg.id_13.mac && !bms_batt_type)
		me.type.id = SIXP_BMS_TYPE_NO_CAN;
	else if (bms_batt_type == SIXP_BMS_TYPE_DEFAULT_L ||
			bms_batt_type == SIXP_BMS_TYPE_DEFAULT_XL ||
			(bms_batt_type >= SIXP_BMS_TYPE_FIRST_SOFT &&
				bms_batt_type <= SIXP_BMS_TYPE_LAST_SOFT))
		me.type.id = bms_batt_type;
	else
		r = ESIXP_BMS_BATT_TYPE;

	if (r != ERROR_OK)
		goto exit;

	/* Populate the matching battery type parameters according to the stored
	 * matrix in the rider-flash-area.
	 */
	r = settings_get_batt_table(batt_table, SIXP_BMS_NBR_OF_BATT_TYPES);
	ERR_CHECK(r);

	for (i = 0; i < SIXP_BMS_NBR_OF_BATT_TYPES; i++) {
		if (batt_table[i].id != me.type.id)
			continue;

		me.type.cutoff = batt_table[i].cutoff;
		me.type.soc_zero_pos = batt_table[i].soc_zero_pos;

		r = vesc_set_battery_cutoff(VESC_BATT_CUTOFF_TOTAL_V, false,
			&me.type.cutoff);
		goto exit;
	}

	/* Suppress error if the not listed battery type is one of the default L/XL
	 * Sixpack types. Potentially these types could also be used with soft
	 * settings, but most likely not.
	 */
	if ((bms_batt_type == SIXP_BMS_TYPE_DEFAULT_L) ||
			(bms_batt_type == SIXP_BMS_TYPE_DEFAULT_XL))
		r = ERROR_OK;
	else
		r = ESIXP_BMS_BATT_TYPE_NOT_LISTED;

exit:
	return r;
}

static void task(void *arg)
{
	struct can_rx_client can_clnt;
	struct eventpump_param param;
	uint8_t initial_bms_soc;
	float highest_cell_v;
	uint32_t uptime_s;
	err_code r;
	uint8_t i;

	uptime_s = 0;
	can_clnt.cb = can_rx_callback;
	can_clnt.nbr_ids = ARRAY_SIZE(can_id_table);
	memcpy(can_clnt.can_id, can_id_table, sizeof(can_id_table));
	for (i = 0; i < MAX_NBR_CAN_CLIENT_REG_RETRIES; i++) {
		r = can_register_rx_client(&me.can_cb_handle, &can_clnt);
		if (r == ERROR_OK)
			break;
		vTaskDelay(pdMS_TO_TICKS(CAN_CLIENT_REG_RETRY_DELAY_MS));
	}

	if (i >= MAX_NBR_CAN_CLIENT_REG_RETRIES) {
		LOGE("%s: reg can clnt r=0x%08lX", __func__, r);
		session_mgr_append_err_code(r, __func__);
		configASSERT(0);
	}

	/* Give the BMS a chance to provide its battery type before processing the
	 * battery type.
	 */
	for (i = 0; i < MAX_NBR_BMS_CAN_PRESENCE_CHECKS; i++) {
		if (me.can_msg.id_16.battery_type && me.can_msg.id_13.mac)
			break;
		vTaskDelay(pdMS_TO_TICKS(CAN_PRESENCE_DELAY_MS));
	}
	if (i >= MAX_NBR_BMS_CAN_PRESENCE_CHECKS)
		session_mgr_append_err_code(ESIXP_BMS_NO_CAN, __func__);

	r = set_battery_type_params();
	if (r != ERROR_OK) {
		LOGW("%s: set type r=0x%08lX", __func__, r);
		session_mgr_append_err_code(r, __func__);
	}

	if (!persistent_get_ptr_sixp_bms()->bms_mac)
		persistent_get_ptr_sixp_bms()->bms_mac = me.can_msg.id_13.mac;

	/* Make sure to store the SoC MAX value once per session with an algorithm
	 * as followed: If the highest cell voltage is above a "fully charged cell
	 * voltage threshold" then the SoC is estimated to have
	 * MAX_CAPACITY_PERCENT (e.g. despite of its provided total capacity value),
	 * else the SoC falls back to it's BMS representation.
	 */
	if (!me.p->soc_max && (me.type.id != SIXP_BMS_TYPE_NO_CAN)) {
		me.p->soc_max = MAX_CAPACITY_PERCENT;
		for (i = 0; i < MAX_NBR_BMS_CAN_PRESENCE_CHECKS; i++) {
			vTaskDelay(pdMS_TO_TICKS(CAN_PRESENCE_DELAY_MS));
			highest_cell_v = (me.can_msg.id_11.highest_cell_voltage /
				BMS_CELL_VOLTAGE_SCALE);

			if (!highest_cell_v || (highest_cell_v >= MAX_CELL_VOLTAGE))
				continue;

			initial_bms_soc = me.can_msg.id_10.total_capacity;

			if (!initial_bms_soc || (initial_bms_soc > MAX_CAPACITY_PERCENT))
				continue;

			if (highest_cell_v < me.settings.fully_charged_cell_thres)
				break;

			me.p->soc_max = initial_bms_soc;
			break;
		}

		/* Log the estimated battery degradation and the provided BMS discharge
		 * cycles once per session. Battery degradation is the difference
		 * between the actual capacity and the estimated maximum capacity.
		 */
		me.degr_percent = (MAX_CAPACITY_PERCENT - me.p->soc_max);
	}

	LOGI("type BMS:0x%02X SOFT:0x%02X, SoC_MAX:%d",
		me.can_msg.id_16.battery_type, me.type.id, me.p->soc_max);

	while (true) {
		switch (me.state) {
		case PCH_WAIT:
			r = walltime_get_uptime(&uptime_s, NULL);
			if (r != ERROR_OK)
				session_mgr_append_err_code(r, __func__);

			if (uptime_s >= me.settings.bms_precharge_time_s) {
				me.state = RUNNING;

				if(!me.p->dc_offset_calibrated) {
					r = vesc_calibrate_dc_offset_foc(true);
					if (r == ERROR_OK)
						me.p->dc_offset_calibrated = true;
					else
						session_mgr_append_err_code(r, __func__);
				}

				param.source = EVENT_SIXP_BMS;
				param.sixp_bms.event = SIXP_BMS_STARTED;
				r = eventpump_post(&param);
				if (r != ERROR_OK)
					session_mgr_append_err_code(r, __func__);
			}
			break;
		case RUNNING:
			process_batt_temperature_cutoff();
			process_batt_cell_volt_cutoff();
			break;
		default:
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(PROC_INT_DELAY_MS));
	}
}

err_code sixp_bms_get_safety_settings(struct sixp_bms_safety_settings
										*const p_settings)
{
	if (!me.p)
		return ESIXP_BMS_NO_INIT;

	if (!p_settings)
		return ESIXP_BMS_INVALID_ARG;

	*p_settings = me.settings;

	return ERROR_OK;
}

err_code sixp_bms_set_safety_settings(const struct sixp_bms_safety_settings
										*const p_settings)
{
	err_code r;

	if (!me.p)
		return ESIXP_BMS_NO_INIT;

	if (!p_settings)
		return ESIXP_BMS_INVALID_ARG;

	if (p_settings->low_cell_cutoff_thres >= MAX_CELL_VOLTAGE)
		r = ESIXP_BMS_INVALID_LOW_CELL_V;

	/* TODO: Add validation of max/min temperature to be set. */

	r = settings_set_batt_settings(p_settings);

	if (r == ERROR_OK)
		me.settings = *p_settings;

	return r;
}

err_code sixp_bms_get_soc(uint8_t *p_soc)
{
	uint8_t soc;

	if (!me.p)
		return ESIXP_BMS_NO_INIT;

	if (!p_soc)
		return ESIXP_BMS_INVALID_ARG;

	if (me.state != RUNNING) {
		*p_soc = 0;
		return ERROR_OK;
	}

	/* Force SoC to critical low upon low-cell voltage detection. */
	if (me.p->low_cell_detected) {
		*p_soc = SOC_LEVEL_CRITICAL_LOW;
		return ERROR_OK;
	}

	if (!util_mutex_lock(me.mutex_handle))
		return ESIXP_BMS_MUTEX;

	soc = (me.can_msg.id_10.total_capacity <=
		MAX_CAPACITY_PERCENT) ? me.can_msg.id_10.total_capacity : 0;
	*p_soc = remap_soc(soc, me.type.soc_zero_pos, me.p->soc_max);

	util_mutex_unlock(me.mutex_handle);

	return ERROR_OK;
}

err_code sixp_bms_get_can_params(struct sixp_bms_can_data *const p_params)
{
	if (!me.p)
		return ESIXP_BMS_NO_INIT;

	if (!p_params)
		return ESIXP_BMS_INVALID_ARG;

	*p_params  = me.can_msg;

	return ERROR_OK;
}

err_code sixp_bms_get_bms_mac(uint64_t *const p_mac)
{
	if (!me.p)
		return ESIXP_BMS_NO_INIT;

	if (!p_mac)
		return ESIXP_BMS_INVALID_ARG;

	if (!me.can_msg.id_13.mac)
		return ESIXP_BMS_NO_MAC;

	*p_mac = persistent_get_ptr_sixp_bms()->bms_mac;

	return ERROR_OK;
}

err_code sixp_bms_get_fw_vers(char *const p_fw_vers, const uint8_t len)
{
	int8_t str_len;

	if (!me.p)
		return ESIXP_BMS_NO_INIT;

	if (!p_fw_vers)
		return ESIXP_BMS_INVALID_ARG;

	if (!me.can_msg.id_15.fw_vers[0])
		return ESIXP_BMS_NO_FW_VERS;

	if (len < SIXP_BMS_FW_VERS_STRLEN)
		return ESIXP_BMS_INVALID_LEN;

	str_len	= snprintf(p_fw_vers, SIXP_BMS_FW_VERS_STRLEN, "%s",
			&me.can_msg.id_15.fw_vers[0]);
	if (str_len < 0)
		return ESIXP_BMS_INVALID_LEN;

	return ERROR_OK;
}

err_code sixp_bms_type_id(struct sixp_bms_type_id *const p_id)
{
	if (!me.p)
		return ESIXP_BMS_NO_INIT;

	if (!p_id)
		return ESIXP_BMS_INVALID_ARG;

	if (me.state != RUNNING)
		return ESIXP_BMS_NOT_STARTED;

	p_id->bms = me.can_msg.id_16.battery_type;
	p_id->soft = me.type.id;

	return ERROR_OK;
}

err_code sixp_bms_get_degradation(uint8_t *const p_degr)
{
	if (!me.p)
		return ESIXP_BMS_NO_INIT;

	if (!p_degr)
		return ESIXP_BMS_INVALID_ARG;

	if (me.state != RUNNING)
		return ESIXP_BMS_NOT_STARTED;

	*p_degr = me.degr_percent;

	return ERROR_OK;
}

err_code sixp_bms_init(void)
{
	struct sixp_bms_safety_settings settings;
	err_code r;

	if (me.p)
		return ERROR_OK;

	r = settings_get_batt_settings(&settings);
	ERR_CHECK(r);

	if (*((uint32_t *)&settings.low_cell_cutoff_thres) ==
			FLASH_EMPTY_WORD)
		me.settings.low_cell_cutoff_thres = LOW_CELL_DEFAULT_CUTOFF_THRES;
	else
		me.settings.low_cell_cutoff_thres = settings.low_cell_cutoff_thres;

	if (*((uint32_t *)&settings.low_cell_motor_curr_limit) ==
			FLASH_EMPTY_WORD)
		me.settings.low_cell_motor_curr_limit = LOW_CELL_DEFAULT_RAMP_END_RATIO;
	else
		me.settings.low_cell_motor_curr_limit =
			settings.low_cell_motor_curr_limit;

	if (*((uint32_t *)&settings.low_cell_ramp_down_ratio) ==
			FLASH_EMPTY_WORD)
		me.settings.low_cell_ramp_down_ratio = LOW_CELL_DEFAULT_RAMP_STEP_RATIO;
	else
		me.settings.low_cell_ramp_down_ratio =
			settings.low_cell_ramp_down_ratio;

	if (settings.temperature_cutoff_start == FLASH_EMPTY_BYTE)
		me.settings.temperature_cutoff_start = TEMPERATURE_DEFAULT_CUTOFF_START;
	else
		me.settings.temperature_cutoff_start =
			settings.temperature_cutoff_start;

	if (settings.bms_precharge_time_s == FLASH_EMPTY_WORD)
		me.settings.bms_precharge_time_s = DEFAULT_PERCHARGE_TIME_S;
	else
		me.settings.bms_precharge_time_s = settings.bms_precharge_time_s;

	if (*((uint32_t *)&settings.fully_charged_cell_thres) ==
			FLASH_EMPTY_WORD)
		me.settings.fully_charged_cell_thres = FULLY_CHARGED_CELL_DEFAULT_THRES;
	else
		me.settings.fully_charged_cell_thres =
			settings.fully_charged_cell_thres;

	memset(&me.settings.temperature_cutoff_curr_ratio[0], FLASH_EMPTY_BYTE,
		sizeof(me.settings.temperature_cutoff_curr_ratio));
	if (memcmp(&me.settings.temperature_cutoff_curr_ratio[0],
			&settings.temperature_cutoff_curr_ratio[0],
			sizeof(me.settings.temperature_cutoff_curr_ratio)) != 0)
		memcpy(&me.settings.temperature_cutoff_curr_ratio[0],
			&settings.temperature_cutoff_curr_ratio[0],
			sizeof(me.settings.temperature_cutoff_curr_ratio));
	else
		memcpy(&me.settings.temperature_cutoff_curr_ratio[0],
			&temp_cutoff_curr_ratio[0],
			sizeof(me.settings.temperature_cutoff_curr_ratio));

	me.mutex_handle = CREATE_STATIC_MUTEX(MODULE_NAME);
	me.task_handle = CREATE_STATIC_TASK(task, MODULE_NAME, TASK_PRIORITY);

	r =  tasktracker_register_task(me.task_handle);
	if (r == ERROR_OK)
		me.p = persistent_get_ptr_sixp_bms();

	return r;
}
