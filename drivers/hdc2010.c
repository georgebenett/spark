#include <FreeRTOS.h>
#include <stdlib.h>
#include <task.h>
#include <timers.h>
#include <nordic_common.h>

#include "drivers/hdc2010.h"
#include "eventpump.h"
#include "freertos_static.h"
#include "freertos_tasktracker.h"

#define DRV_NAME	HDC2010
#define LOG_TAG		STRINGIFY(DRV_NAME)
#include "log.h"

#define TEMP_MEASUREMENT_ADDR				0x00
#define TEMP_MEASUREMENT_SIZE				2
#define TEMP_CALC_FACTOR					165
#define TEMP_CALC_DIVISOR					65536
#define TEMP_CALC_OFFSET					-40
/* Temperature change-threshold used for triggering data-changed events */
#define TEMPERATURE_CHANGE_THRES			2

#define HUMIDITY_MEASUREMENT_ADDR			0x02
#define HUMIDITY_MEASUREMENT_SIZE			2
#define HUMIDITY_CALC_FACTOR				100
#define HUMIDITY_CALC_DIVISOR				65536
/* Humidity change-threshold used for triggering data-changed events */
#define HUMIDITY_CHANGE_THRES				2

#define DEVICE_ID_ADDR						0xFE
#define DEVICE_ID_SIZE						2
#define DEVICE_ID_EXPECTED					0x07D0

#define MEAS_CONF_REG_ADDR					0x0F
#define RESET_AND_DRDY_INT_CONF_REG_ADDR	0x0E

#define POLL_INTERVAL_MS					1000
#define TASK_NAME							DRV_NAME
#define TASK_PRIORITY						2
#define TASK_STACK_DEPTH					192

STATIC_TASK(TASK_NAME, TASK_STACK_DEPTH);

enum int_mode {
	LEVEL_SENSITIVE = 0,
	COMPARATOR_MODE,
};

enum int_pol {
	ACTIVE_LOW = 0,
	ACTIVE_HIGH,
};

enum drdy_int_en {
	HIGH_Z = 0,
	ENABLE,
};

enum heat_en {
	HEATER_OFF = 0,
	HEATER_ON,
};

enum auto_meas_option {
	AMM_DISABLED = 0,
	AMM_1_SAMPLE_EVERY_2_MINS,
	AMM_1_SAMPLE_EVERY_1_MIN,
	AMM_1_SAMPLE_EVERY_10_SECS,
	AMM_1_SAMPLE_EVERY_5_SECS,
	AMM_1_SAMPLE_EVERY_1_SEC,
	AMM_2_SAMPLES_EVERY_1_SEC,
	AMM_5_SAMPLES_EVERY_1_SEC,
};

enum soft_res {
	NORMAL_OPERATION = 0,
	SOFT_RESET,
};

union reset_and_drdy_int_reg {
	struct {
		uint8_t int_mode			: 1;
		uint8_t int_pol				: 1;
		uint8_t drdy_int_en			: 1;
		uint8_t heat_en				: 1;
		uint8_t auto_meas_option	: 3;
		uint8_t soft_res			: 1;
	} fields;
	uint8_t raw_value;
};

enum meas_trigger {
	NO_ACTION = 0,
	START_MEAS,
};

enum meas_conf {
	MEAS_HUMIDITY_AND_TEMP = 0,
	MEAS_TEMP_ONLY,
};

enum meas_resolution {
	RES_14_BIT = 0,
	RES_11_BIT,
	RES_9_BIT,
};

union meas_conf_reg {
	struct {
		uint8_t meas_trigger		: 1;
		uint8_t meas_conf			: 2;
		uint8_t reserved			: 1;
		uint8_t hum_res				: 2;
		uint8_t temp_res			: 2;
	} fields;

	uint8_t raw_value;
};

static const union meas_conf_reg meas_conf_reg_default = {
	.fields.meas_trigger = START_MEAS,
	.fields.meas_conf = MEAS_HUMIDITY_AND_TEMP,
	.fields.reserved = 0,
	.fields.hum_res = RES_14_BIT,
	.fields.temp_res = RES_14_BIT
};

static const union reset_and_drdy_int_reg reset_and_drdy_int_reg_default = {
	.fields.int_mode = LEVEL_SENSITIVE,
	.fields.int_pol = ACTIVE_LOW,
	.fields.drdy_int_en = HIGH_Z,
	.fields.heat_en = HEATER_OFF,
	.fields.auto_meas_option = AMM_1_SAMPLE_EVERY_1_SEC,
	.fields.soft_res = NORMAL_OPERATION,
};

static struct {
	TaskHandle_t				task_handle;
	const struct twim_client	*p_twim_client;
	int8_t						temperature;
	uint8_t						humidity;
} me;

__STATIC_INLINE uint8_t calc_humidity_perc(uint16_t humidity_raw)
{
	uint32_t humidity_calc = humidity_raw;

	humidity_calc *= HUMIDITY_CALC_FACTOR;
	humidity_calc /= HUMIDITY_CALC_DIVISOR;

	return (uint8_t)humidity_calc;
}

__STATIC_INLINE int8_t calc_temp_celc(uint16_t temp_raw)
{
	int32_t temp_calc = temp_raw;

	temp_calc *= TEMP_CALC_FACTOR;
	temp_calc /= TEMP_CALC_DIVISOR;
	temp_calc += TEMP_CALC_OFFSET;

	return (int8_t)temp_calc;
}

static err_code read_chip_data(uint8_t data_address, uint8_t *p_buffer,
		uint8_t data_size)
{
	err_code r = twim_client_write(me.p_twim_client, &data_address,
			sizeof(data_address));

	ERR_CHECK(r);

	return twim_client_read(me.p_twim_client, p_buffer,
			data_size);
}

static err_code write_chip_register(uint8_t reg_address, uint8_t reg_value)
{
	uint8_t write_payload[2];

	write_payload[0] = reg_address;
	write_payload[1] = reg_value;

	return twim_client_write(me.p_twim_client, write_payload,
			sizeof(write_payload));
}

static err_code verify_chip_id(void)
{
	uint16_t chip_id;
	err_code r = read_chip_data(DEVICE_ID_ADDR, (uint8_t *)&chip_id,
			DEVICE_ID_SIZE);
	ERR_CHECK(r);

	if (chip_id != DEVICE_ID_EXPECTED) {
		me.p_twim_client = NULL;
		LOGE("Unexpected chip id: 0x%X", chip_id);
		return EHDC2010_UNEXPECTED_DEVICE_ID;
	}

	return ERROR_OK;
}

static err_code read_temp(int8_t *const p_temp)
{
	err_code r;
	uint16_t temp_raw;

	if (me.p_twim_client == NULL)
		return EHDC2010_NO_INIT;

	if (p_temp == NULL)
		return EHDC2010_INVALID_ARG;

	r = read_chip_data(TEMP_MEASUREMENT_ADDR, (uint8_t *)&temp_raw,
			TEMP_MEASUREMENT_SIZE);

	ERR_CHECK(r);

	*p_temp = calc_temp_celc(temp_raw);

	return ERROR_OK;
}

static err_code read_humidity(uint8_t *const p_humidity)
{
	err_code r;
	uint16_t humidity_raw;

	if (me.p_twim_client == NULL)
		return EHDC2010_NO_INIT;

	if (p_humidity == NULL)
		return EHDC2010_INVALID_ARG;

	r = read_chip_data(HUMIDITY_MEASUREMENT_ADDR, (uint8_t *)&humidity_raw,
			HUMIDITY_MEASUREMENT_SIZE);

	ERR_CHECK(r);

	*p_humidity = calc_humidity_perc(humidity_raw);

	return ERROR_OK;
}

static void task(void *arg)
{
	struct eventpump_param param;
	int8_t temperature_prev = 0;
	uint8_t humidity_prev = 0;
	err_code r;

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));

		r = read_humidity(&me.humidity);
		if (r != ERROR_OK) {
			LOGE("read_humidity r=0x%08lX", r);
			continue;
		}

		if (abs(humidity_prev - me.humidity) > HUMIDITY_CHANGE_THRES) {
			param.source = EVENT_HUMIDITY;
			param.humidity.value = me.humidity;
			humidity_prev = me.humidity;
			r = eventpump_post(&param);
			if (r != ERROR_OK)
				LOGE("eventpump_post r=0x%08lX", r);
		}

		r = read_temp(&me.temperature);
		if (r != ERROR_OK) {
			LOGE("read_temp r=0x%08lX", r);
			continue;
		}

		if (abs(temperature_prev - me.temperature) > TEMPERATURE_CHANGE_THRES) {
			param.source = EVENT_TEMPERATURE;
			param.temperature.value = me.temperature;
			temperature_prev = me.temperature;
			r = eventpump_post(&param);
			if (r != ERROR_OK)
				LOGE("eventpump_post r=0x%08lX", r);
		}
	}
}

int8_t hdc2010_get_last_temp(void)
{
	return me.temperature;
}

uint8_t hdc2010_get_last_humidity(void)
{
	return me.humidity;
}

err_code hdc2010_init(const struct twim_client * const p_twim_client)
{
	err_code r;

	if (me.p_twim_client != NULL)
		return ERROR_OK;

	r = twim_client_reg(p_twim_client);
	ERR_CHECK(r);

	me.p_twim_client = p_twim_client;

	r = verify_chip_id();
	ERR_CHECK(r);

	/* Write default configuration */
	r = write_chip_register(RESET_AND_DRDY_INT_CONF_REG_ADDR,
			reset_and_drdy_int_reg_default.raw_value);
	ERR_CHECK(r);

	r = write_chip_register(MEAS_CONF_REG_ADDR,
			meas_conf_reg_default.raw_value);
	ERR_CHECK(r);

	me.task_handle = CREATE_STATIC_TASK(task, TASK_NAME, TASK_PRIORITY);

	return tasktracker_register_task(me.task_handle);
}
