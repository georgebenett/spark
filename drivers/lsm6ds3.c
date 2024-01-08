#include "drivers/lsm6ds3.h"

#define LOG_TAG "LSM6DS3"
#include "log.h"

#define WHO_AM_I_ADDR				0x0F
#define WHO_AM_I_SIZE				1
#define WHO_AM_I_EXPECTED			0x69

static struct {
	const struct twim_client *p_twim_client;
} me;

static err_code read_chip_data(uint8_t data_address, uint8_t *p_buffer,
		uint8_t data_size)
{
	if (me.p_twim_client == NULL)
		return LSM6DS3_NO_INIT;

	err_code r = twim_client_write(me.p_twim_client, &data_address,
			sizeof(data_address));
	ERR_CHECK(r);

	return twim_client_read(me.p_twim_client, p_buffer,
			data_size);
}

static err_code verify_chip_id(void)
{
	uint8_t who_am_i;
	err_code r = read_chip_data(WHO_AM_I_ADDR, &who_am_i, WHO_AM_I_SIZE);

	ERR_CHECK(r);

	if (who_am_i != WHO_AM_I_EXPECTED) {
		LOGE("Unexpected who_am_i: 0x%X", who_am_i);
		return LSM6DS3_UNEXPECTED_WHO_AM_I;
	}

	return ERROR_OK;
}

err_code lsm6ds3_init(const struct twim_client * const p_twim_client)
{
	err_code r;

	if (me.p_twim_client != NULL)
		return ERROR_OK;

	r = twim_client_reg(p_twim_client);
	ERR_CHECK(r);

	me.p_twim_client = p_twim_client;

	r = verify_chip_id();

	if (r != ERROR_OK)
		me.p_twim_client = NULL;

	return r;
}
