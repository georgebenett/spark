#pragma once

#include "error.h"
#include "drivers/twim.h"

#define LSM6DS3_UNEXPECTED_WHO_AM_I		(LSM6DS3_BASE + 0x00)
#define LSM6DS3_NO_INIT					(LSM6DS3_BASE + 0x01)

#define LSM6DS3_TWI_SLAVE_ADDRESS		0x6A

#if (FEAT_HW_LSM6DS3 == 1)

err_code lsm6ds3_init(const struct twim_client * const p_twim_client);

#else

__STATIC_INLINE
err_code lsm6ds3_init(const struct twim_client * const p_twim_client)
	{return ERROR_OK;}

#endif
