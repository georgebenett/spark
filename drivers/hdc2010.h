#pragma once

#include "error.h"
#include "drivers/twim.h"

#define EHDC2010_INVALID_ARG				(EHDC2010_BASE + 0x00)
#define EHDC2010_UNEXPECTED_DEVICE_ID		(EHDC2010_BASE + 0x01)
#define EHDC2010_NO_INIT					(EHDC2010_BASE + 0x02)

#define HDC2010_TWI_SLAVE_ADDRESS			0x40

#if (FEAT_HW_HDC2010 == 1)

int8_t hdc2010_get_last_temp(void);
uint8_t hdc2010_get_last_humidity(void);
err_code hdc2010_init(const struct twim_client * const p_twim_client);

#else

__STATIC_INLINE
int8_t hdc2010_get_last_temp(void)
	{return 0;}
__STATIC_INLINE
uint8_t hdc2010_get_last_humidity(void)
	{return 0;}
__STATIC_INLINE
err_code hdc2010_init(const struct twim_client * const p_twim_client)
	{return ERROR_OK;}

#endif
