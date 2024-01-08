#pragma once

#include <nrf_spim.h>

#include "error.h"

#define EA2235_PDATA					(EA2235_BASE + 0x00)
#define EA2235_NO_INIT					(EA2235_BASE + 0x01)
#define EA2235_NO_VERIF					(EA2235_BASE + 0x02)
#define EA2235_FW_VERSION				(EA2235_BASE + 0x03)
#define EA2235_SPI_TRX					(EA2235_BASE + 0x04)
#define EA2235_SPI_TRX_INVALID_BUF		(EA2235_BASE + 0x05)
#define EA2235_SPI_RAM_BUF				(EA2235_BASE + 0x06)
#define EA2235_SPI_TMO					(EA2235_BASE + 0x07)
#define EA2235_INVALID_ARG				(EA2235_BASE + 0x08)
#define EA2235_INVALID_NMEA_RSP			(EA2235_BASE + 0x09)
#define EA2235_MUTEX					(EA2235_BASE + 0x0A)
#define EA2235_SPI_INIT					(EA2235_BASE + 0x0B)
#define EA2235_SPI_INSTANCE				(EA2235_BASE + 0x0C)
#define EA2235_TMR_START				(EA2235_BASE + 0x0D)
#define EA2235_TMR_UPDATE				(EA2235_BASE + 0x0E)
#define EA2235_TMR_STOP					(EA2235_BASE + 0x0F)
#define EA2235_QUEUE_RCV_TMO			(EA2235_BASE + 0x10)
#define EA2235_NMEA_ENCODE_PAYLOAD		(EA2235_BASE + 0x11)
#define EA2235_OK_TO_SEND				(EA2235_BASE + 0x12)
#define EA2235_QUEUE_FULL				(EA2235_BASE + 0x13)
#define EA2235_WAKEUP_TMO				(EA2235_BASE + 0x14)
#define EA2235_WAKEUP_NOT_ACTIVE		(EA2235_BASE + 0x15)

struct pins_gps_a2235 {
	uint8_t on;
	uint8_t rst;
	uint8_t wakeup;
	uint8_t spi_cs;
	uint8_t spi_clk;
	uint8_t spi_mosi;
	uint8_t spi_miso;
} __packed;

struct gps_a2235_board {
	struct pins_gps_a2235	pins;
	nrf_spim_frequency_t	frequency;
	NRF_SPIM_Type			*p_instance;
};

#if (FEAT_HW_GPS_A2235_H == 1)

err_code gps_a2235_start(void);
err_code gps_a2235_stop(void);
err_code gps_a2235_init(const struct gps_a2235_board *const p_board);

#else

__STATIC_INLINE
err_code gps_a2235_start(void)
	{return EA2235_NO_INIT;}
__STATIC_INLINE
err_code gps_a2235_stop(void)
	{return EA2235_NO_INIT;}
__STATIC_INLINE
err_code gps_a2235_init(const struct gps_a2235_board *const p_board)
	{return ERROR_OK;}

#endif
