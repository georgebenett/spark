#pragma once

#include <nrf_spim.h>

#include "drivers/gpio_irq.h"
#include "drivers/twim.h"
#include "error.h"

#define EGPS_UBLOX_NO_INIT					(EGPS_UBLOX_BASE + 0x00)
#define EGPS_UBLOX_PDATA					(EGPS_UBLOX_BASE + 0x01)
#define EGPS_UBLOX_INVALID_ARG				(EGPS_UBLOX_BASE + 0x02)
#define EGPS_UBLOX_READ_BUF_OVERFLOW		(EGPS_UBLOX_BASE + 0x03)
#define EGPS_UBLOX_NO_RESPONSE				(EGPS_UBLOX_BASE + 0x04)
#define EGPS_UBLOX_UBX_NACK					(EGPS_UBLOX_BASE + 0x05)
#define EGPS_UBLOX_UBX_ERROR_RSP			(EGPS_UBLOX_BASE + 0x06)
#define EGPS_UBLOX_STREAM_SIZE				(EGPS_UBLOX_BASE + 0x07)
#define EGPS_UBLOX_QUEUE_FULL				(EGPS_UBLOX_BASE + 0x08)
#define EGPS_UBLOX_WRITE_CFG				(EGPS_UBLOX_BASE + 0x09)
#define EGPS_UBLOX_INVALID_INTERFACE		(EGPS_UBLOX_BASE + 0x0A)
#define EGPS_UBLOX_INVALID_VARIANT			(EGPS_UBLOX_BASE + 0x0B)
#define EGPS_UBLOX_SPI_TMO					(EGPS_UBLOX_BASE + 0x0C)
#define EGPS_UBLOX_WRITE_BUF_OVERFLOW		(EGPS_UBLOX_BASE + 0x0D)

#define GPS_UBLOX_TWI_SLAVE_ADDRESS			0x42

enum gps_ublox_if {
	GPS_UBLOX_IF_I2C = 0,
	GPS_UBLOX_IF_SPI,
	NUM_GPS_UBLOX_IFS,
};

enum gps_ublox_variant {
	GPS_UBLOX_VARIANT_8 = 0,
	GPS_UBLOX_VARIANT_M8,
	NUM_GPS_UBLOX_VARIANTS,
};

struct gps_ublox_spi_pins {
	uint8_t spi_cs;
	uint8_t spi_clk;
	uint8_t spi_mosi;
	uint8_t spi_miso;
};

struct gps_ublox_spi {
	struct gps_ublox_spi_pins		pins;
	nrf_spim_frequency_t			frequency;
	NRF_SPIM_Type					*p_instance;
};

struct gps_ublox_board {
	enum gps_ublox_if			interface;
	enum gps_ublox_variant		variant;
	struct twim_client			twim_client;
	struct gps_ublox_spi		spi;
	enum gpio_irq_polarity		irq_pol;
	uint8_t						gpio_irq;
	uint8_t						gpio_rst;
};

#if (FEAT_HW_GPS_UBLOX == 1)

err_code gps_ublox_init(const struct gps_ublox_board *const p_board);

#else

__STATIC_INLINE
err_code gps_ublox_init(const struct gps_ublox_board *const p_board)
	{ return ERROR_OK; }

#endif
