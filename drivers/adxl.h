#pragma once

#include <nrf.h>
#include <nrf_spim.h>

#include "error.h"

#define EADXL_BOARD			(EADXL_BASE + 0x00)
#define EADXL_NO_INIT		(EADXL_BASE + 0x01)
#define EADXL_INVALID_ARG	(EADXL_BASE + 0x02)
#define EADXL_SPI_INIT		(EADXL_BASE + 0x03)
#define EADXL_SPI_TRANSFER	(EADXL_BASE + 0x04)
#define EADXL_SPI_DISABLED	(EADXL_BASE + 0x05)
#define EADXL_WRONG_ID		(EADXL_BASE + 0x06)
#define EADXL_NOT_ENABLED	(EADXL_BASE + 0x07)
#define EADXL_NOT_POWERED	(EADXL_BASE + 0x08)
#define EADXL_EMPTY_FIFO	(EADXL_BASE + 0x09)
#define EADXL_MUTEX			(EADXL_BASE + 0x0A)
#define EADXL_TIMEOUT		(EADXL_BASE + 0x0B)

/* Output Data Rate (ODR), as defined for the ADXL362 FILTER_CTL register */
enum adxl_odr {
	ADXL_ODR_12_5HZ =  0,
	ADXL_ODR_25HZ,
	ADXL_ODR_50HZ,
	ADXL_ODR_100HZ,
	ADXL_ODR_200HZ,
	ADXL_ODR_400HZ,
	NBR_OF_ADXL_ODRS
};

enum adxl_mode {
	ADXL_MODE_OFF = 0,
	ADXL_MODE_STREAM,
	ADXL_MODE_STREAM_MOTION_DETECT,
	ADXL_MODE_MOTION_DETECT,
	ADXL_MODE_PRODTEST,
	NBR_OF_ADXL_MODES
};

enum adxl_event {
	ADXL_EVENT_DATA = 0,
	ADXL_EVENT_MOVEMENT,
	ADXL_EVENT_STILLNESS,
	NBR_OF_ADXL_EVENTS
};

struct adxl_chip_id {
	uint8_t dev;
	uint8_t part;
	uint8_t rev;
};

struct adxl_pins {
	int8_t vdd;
	uint8_t int1;
	uint8_t spi_cs;
	uint8_t spi_clk;
	uint8_t spi_mosi;
	uint8_t spi_miso;
} __packed;

struct adxl_board {
	struct adxl_pins pins;
	nrf_spim_frequency_t frequency;
	NRF_SPIM_Type *p_spi;
	enum adxl_odr rate;
	bool wakeup_enabled;
};

#if (FEAT_HW_ADXL == 1)

err_code adxl_get_temperature(int16_t *temperature);
err_code adxl_get_samples(int16_t **p_samples, int *p_num_samples);
err_code adxl_get_sample(int16_t *x, int16_t *y, int16_t *z);
err_code adxl_get_id(struct adxl_chip_id *p_id);
err_code adxl_get_is_initialized(bool *p_initialized);
err_code adxl_enable(enum adxl_mode mode);
err_code adxl_disable(void);
err_code adxl_deinit(void);
err_code adxl_init(const struct adxl_board *p_board);

#else

__STATIC_INLINE
err_code adxl_get_temperature(int16_t *temperature)
	{ return EADXL_NO_INIT; }
__STATIC_INLINE
err_code adxl_get_samples(int16_t **p_samples, int *p_num_samples)
	{ return EADXL_NO_INIT; }
__STATIC_INLINE
err_code adxl_get_sample(int16_t *p_x, int16_t *p_y, int16_t *p_z)
	{ return EADXL_NO_INIT; }
__STATIC_INLINE
err_code adxl_get_id(struct adxl_chip_id *p_id)
	{ return EADXL_NO_INIT; }
__STATIC_INLINE
err_code adxl_get_is_initialized(bool *p_initialized)
	{ return EADXL_NO_INIT; }
__STATIC_INLINE
err_code adxl_enable(enum adxl_mode mode)
	{ return EADXL_NO_INIT; }
__STATIC_INLINE
err_code adxl_disable(void)
	{ return EADXL_NO_INIT; }
__STATIC_INLINE
err_code adxl_deinit(void)
	{ return ERROR_OK; }
__STATIC_INLINE
err_code adxl_init(const struct adxl_board *p_board)
	{ return ERROR_OK; }

#endif
