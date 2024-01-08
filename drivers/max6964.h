#pragma once

#include <nrf_spim.h>

#include "drivers/leds.h"
#include "drivers/twim.h"
#include "error.h"

#define EMAX6964_NO_INIT					(EMAX6964_BASE + 0x00)
#define EMAX6964_INVALID_GPO				(EMAX6964_BASE + 0x01)
#define EMAX6964_INVALID_INTENSITY			(EMAX6964_BASE + 0x02)
#define EMAX6964_INVALID_ARG				(EMAX6964_BASE + 0x03)
#define EMAX6964_MUTEX						(EMAX6964_BASE + 0x04)
#define EMAX6964_NO_LED						(EMAX6964_BASE + 0x05)
#define EMAX6964_LED_IDX					(EMAX6964_BASE + 0x06)
#define EMAX6964_QUEUE_FULL					(EMAX6964_BASE + 0x07)

#define MAX6964_TWI_SLAVE_ADDRESS_1			0x20
#define MAX6964_TWI_SLAVE_ADDRESS_2			0x24
#define MAX6964_TWI_SLAVE_ADDRESS_3			0x60
#define MAX6964_TWI_SLAVE_ADDRESS_4			0x64

enum max6964_gpo {
	MAX6964_GPO_0 = 0,
	MAX6964_GPO_1,
	MAX6964_GPO_2,
	MAX6964_GPO_3,
	MAX6964_GPO_4,
	MAX6964_GPO_5,
	MAX6964_GPO_6,
	MAX6964_GPO_7,
	MAX6964_GPO_8,
	MAX6964_GPO_9,
	MAX6964_GPO_10,
	MAX6964_GPO_11,
	MAX6964_GPO_12,
	MAX6964_GPO_13,
	MAX6964_GPO_14,
	MAX6964_GPO_15,
	MAX6964_GPO_16,
	NBR_OF_MAX6964_GPOS,
};

enum max6964_led_type {
	MAX6964_LED_MONO = 0,
	MAX6964_LED_RGB,
	NBR_OF_MAX6964_LED_TYPES,
};

struct max6964_led {
	enum max6964_led_type type;
	union {
		struct {
			enum max6964_gpo red;
			enum max6964_gpo green;
			enum max6964_gpo blue;
		};
		enum max6964_gpo mono_led;
	};
};

struct max6964_board {
	struct twim_client twim_client;
	struct {
		int8_t reset;
		int8_t blink;
	} pins;
	const struct max6964_led *p_leds;
	uint8_t num_leds;
};

#if (FEAT_HW_MAX6964 == 1)

err_code max6964_led_set_level(uint8_t led, uint8_t level);
err_code max6964_led_set_rgb_level(uint8_t led, struct leds_rgb_levels levels);
err_code max6964_led_set_visible(uint8_t led, bool visible);
err_code max6964_set_master_level(uint8_t level);
err_code max6964_deinit(void);
err_code max6964_init(const struct max6964_board * const p_board);

#else

__STATIC_INLINE
err_code max6964_led_set_level(uint8_t led, uint8_t level)
	{ return EMAX6964_NO_INIT; }
__STATIC_INLINE
err_code max6964_led_set_rgb_level(uint8_t led, struct leds_rgb_levels levels)
	{ return EMAX6964_NO_INIT; }
__STATIC_INLINE
err_code max6964_led_set_visible(const uint8_t led, bool visible)
	{ return EMAX6964_NO_INIT; }
__STATIC_INLINE
err_code max6964_set_master_level(uint8_t level)
	{ return EMAX6964_NO_INIT; }
__STATIC_INLINE
err_code max6964_deinit(void)
	{ return ERROR_OK; }
__STATIC_INLINE
err_code max6964_init(const struct max6964_board * const p_board)
	{ return ERROR_OK; }

#endif
