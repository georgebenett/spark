#include "drivers/leds.h"
#include "error.h"

#define DRV_NAME	LEDS
#define LOG_TAG		STRINGIFY(DRV_NAME)
#include "log.h"

static struct {
	const struct leds_board	*p_board;
} me;

err_code leds_set_level(enum leds_index led, const uint8_t level)
{
	if (!me.p_board)
		return ELEDS_NO_INIT;

	if (!me.p_board->fp_set_level)
		return ELEDS_NO_SET_LEVEL_FUNC;

	return me.p_board->fp_set_level(led, level);
}

err_code leds_set_rgb_level(const uint8_t led,
	const struct leds_rgb_levels levels)
{
	if (!me.p_board)
		return ELEDS_NO_INIT;

	if (!me.p_board->fp_set_rgb_level)
		return ELEDS_NO_SET_RGB_LEVEL_FUNC;

	return me.p_board->fp_set_rgb_level(led, levels);
}

err_code leds_set_pattern(enum leds_index led, const enum leds_pattern pattern)
{
	if (!me.p_board)
		return ELEDS_NO_INIT;

	if (!me.p_board->fp_set_pattern)
		return ELEDS_NO_SET_PATTERN_FUNC;

	return me.p_board->fp_set_pattern(led, pattern);
}

err_code leds_init(const struct leds_board * const p_board)
{
	if (p_board == NULL)
		return ELEDS_INVALID_ARG;

	me.p_board = p_board;

	return ERROR_OK;
}
