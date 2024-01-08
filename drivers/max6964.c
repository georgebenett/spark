#include <string.h>

#include <FreeRTOS.h>
#include <nrf_gpio.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>

#include "boards.h"
#include "drivers/leds.h"
#include "drivers/max6964.h"
#include "drivers/twim_spim_irq.h"
#include "freertos_static.h"
#include "util.h"

#define DRV_NAME					MAX6964
#define LOG_TAG						STRINGIFY(DRV_NAME)
#include "log.h"

#define MUTEX_NAME					DRV_NAME
#define QUEUE_LEN					16
#define QUEUE_NAME					DRV_NAME
#define TASK_NAME					DRV_NAME
#define TASK_PRIORITY				2
#define TASK_STACK_DEPTH			192

#define ADDR_OFFSET					0
#define PAYLOAD_OFFSET				1
#define CFG_REQ_LEN					2
#define GLOBAL_INTENS_REQ_LEN		2
#define BLK_PHASE_REQ_LEN			3
#define GPO_INTENS_REQ_LEN			2
#define ADDR_LEN					1
#define NBR_GPO_PER_REG				2

#define BLK_PHASE_ALL_ACTIVE_LOW	0xFFFF
#define BLK_PHASE_ALL_ACTIVE_HIGH	0x0000

#define LED_BLK_PHASE				BLK_PHASE_ALL_ACTIVE_LOW
#define LED_MASTER_INTENS_OFF		0x0

#define MAX_INTENSITY				0x0F
#define GPO_INTENSITY_OFF			MAX_INTENSITY
#define LEVEL2INTENS(x)				((x) >> 4)

STATIC_MUTEX(DRV_NAME);

enum command {
	COMMAND_SET_MASTER_INTENS = 0,
	COMMAND_SET_INTENS,
	COMMAND_SET_VISIBLE,
};

struct queue_item {
	enum command command;
	uint8_t led;

	union {
		uint8_t intens;
		struct leds_rgb_levels rgb_intens;
		bool visible;
	};
};

STATIC_TASK(TASK_NAME, TASK_STACK_DEPTH);
STATIC_QUEUE(QUEUE_NAME, QUEUE_LEN, struct queue_item);
STATIC_MUTEX(TASK_NAME);

enum reg_addr {
	REG_ADDR_BLK_PHASE_0_LO = 0x02,
	REG_ADDR_BLK_PHASE_0_HI = 0x03,
	REG_ADDR_USR_RAM_0 = 0x06,
	REG_ADDR_USR_RAM_1 = 0x07,
	REG_ADDR_BLK_PHASE_1_LO = 0x0A,
	REG_ADDR_BLK_PHASE_1_HI = 0x0B,
	REG_ADDR_OUT_INTENS_MST_16 = 0x0E,
	REG_ADDR_CFG = 0x0F,
	REG_ADDR_OUT_INTENS_1_0 = 0x10,
	REG_ADDR_OUT_INTENS_3_2 = 0x11,
	REG_ADDR_OUT_INTENS_5_4 = 0x12,
	REG_ADDR_OUT_INTENS_7_6 = 0x13,
	REG_ADDR_OUT_INTENS_9_8 = 0x14,
	REG_ADDR_OUT_INTENS_11_10 = 0x15,
	REG_ADDR_OUT_INTENS_13_12 = 0x16,
	REG_ADDR_OUT_INTENS_15_14 = 0x17,
};

struct cfg_reg {
	uint8_t blk_en : 1;
	uint8_t blk_flip : 1;
	uint8_t global_intens_en : 1;
	uint8_t unused_2 : 1;
	uint8_t gpo_16 : 2;
	uint8_t blk_stat : 1;
	uint8_t unused_1 : 1;
};

struct gpo_intens_reg {
	uint8_t lower_gpo : 4;
	uint8_t upper_gpo : 4;
};

static struct {
	const struct max6964_board	*p_board;
	uint8_t						regulator_5v_handle;

	SemaphoreHandle_t			mutex_handle;
	QueueHandle_t				queue_handle;
	TaskHandle_t				task_handle;

	uint8_t						intens[NBR_OF_MAX6964_GPOS];
	bool						visible[NBR_OF_MAX6964_GPOS];
} me;

static err_code configure(const struct twim_client * const p_client,
	bool global_intens_en, bool blk_flip, bool blk_en)
{
	struct cfg_reg *p_cfg_reg;
	uint8_t buf[CFG_REQ_LEN];
	err_code r;

	buf[ADDR_OFFSET] = REG_ADDR_CFG;

	r = twim_client_indexed_read(p_client, &buf[ADDR_OFFSET], ADDR_LEN,
		&buf[PAYLOAD_OFFSET], (sizeof(buf) - ADDR_LEN));
	ERR_CHECK(r);

	p_cfg_reg = (struct cfg_reg *)&buf[PAYLOAD_OFFSET];
	p_cfg_reg->global_intens_en = global_intens_en;
	p_cfg_reg->blk_flip = blk_flip;
	p_cfg_reg->blk_en = blk_en;

	return twim_client_write(p_client, &buf[0], sizeof(buf));
}

static err_code set_blk_phase_output(const struct twim_client * const p_client,
	uint16_t blk_phase_bits, int8_t blk_phase)
{
	uint8_t buf[BLK_PHASE_REQ_LEN];

	if (blk_phase)
		buf[ADDR_OFFSET] = REG_ADDR_BLK_PHASE_1_LO;
	else
		buf[ADDR_OFFSET] = REG_ADDR_BLK_PHASE_0_LO;

	memcpy(&buf[PAYLOAD_OFFSET], &blk_phase_bits, sizeof(blk_phase_bits));

	return twim_client_write(p_client, &buf[0], sizeof(buf));
}

static err_code set_gpo_intens(const struct twim_client * const p_client,
	enum max6964_gpo gpo, uint8_t intens)
{
	struct gpo_intens_reg *p_reg;
	uint8_t buf[GPO_INTENS_REQ_LEN];
	err_code r;

	if (gpo >= NBR_OF_MAX6964_GPOS)
		return EMAX6964_INVALID_GPO;

	if (intens > MAX_INTENSITY)
		return EMAX6964_INVALID_INTENSITY;

	if (gpo == MAX6964_GPO_16)
		buf[ADDR_OFFSET] = REG_ADDR_OUT_INTENS_MST_16;
	else
		buf[ADDR_OFFSET] = (REG_ADDR_OUT_INTENS_1_0 + (gpo / NBR_GPO_PER_REG));

	r = twim_client_indexed_read(p_client, &buf[ADDR_OFFSET], ADDR_LEN,
		&buf[PAYLOAD_OFFSET], (sizeof(buf) - ADDR_LEN));
	ERR_CHECK(r);

	p_reg = (struct gpo_intens_reg *)&buf[PAYLOAD_OFFSET];
	if (gpo % NBR_GPO_PER_REG)
		p_reg->upper_gpo = intens;
	else
		p_reg->lower_gpo = intens;

	return twim_client_write(p_client, &buf[0], sizeof(buf));
}

static err_code set_master_intens(const struct twim_client * const p_client,
	uint8_t intens)
{
	struct gpo_intens_reg *p_reg;
	uint8_t buf[GLOBAL_INTENS_REQ_LEN];
	err_code r;

	if (intens > MAX_INTENSITY)
		return EMAX6964_INVALID_INTENSITY;

	buf[ADDR_OFFSET] = REG_ADDR_OUT_INTENS_MST_16;

	r = twim_client_indexed_read(p_client, &buf[ADDR_OFFSET], ADDR_LEN,
		&buf[PAYLOAD_OFFSET], (sizeof(buf) - ADDR_LEN));
	ERR_CHECK(r);

	p_reg = (struct gpo_intens_reg *)&buf[PAYLOAD_OFFSET];
	p_reg->upper_gpo = intens;

	return twim_client_write(p_client, &buf[0], sizeof(buf));
}

static err_code update_led_state(uint8_t led)
{
	const struct max6964_led *p_led;
	err_code r;

	p_led = &me.p_board->p_leds[led];

	switch (p_led->type) {
	case MAX6964_LED_MONO:
		r = set_gpo_intens(&me.p_board->twim_client, p_led->mono_led,
			me.visible[p_led->mono_led]
				? me.intens[p_led->mono_led]
				: GPO_INTENSITY_OFF);
		ERR_CHECK(r);
		break;
	case MAX6964_LED_RGB:
		r = set_gpo_intens(&me.p_board->twim_client, p_led->red,
			me.visible[p_led->red]
				? me.intens[p_led->red]
				: GPO_INTENSITY_OFF);
		ERR_CHECK(r);
		r = set_gpo_intens(&me.p_board->twim_client, p_led->green,
			me.visible[p_led->green]
				? me.intens[p_led->green]
				: GPO_INTENSITY_OFF);
		ERR_CHECK(r);
		r = set_gpo_intens(&me.p_board->twim_client, p_led->blue,
			me.visible[p_led->blue]
				? me.intens[p_led->blue]
				: GPO_INTENSITY_OFF);
		ERR_CHECK(r);
		break;
	default:
		/* should never happen */
		break;
	}

	return ERROR_OK;
}

static void process_set_intens_command(struct queue_item *p_item)
{
	const struct max6964_led *p_led;

	p_led = &me.p_board->p_leds[p_item->led];

	switch (p_led->type) {
	case MAX6964_LED_MONO:
		me.intens[p_led->mono_led] = p_item->intens;
		break;
	case MAX6964_LED_RGB:
		me.intens[p_led->red] = p_item->rgb_intens.red;
		me.intens[p_led->green] = p_item->rgb_intens.green;
		me.intens[p_led->blue] = p_item->rgb_intens.blue;
		break;
	default:
		/* should never happen */
		break;
	}
}

static void process_set_visible_command(struct queue_item *p_item)
{
	const struct max6964_led *p_led;

	p_led = &me.p_board->p_leds[p_item->led];

	switch (p_led->type) {
	case MAX6964_LED_MONO:
		me.visible[p_led->mono_led] = p_item->visible;
		break;
	case MAX6964_LED_RGB:
		me.visible[p_led->red] = p_item->visible;
		me.visible[p_led->green] = p_item->visible;
		me.visible[p_led->blue] = p_item->visible;
		break;
	default:
		/* should never happen */
		break;
	}
}

static err_code process_command(struct queue_item *p_item)
{
	err_code r;

	if (!util_mutex_lock(me.mutex_handle))
		return EMAX6964_MUTEX;

	switch (p_item->command) {
	case COMMAND_SET_MASTER_INTENS:
		r = set_master_intens(&me.p_board->twim_client, p_item->intens);
		ERR_CHECK_GOTO(r, exit);
		break;
	case COMMAND_SET_INTENS:
		process_set_intens_command(p_item);
		break;
	case COMMAND_SET_VISIBLE:
		process_set_visible_command(p_item);
		break;
	default:
		/* should never happen */
		break;
	}

	r = update_led_state(p_item->led);

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

static void task(void *arg)
{
	struct queue_item item;
	err_code r;

	(void)arg;

	while (1) {
		if (xQueueReceive(me.queue_handle, &item, portMAX_DELAY) != pdTRUE) {
			/* should never happen */
			continue;
		}

		r = process_command(&item);
		if (r != ERROR_OK)
			LOGE("process_command r=%08lX", r);
	}
}

err_code max6964_led_set_level(uint8_t led, uint8_t level)
{
	struct queue_item item;

	if (!me.p_board)
		return EMAX6964_NO_INIT;

	if (!me.p_board->p_leds || !me.p_board->num_leds)
		return EMAX6964_NO_LED;

	if (led >= me.p_board->num_leds)
		return EMAX6964_INVALID_ARG;

	if (me.p_board->p_leds[led].type != MAX6964_LED_MONO)
		return EMAX6964_LED_IDX;

	/* Intensity level 0x0 is the highest individual intensity and
	 * intensity level 0xf is the lowest individual intensity.
	 */
	item.command = COMMAND_SET_INTENS;
	item.led = led;
	item.intens = (MAX_INTENSITY - LEVEL2INTENS(level));

	if (xQueueSendToBack(me.queue_handle, &item,
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
		return EMAX6964_QUEUE_FULL;

	return ERROR_OK;
}

err_code max6964_led_set_rgb_level(uint8_t led, struct leds_rgb_levels levels)
{
	struct queue_item item;

	if (!me.p_board)
		return EMAX6964_NO_INIT;

	if (!me.p_board->p_leds || !me.p_board->num_leds)
		return EMAX6964_NO_LED;

	if (led >= me.p_board->num_leds)
		return EMAX6964_INVALID_ARG;

	if (me.p_board->p_leds[led].type != MAX6964_LED_RGB)
		return EMAX6964_LED_IDX;

	/* Intensity level 0x0 is the highest individual intensity and
	 * intensity level 0xf is the lowest individual intensity.
	 */
	item.command = COMMAND_SET_INTENS;
	item.led = led;
	item.rgb_intens.red = (MAX_INTENSITY - LEVEL2INTENS(levels.red));
	item.rgb_intens.green = (MAX_INTENSITY - LEVEL2INTENS(levels.green));
	item.rgb_intens.blue = (MAX_INTENSITY - LEVEL2INTENS(levels.blue));

	if (xQueueSendToBack(me.queue_handle, &item,
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
		return EMAX6964_QUEUE_FULL;

	return ERROR_OK;
}

err_code max6964_led_set_visible(const uint8_t led, bool visible)
{
	struct queue_item item;

	if (!me.p_board)
		return EMAX6964_NO_INIT;

	if (!me.p_board->p_leds || !me.p_board->num_leds)
		return EMAX6964_NO_LED;

	if (led >= me.p_board->num_leds)
		return EMAX6964_INVALID_ARG;

	item.command = COMMAND_SET_VISIBLE;
	item.led = led;
	item.visible = visible;

	if (xQueueSendToBack(me.queue_handle, &item,
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
		return EMAX6964_QUEUE_FULL;

	return ERROR_OK;
}

err_code max6964_set_master_level(uint8_t level)
{
	struct queue_item item;

	if (!me.p_board)
		return EMAX6964_NO_INIT;

	/* When 'blink phase' is set to active low, then:
	 * Master intensity level 0x1 is the highest master intensity.
	 * Master intensity level 0xf is the lowest master intensity.
	 * Master intensity level 0x0 turns off all leds.
	 */
	item.command = COMMAND_SET_MASTER_INTENS;
	item.intens =
		(LEVEL2INTENS(level) == 0x0) ? 0x0 : (0x10 - LEVEL2INTENS(level));

	if (xQueueSendToBack(me.queue_handle, &item,
			pdMS_TO_TICKS(RTOS_FUNC_WAIT_MS)) != pdPASS)
		return EMAX6964_QUEUE_FULL;

	return ERROR_OK;
}

err_code max6964_deinit(void)
{
	err_code r;

	if (!me.p_board)
		return ERROR_OK;

	if (!util_mutex_lock(me.mutex_handle))
		return EMAX6964_MUTEX;

	r = twim_client_unreg(&me.p_board->twim_client);
	ERR_CHECK_GOTO(r, exit);

	nrf_gpio_pin_clear(me.p_board->pins.reset);

	me.p_board = NULL;

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code max6964_init(const struct max6964_board * const p_board)
{
	err_code r;
	int i;

	if (me.p_board)
		return ERROR_OK;

	if (!p_board)
		return EMAX6964_INVALID_ARG;

	r = util_validate_pins((uint8_t *)&p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	nrf_gpio_pin_set(p_board->pins.reset);
	nrf_gpio_cfg_output(p_board->pins.reset);

	if (p_board->pins.blink != BOARD_UNUSED_PIN) {
		nrf_gpio_pin_clear(p_board->pins.blink);
		nrf_gpio_cfg_output(p_board->pins.blink);
	}

	r = twim_client_reg(&p_board->twim_client);
	ERR_CHECK(r);

	r = configure(&p_board->twim_client, false, false, false);
	ERR_CHECK(r);

	r = set_blk_phase_output(&p_board->twim_client, LED_BLK_PHASE, 0);
	ERR_CHECK(r);

	r = set_master_intens(&p_board->twim_client, LED_MASTER_INTENS_OFF);
	ERR_CHECK(r);

	for (i = 0; i < NBR_OF_MAX6964_GPOS; i++) {
		me.visible[i] = true;
		me.intens[i] = GPO_INTENSITY_OFF;
	}

	me.mutex_handle = CREATE_STATIC_MUTEX(DRV_NAME);
	me.queue_handle = CREATE_STATIC_QUEUE(TASK_NAME);
	me.task_handle = CREATE_STATIC_TASK(task, TASK_NAME, TASK_PRIORITY);

	me.p_board = p_board;

	return ERROR_OK;
}
