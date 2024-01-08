#pragma once

#include "error.h"

#define EWDT_NO_INIT		(EWDT_BASE + 0x00)
#define EWDT_RELOAD_VAL		(EWDT_BASE + 0x01)
#define EWDT_CHANNEL_ALLOC	(EWDT_BASE + 0x02)

err_code watchdog_feed(void);
err_code watchdog_init(void);
