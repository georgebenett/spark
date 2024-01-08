#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "error.h"

#define ECAN_NO_INIT			(ECAN_BASE + 0x00)
#define ECAN_INVALID_ARG		(ECAN_BASE + 0x01)
#define ECAN_NO_SEND_FUNC		(ECAN_BASE + 0x02)

#define CAN_CLIENT_MAX_NBR_IDS	25
#define CAN_FRAME_MAX_SIZE		8

struct can_id {
	uint32_t	sid:11;
	uint32_t	eid:19;
	bool		use_eid;
};

typedef void (*can_rx_cb)(const struct can_id can_id,
	const uint8_t * const p_buf, const uint8_t len);

struct can_rx_client {
	can_rx_cb		cb;
	uint8_t			nbr_ids;
	struct can_id	can_id[CAN_CLIENT_MAX_NBR_IDS];
};

typedef err_code (*can_reg_rx_clnt_fnc)(uint32_t * const p_handle,
	const struct can_rx_client * const p_clnt);

typedef err_code (*can_unreg_rx_clnt_fnc)(const uint32_t handle);

typedef err_code (*can_send_frame_fnc)(const struct can_id can_id,
	const uint8_t * const p_buf, const uint8_t size);

struct can_board {
	can_reg_rx_clnt_fnc		fp_reg_rx_clnt;
	can_unreg_rx_clnt_fnc	fp_unreg_rx_clnt;
	can_send_frame_fnc		fp_send_frame;
};

#if (FEAT_HW_CAN == 1)

err_code can_send_can_frame(const struct can_id can_id,
	const uint8_t * const p_buf, const uint8_t size);
err_code can_register_rx_client(uint32_t * const p_handle,
	const struct can_rx_client * const p_clnt);
err_code can_unregister_rx_client(const uint32_t handle);
err_code can_init(const struct can_board * const p_board);

#else

__STATIC_INLINE
err_code can_init(const struct can_board * const p_board)
	{ return ERROR_OK; }
__STATIC_INLINE
err_code can_send_can_frame(const struct can_id can_id,
	const uint8_t * const p_buf, const uint8_t size)
	{ return ECAN_NO_INIT; }
__STATIC_INLINE
err_code can_register_rx_client(uint32_t * const p_handle,
	const struct can_rx_client * const p_clnt)
	{ return ECAN_NO_INIT; }
__STATIC_INLINE
err_code can_unregister_rx_client(const uint32_t handle)
	{ return ECAN_NO_INIT; }

#endif
