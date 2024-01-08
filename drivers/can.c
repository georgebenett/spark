#include <app_util_platform.h>

#include "boards.h"
#include "drivers/can.h"
#include "error.h"

static struct {
	const struct can_board	*p_board;
} me;

err_code can_send_can_frame(const struct can_id can_id,
	const uint8_t * const p_buf, const uint8_t size)
{
	if (!me.p_board)
		return ECAN_NO_INIT;

	if (!me.p_board->fp_send_frame)
		return ECAN_NO_SEND_FUNC;

	if (!p_buf || !size)
		return ECAN_INVALID_ARG;

	return me.p_board->fp_send_frame(can_id, p_buf, size);
}

err_code can_register_rx_client(uint32_t * const p_handle,
	const struct can_rx_client * const p_clnt)
{
	if (!me.p_board)
		return ECAN_NO_INIT;

	if (!p_handle || !p_clnt)
		return ECAN_INVALID_ARG;

	return me.p_board->fp_reg_rx_clnt(p_handle, p_clnt);
}

err_code can_unregister_rx_client(const uint32_t handle)
{
	if (!me.p_board)
		return ECAN_NO_INIT;

	return me.p_board->fp_unreg_rx_clnt(handle);
}

err_code can_init(const struct can_board * const p_board)
{
	if (!p_board || !p_board->fp_reg_rx_clnt || !p_board->fp_unreg_rx_clnt)
		return ECAN_INVALID_ARG;

	me.p_board = p_board;

	return ERROR_OK;
}
