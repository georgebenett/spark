/* Internal driver file; should not be included by other users. */
#pragma once

#include <stdint.h>

#include <nrf.h>

#include "error.h"
#include "espwifi_buf.h"

err_code espwifi_pkt_clear_recv_buffer(void);
err_code espwifi_pkt_recv(struct rbuf * const p_packet,
	const uint32_t start_ticks, const uint32_t timeout_ticks);
err_code espwifi_pkt_send_prepare(struct wbuf * const p_payload);
err_code espwifi_pkt_send_submit(const struct wbuf * const p_payload);
err_code espwifi_pkt_set_enabled(const bool enabled);
err_code espwifi_pkt_init(const NRF_UARTE_Type * const p_uarte);
