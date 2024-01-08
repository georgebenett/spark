#include <string.h>

#include "drivers/uarte.h"
#include "esp_nrf_prot.h"
#include "espwifi.h"
#include "espwifi_pkt.h"
#include "fast_crc32.h"
#include "util.h"

#define START_BYTE					(ESP_NRF_PROT_MAGIC & 0xFF)
#define SILENCE_TICKS				pdMS_TO_TICKS(20)

static struct {
	const NRF_UARTE_Type	*p_uarte;
	uint8_t					rx_buf[ESP_NRF_PROT_MAX_PACKET_LEN];
	uint16_t				rx_buf_len;
	uint16_t				last_rx_pkt_len;
	uint8_t					tx_pkt[ESP_NRF_PROT_MAX_PACKET_LEN];
} me;

static void discard_garbage(void)
{
	uint16_t i;

	for (i = 1; i < me.rx_buf_len; i++) {
		if (me.rx_buf[i] == START_BYTE) {
			me.rx_buf_len -= i;
			memmove(me.rx_buf, &me.rx_buf[i], me.rx_buf_len);
			return;
		}
	}

	me.rx_buf_len = 0;
}

static err_code ensure_bytes(const uint16_t num_bytes,
	const uint32_t start_ticks, const uint32_t timeout_ticks)
{
	uint32_t ticks_to_wait;
	uint32_t ticks_passed;
	uint16_t numread;
	err_code r;

	while (me.rx_buf_len < num_bytes) {
		ticks_passed = util_ticks_diff_now(start_ticks);
		if (ticks_passed >= timeout_ticks)
			return EESPWIFI_RECV_TIMEOUT;

		/* Avoid waking up when there is no data */
		ticks_to_wait = (timeout_ticks - ticks_passed);
		if (me.rx_buf_len > 0)
			ticks_to_wait = SILENCE_TICKS;

		r = uarte_read(me.p_uarte, &me.rx_buf[me.rx_buf_len],
			(sizeof(me.rx_buf) - me.rx_buf_len), &numread, ticks_to_wait);
		if (r != ERROR_OK)
			return r;

		if (numread == 0)
			return EESPWIFI_RECV_SILENCE;

		me.rx_buf_len += numread;
	}

	return ERROR_OK;
}

err_code espwifi_pkt_clear_recv_buffer(void)
{
	err_code r;

	r = uarte_rx_flush_buffer(me.p_uarte);
	ERR_CHECK(r);

	me.last_rx_pkt_len = 0;
	me.rx_buf_len = 0;
	return ERROR_OK;
}

err_code espwifi_pkt_recv(struct rbuf * const p_packet,
	const uint32_t start_ticks, const uint32_t timeout_ticks)
{
	struct esp_nrf_prot_header header;
	struct esp_nrf_prot_footer footer;
	uint32_t actual_crc;
	err_code r;

	if (p_packet == NULL)
		return EESPWIFI_INVALID_ARG;

	if (me.last_rx_pkt_len > 0) {
		me.rx_buf_len -= me.last_rx_pkt_len;
		memmove(me.rx_buf, &me.rx_buf[me.last_rx_pkt_len], me.rx_buf_len);
		me.last_rx_pkt_len = 0;
	}

	while (true) {
		r = ensure_bytes(sizeof(header), start_ticks, timeout_ticks);
		if (r == EESPWIFI_RECV_SILENCE) {
			discard_garbage();
			continue;
		}
		if (r != ERROR_OK)
			return r;

		memcpy(&header, me.rx_buf, sizeof(header));

		if (header.magic != ESP_NRF_PROT_MAGIC) {
			discard_garbage();
			continue;
		}

		if (header.payload_len > ESP_NRF_PROT_MAX_PAYLOAD_LEN) {
			discard_garbage();
			continue;
		}

		r = ensure_bytes((sizeof(header) + sizeof(footer) + header.payload_len),
			start_ticks, timeout_ticks);
		if (r == EESPWIFI_RECV_SILENCE) {
			discard_garbage();
			continue;
		}
		if (r != ERROR_OK)
			return r;

		memcpy(&footer, &me.rx_buf[sizeof(header) + header.payload_len],
			sizeof(footer));

		actual_crc = fast_crc32(me.rx_buf,
			(sizeof(header) + header.payload_len), 0);

		if (footer.crc != actual_crc) {
			discard_garbage();
			continue;
		}

		p_packet->len = header.payload_len;
		p_packet->p_data = &me.rx_buf[sizeof(header)];
		me.last_rx_pkt_len = (sizeof(header) + sizeof(footer) +
			header.payload_len);
		return ERROR_OK;
	}
}

err_code espwifi_pkt_send_prepare(struct wbuf * const p_payload)
{
	if (p_payload == NULL)
		return EESPWIFI_INVALID_ARG;

	p_payload->p_data = &me.tx_pkt[sizeof(struct esp_nrf_prot_header)];
	p_payload->cap = ESP_NRF_PROT_MAX_PAYLOAD_LEN;
	return ERROR_OK;
}

err_code espwifi_pkt_send_submit(const struct wbuf * const p_payload)
{
	struct esp_nrf_prot_header header;
	struct esp_nrf_prot_footer footer;
	uint16_t footer_ofs;

	if ((p_payload == NULL) ||
		(p_payload->cap > ESP_NRF_PROT_MAX_PAYLOAD_LEN))
		return EESPWIFI_INVALID_ARG;

	header.magic = ESP_NRF_PROT_MAGIC;
	header.payload_len = (ESP_NRF_PROT_MAX_PAYLOAD_LEN - p_payload->cap);
	memcpy(me.tx_pkt, &header, sizeof(header));

	footer_ofs = (sizeof(header) + header.payload_len);
	footer.crc = fast_crc32(me.tx_pkt, footer_ofs, 0);
	memcpy(&me.tx_pkt[footer_ofs], &footer, sizeof(footer));

	return uarte_write(me.p_uarte, me.tx_pkt, (footer_ofs + sizeof(footer)));
}

err_code espwifi_pkt_set_enabled(const bool enabled)
{
	err_code r;

	if (enabled) {
		r = uarte_tx_enable(me.p_uarte);
		ERR_CHECK(r);
		r = uarte_rx_enable(me.p_uarte);
		if (r == EUARTE_RX_BUSY)
			r = ERROR_OK;
	} else {
		r = uarte_tx_disable(me.p_uarte);
		ERR_CHECK(r);
		r = uarte_rx_disable(me.p_uarte);
	}

	return r;
}

err_code espwifi_pkt_init(const NRF_UARTE_Type * const p_uarte)
{
	err_code r;

	if (me.p_uarte != NULL)
		return ERROR_OK;

	r = uarte_rx_enable(p_uarte);
	ERR_CHECK(r);

	r = uarte_tx_enable(p_uarte);
	ERR_CHECK(r);

	me.p_uarte = p_uarte;
	return ERROR_OK;
}
