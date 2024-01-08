#include <string.h>

#include "esp_nrf_prot.h"
#include "espwifi.h"
#include "espwifi_cmd.h"
#include "espwifi_pkt.h"
#include "util.h"

#define MSGPACK_BIN16_START_BYTES		3

static err_code add_http_headers(struct wbuf *p_wbuf,
	const struct wifi_http_header * const p_headers,
	const uint8_t num_headers)
{
	err_code r;
	uint8_t i;

	r = wbuf_add_msgpack_map_start(p_wbuf, num_headers);
	ERR_CHECK(r);

	for (i = 0; i < num_headers; i++) {
		r = wbuf_add_msgpack_str(p_wbuf, p_headers[i].p_key,
			strlen(p_headers[i].p_key));
		ERR_CHECK(r);
		r = wbuf_add_msgpack_str(p_wbuf, p_headers[i].p_value,
			strlen(p_headers[i].p_value));
		ERR_CHECK(r);
	}
	return ERROR_OK;
}

static bool decode_params(const uint32_t sent_req_cmd, const uint32_t cmd,
	const uint32_t num_args, struct rbuf * const p_msgpack,
	struct cmd_info * const p_cmd_info)
{
	uint32_t recv_req_cmd;
	uint32_t len;
	err_code r;

	switch (cmd) {
	case ESP_NRF_PROT_SCC_REPLY_UNKNOWN_REQUEST:
		if (num_args < 1)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack, &recv_req_cmd);
		if (r != ERROR_OK)
			return false;
		if (recv_req_cmd != sent_req_cmd)
			return false;
		return true;

	case ESP_NRF_PROT_SCC_REPLY_REQUEST_OK:
		if (num_args < 1)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack, &recv_req_cmd);
		if (r != ERROR_OK)
			return false;
		if (recv_req_cmd != sent_req_cmd)
			return false;
		p_cmd_info->request_ok_reply.num_args = (num_args - 1);
		p_cmd_info->request_ok_reply.result = *p_msgpack;
		return true;

	case ESP_NRF_PROT_SCC_REPLY_REQUEST_FAILED:
		if (num_args < 2)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack, &recv_req_cmd);
		if (r != ERROR_OK)
			return false;
		if (recv_req_cmd != sent_req_cmd)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack,
			&p_cmd_info->request_failed_reply.error_code);
		return (r == ERROR_OK);

	case ESP_NRF_PROT_SCC_NOTIF_FOTA_STATUS:
		if (num_args < 2)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack,
			&p_cmd_info->fota_status_notif.received_bytes);
		if (r != ERROR_OK)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack,
			&p_cmd_info->fota_status_notif.total_bytes);
		return (r == ERROR_OK);

	case ESP_NRF_PROT_SCC_REQUEST_UPLOAD_DATA:
		if (num_args < 1)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack,
			&p_cmd_info->upload_data_request.offset);
		return (r == ERROR_OK);

	case ESP_NRF_PROT_SCC_REQUEST_DOWNLOAD_METADATA:
		if (num_args < 1)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack,
			&p_cmd_info->download_metadata_request.content_length);
		return (r == ERROR_OK);

	case ESP_NRF_PROT_SCC_REQUEST_DOWNLOAD_DATA:
		if (num_args < 2)
			return false;
		r = rbuf_get_msgpack_uint32(p_msgpack,
			&p_cmd_info->download_data_request.offset);
		if (r != ERROR_OK)
			return false;
		r = rbuf_get_msgpack_bin_start(p_msgpack, &len);
		if (r != ERROR_OK)
			return false;
		if (len > p_msgpack->len)
			return false;
		p_cmd_info->download_data_request.data.p_data = p_msgpack->p_data;
		p_cmd_info->download_data_request.data.len = len;
		return true;

	default:
		return true;
	}
}

err_code espwifi_cmd_recv(const uint32_t req_cmd,
	struct cmd_info * const p_cmd_info, const uint32_t start_ticks,
	const uint32_t timeout_ticks)
{
	struct rbuf packet;
	uint32_t num_args;
	uint32_t cmd;
	err_code r;

	while (true) {
		r = espwifi_pkt_recv(&packet, start_ticks, timeout_ticks);
		if (r != ERROR_OK)
			return r;

		r = rbuf_get_msgpack_array_start(&packet, &num_args);
		if (r != ERROR_OK)
			continue;

		if (num_args < 1)
			continue;

		r = rbuf_get_msgpack_uint32(&packet, &cmd);
		if (r != ERROR_OK)
			continue;

		num_args--;

		memset(p_cmd_info, 0, sizeof(*p_cmd_info));
		p_cmd_info->cmd = cmd;

		if (!decode_params(req_cmd, cmd, num_args, &packet, p_cmd_info))
			continue;

		return ERROR_OK;
	}
}

err_code espwifi_cmd_send_simple(const uint32_t cmd)
{
	struct wbuf wbuf;
	err_code r;

	r = espwifi_pkt_send_prepare(&wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&wbuf, 1);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&wbuf, cmd);
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&wbuf);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_send_connect_wifi(
	const struct wifi_ap_config * const p_ap_config)
{
	struct wbuf tx_wbuf;
	err_code r;

	r = espwifi_pkt_send_prepare(&tx_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&tx_wbuf, 3);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf,
		ESP_NRF_PROT_CSC_REQUEST_CONNECT_WIFI);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_bin(&tx_wbuf, p_ap_config->ssid.data,
		p_ap_config->ssid.len);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_bin(&tx_wbuf, p_ap_config->pass.data,
		p_ap_config->pass.len);
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&tx_wbuf);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_send_fota(const char * const p_url,
	const uint32_t url_len, const struct wifi_http_header * const p_headers,
	const uint8_t num_headers)
{
	struct wbuf tx_wbuf;
	err_code r;

	r = espwifi_pkt_send_prepare(&tx_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&tx_wbuf, 3);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf, ESP_NRF_PROT_CSC_REQUEST_FOTA_URL);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_bin(&tx_wbuf, (const uint8_t *)p_url, url_len);
	ERR_CHECK(r);
	r = add_http_headers(&tx_wbuf, p_headers, num_headers);
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&tx_wbuf);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_send_http_get(
	const char * const p_url, const uint32_t url_len,
	const struct rbuf * const p_accept,
	const struct wifi_http_header * const p_headers,
	const uint8_t num_headers)
{
	struct wbuf tx_wbuf;
	err_code r;

	r = espwifi_pkt_send_prepare(&tx_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&tx_wbuf, 4);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf, ESP_NRF_PROT_CSC_REQUEST_HTTP_GET);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_bin(&tx_wbuf, (const uint8_t *)p_url, url_len);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_str(&tx_wbuf, (const char *)p_accept->p_data,
		p_accept->len);
	ERR_CHECK(r);
	r = add_http_headers(&tx_wbuf, p_headers, num_headers);
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&tx_wbuf);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_send_http_post(
	const char * const p_url, const uint32_t url_len,
	const struct rbuf * const p_content_type,
	const uint32_t content_length,
	const struct wifi_http_header * const p_headers,
	const uint8_t num_headers)
{
	struct wbuf tx_wbuf;
	err_code r;

	r = espwifi_pkt_send_prepare(&tx_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&tx_wbuf, 5);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf, ESP_NRF_PROT_CSC_REQUEST_HTTP_POST);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_bin(&tx_wbuf, (const uint8_t *)p_url, url_len);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_str(&tx_wbuf, (const char *)p_content_type->p_data,
		p_content_type->len);
		ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf, content_length);
	ERR_CHECK(r);
	r = add_http_headers(&tx_wbuf, p_headers, num_headers);
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&tx_wbuf);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_send_cancel_reply(void)
{
	return espwifi_cmd_send_simple(ESP_NRF_PROT_CSC_REPLY_CANCEL);
}

err_code espwifi_cmd_send_download_metadata_reply(void)
{
	return espwifi_cmd_send_simple(ESP_NRF_PROT_CSC_REPLY_DOWNLOAD_METADATA);
}

err_code espwifi_cmd_send_download_data_reply(const uint32_t offset)
{
	struct wbuf tx_wbuf;
	err_code r;

	r = espwifi_pkt_send_prepare(&tx_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&tx_wbuf, 2);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf, ESP_NRF_PROT_CSC_REPLY_DOWNLOAD_DATA);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf, offset);
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&tx_wbuf);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_prepare_upload_data_reply(struct wbuf * const p_wbuf,
	const uint32_t offset, const uint32_t max_bytes_to_send,
	uint32_t * const p_bytes_to_send)
{
	err_code r;

	r = espwifi_pkt_send_prepare(p_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(p_wbuf, 3);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(p_wbuf, ESP_NRF_PROT_CSC_REPLY_UPLOAD_DATA);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(p_wbuf, offset);
	ERR_CHECK(r);

	if (max_bytes_to_send > (p_wbuf->cap - MSGPACK_BIN16_START_BYTES))
		*p_bytes_to_send = (p_wbuf->cap - MSGPACK_BIN16_START_BYTES);
	else
		*p_bytes_to_send = max_bytes_to_send;

	r = wbuf_add_msgpack_bin_start(p_wbuf, *p_bytes_to_send);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_submit_upload_data_reply(const struct wbuf * const p_wbuf)
{
	return espwifi_pkt_send_submit(p_wbuf);
}

err_code espwifi_cmd_send_set_device_id(void)
{
	uint8_t mac[UTIL_MAC_LEN];
	struct wbuf tx_wbuf;
	err_code r;

	r = util_get_mac_addr_bytes(mac, sizeof(mac));
	ERR_CHECK(r);

	r = espwifi_pkt_send_prepare(&tx_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&tx_wbuf, 2);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf,
		ESP_NRF_PROT_CSC_REQUEST_SET_DEVICE_ID);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_bin(&tx_wbuf, mac, sizeof(mac));
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&tx_wbuf);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_send_set_baudrate(const uint32_t baudrate)
{
	struct wbuf tx_wbuf;
	err_code r;

	r = espwifi_pkt_send_prepare(&tx_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&tx_wbuf, 2);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf,
		ESP_NRF_PROT_CSC_REQUEST_SET_BAUDRATE);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf, baudrate);
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&tx_wbuf);
	ERR_CHECK(r);
	return r;
}

err_code espwifi_cmd_send_set_country_code(const char *	const p_country_code)
{
	struct wbuf tx_wbuf;
	err_code r;

	if (p_country_code == NULL)
		return EESPWIFI_INVALID_ARG;

	r = espwifi_pkt_send_prepare(&tx_wbuf);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_array_start(&tx_wbuf, 2);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_uint32(&tx_wbuf,
		ESP_NRF_PROT_CSC_REQUEST_SET_COUNTRY_CODE);
	ERR_CHECK(r);
	r = wbuf_add_msgpack_str(&tx_wbuf, p_country_code,
		ESPWIFI_COUNTRY_CODE_LEN);
	ERR_CHECK(r);
	r = espwifi_pkt_send_submit(&tx_wbuf);
	ERR_CHECK(r);
	return r;
}
