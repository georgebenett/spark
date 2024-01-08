/* Internal driver file; should not be included by other users. */
#pragma once

#include <stdint.h>

#include "error.h"
#include "espwifi_buf.h"
#include "wifi/wifi_types.h"

struct cmd_info {
	uint32_t cmd;
	union {
		struct {
			uint32_t num_args;
			struct rbuf result;
		} request_ok_reply;
		struct {
			uint32_t error_code;
		} request_failed_reply;
		struct {
			uint32_t received_bytes;
			uint32_t total_bytes;
		} fota_status_notif;
		struct {
			uint32_t offset;
		} upload_data_request;
		struct {
			uint32_t content_length;
		} download_metadata_request;
		struct {
			uint32_t offset;
			struct rbuf data;
		} download_data_request;
	};
};

err_code espwifi_cmd_recv(const uint32_t req_cmd,
	struct cmd_info * const p_cmd_info, const uint32_t start_ticks,
	const uint32_t timeout_ticks);

err_code espwifi_cmd_send_simple(const uint32_t cmd);
err_code espwifi_cmd_send_connect_wifi(
	const struct wifi_ap_config * const p_ap_config);
err_code espwifi_cmd_send_fota(const char * const p_url,
	const uint32_t url_len, const struct wifi_http_header * const p_headers,
	const uint8_t num_headers);
err_code espwifi_cmd_send_http_get(const char * const p_url,
	const uint32_t url_len, const struct rbuf * const p_accept,
	const struct wifi_http_header * const p_headers,
	const uint8_t num_headers);
err_code espwifi_cmd_send_http_post(const char * const p_url,
	const uint32_t url_len, const struct rbuf * const p_content_type,
	const uint32_t content_length,
	const struct wifi_http_header * const p_headers,
	const uint8_t num_headers);
err_code espwifi_cmd_send_cancel_reply(void);
err_code espwifi_cmd_send_download_metadata_reply(void);
err_code espwifi_cmd_send_download_data_reply(const uint32_t offset);
err_code espwifi_cmd_prepare_upload_data_reply(struct wbuf * const p_wbuf,
	const uint32_t offset, const uint32_t max_bytes_to_send,
	uint32_t * const p_bytes_to_send);
err_code espwifi_cmd_submit_upload_data_reply(const struct wbuf * const p_wbuf);
err_code espwifi_cmd_send_set_device_id(void);
err_code espwifi_cmd_send_set_baudrate(const uint32_t baudrate);
err_code espwifi_cmd_send_set_country_code(const char * const p_country_code);
