#pragma once

#include <stdint.h>

/* Magic chosen to be unlikely to occur in a binary executable. The first byte
 * is non-ascii to ensure it cannot be confused with the text logs from the
 * stage 1 bootloader in the ESP.
 */
#define ESP_NRF_PROT_MAGIC				0x7E8F
#define ESP_NRF_PROT_MAX_PACKET_LEN		2048
#define ESP_NRF_PROT_MAX_PAYLOAD_LEN	(ESP_NRF_PROT_MAX_PACKET_LEN - \
										sizeof(struct esp_nrf_prot_header) - \
										sizeof(struct esp_nrf_prot_footer))

#define ESP_NRF_PROT_FIRST_REPLY_CMD	10000
#define ESP_NRF_PROT_FIRST_NOTIF_CMD	20000

struct esp_nrf_prot_header {
	uint16_t	magic;
	uint16_t	payload_len;
};

struct esp_nrf_prot_footer {
	uint32_t	crc;
};

/* Client-to-server commands. */
enum esp_nrf_prot_csc {
	/* Reserved value */
	ESP_NRF_PROT_CSC_NONE = 0,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_PING,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_GET_MAC,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_GET_FW_VERSION,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_GET_CONNECTED_WIFI,
	/* params: ssid, password */
	ESP_NRF_PROT_CSC_REQUEST_CONNECT_WIFI,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_DISCONNECT_WIFI,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_SCAN_WIFI,
	/* params: url */
	ESP_NRF_PROT_CSC_REQUEST_FOTA_URL,
	/* params: url, accept-mime-type */
	ESP_NRF_PROT_CSC_REQUEST_HTTP_GET,
	/* params: url, content-type, content-length */
	ESP_NRF_PROT_CSC_REQUEST_HTTP_POST,
	/* params: baudrate */
	ESP_NRF_PROT_CSC_REQUEST_SET_BAUDRATE,
	/* params: device id (mac) */
	ESP_NRF_PROT_CSC_REQUEST_SET_DEVICE_ID,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_FOTA_BEGIN,
	/* params: data */
	ESP_NRF_PROT_CSC_REQUEST_FOTA_WRITE,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_FOTA_PERFORM,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REQUEST_FOTA_ABORT,
	/* params: country code (two-letter iso) */
	ESP_NRF_PROT_CSC_REQUEST_SET_COUNTRY_CODE,

	NBR_OF_ESP_NRF_PROT_CSC_REQUESTS,

	/* params: offset, [data] */
	ESP_NRF_PROT_CSC_REPLY_UPLOAD_DATA = ESP_NRF_PROT_FIRST_REPLY_CMD,
	/* params: content_length */
	ESP_NRF_PROT_CSC_REPLY_DOWNLOAD_METADATA,
	/* params: offset */
	ESP_NRF_PROT_CSC_REPLY_DOWNLOAD_DATA,
	/* params: (none) */
	ESP_NRF_PROT_CSC_REPLY_CANCEL,
};

/* Server-to-client commands */
enum esp_nrf_prot_scc {
	/* Reserved value */
	ESP_NRF_PROT_SCC_NONE = 0,
	/* params: offset */
	ESP_NRF_PROT_SCC_REQUEST_UPLOAD_DATA,
	/* params: content_length */
	ESP_NRF_PROT_SCC_REQUEST_DOWNLOAD_METADATA,
	/* params: offset, [data] */
	ESP_NRF_PROT_SCC_REQUEST_DOWNLOAD_DATA,

	/* params: (none) */
	ESP_NRF_PROT_SCC_REPLY_INVALID_REQUEST = ESP_NRF_PROT_FIRST_REPLY_CMD,
	/* params: cmd */
	ESP_NRF_PROT_SCC_REPLY_UNKNOWN_REQUEST,
	/* params: cmd, (result) */
	ESP_NRF_PROT_SCC_REPLY_REQUEST_OK,
	/* params: cmd */
	ESP_NRF_PROT_SCC_REPLY_REQUEST_FAILED,

	/* params: (none) */
	ESP_NRF_PROT_SCC_NOTIF_BOOT_FINISHED = ESP_NRF_PROT_FIRST_NOTIF_CMD,
	/* params: received_bytes, total_bytes */
	ESP_NRF_PROT_SCC_NOTIF_FOTA_STATUS,
};
