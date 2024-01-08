#pragma once

#include <nrf.h>
#include <stdint.h>

#include "error.h"
#include "wifi/wifi_types.h"

#define EESPWIFI_NO_INIT					(EESPWIFI_BASE + 0x00)
#define EESPWIFI_INVALID_ARG				(EESPWIFI_BASE + 0x01)
#define EESPWIFI_MUTEX						(EESPWIFI_BASE + 0x02)
#define EESPWIFI_POWER_OFF					(EESPWIFI_BASE + 0x03)
#define EESPWIFI_RECV_TIMEOUT				(EESPWIFI_BASE + 0x04)
#define EESPWIFI_RECV_SILENCE				(EESPWIFI_BASE + 0x05)
#define EESPWIFI_BUF_OVERFLOW				(EESPWIFI_BASE + 0x06)
#define EESPWIFI_BUF_UNDERFLOW				(EESPWIFI_BASE + 0x07)
#define EESPWIFI_WRONG_MSGPACK_TYPE			(EESPWIFI_BASE + 0x08)
#define EESPWIFI_INVALID_REPLY				(EESPWIFI_BASE + 0x09)
#define EESPWIFI_REQUEST_FAILED				(EESPWIFI_BASE + 0x0A)
#define EESPWIFI_UNKNOWN_REQUEST			(EESPWIFI_BASE + 0x0B)
#define EESPWIFI_GET_MAC_FAILED				(EESPWIFI_BASE + 0x0C)
#define EESPWIFI_GET_CONNECTED_WIFI_FAILED	(EESPWIFI_BASE + 0x0D)
#define EESPWIFI_DISCONNECT_WIFI_FAILED		(EESPWIFI_BASE + 0x0E)
#define EESPWIFI_SCAN_WIFI_FAILED			(EESPWIFI_BASE + 0x10)
#define EESPWIFI_FOTA_FAILED				(EESPWIFI_BASE + 0x13)
#define EESPWIFI_FOTA_NOT_SUPPORTED			(EESPWIFI_BASE + 0x14)
#define EESPWIFI_INVALID_URL				(EESPWIFI_BASE + 0x15)
#define EESPWIFI_UNSUPPORTED_URL			(EESPWIFI_BASE + 0x16)
#define EESPWIFI_ESP_OUT_OF_MEMORY			(EESPWIFI_BASE + 0x17)
#define EESPWIFI_CONNECT_HOST_FAILED		(EESPWIFI_BASE + 0x18)
#define EESPWIFI_SOCKET_SEND_FAILED			(EESPWIFI_BASE + 0x19)
#define EESPWIFI_SOCKET_RECV_FAILED			(EESPWIFI_BASE + 0x1A)
#define EESPWIFI_CONNECTION_CLOSED			(EESPWIFI_BASE + 0x1B)
#define EESPWIFI_SERVER_HEADERS_TOO_LONG	(EESPWIFI_BASE + 0x1C)
#define EESPWIFI_INVALID_SERVER_RESPONSE	(EESPWIFI_BASE + 0x1D)
#define EESPWIFI_SERVER_BODY_TOO_LONG		(EESPWIFI_BASE + 0x1E)
#define EESPWIFI_FOTA_PARTITION_NOT_FOUND	(EESPWIFI_BASE + 0x1F)
#define EESPWIFI_FOTA_INIT_FAILED			(EESPWIFI_BASE + 0x20)
#define EESPWIFI_FOTA_INVALID_FILE_SIZE		(EESPWIFI_BASE + 0x21)
#define EESPWIFI_FOTA_INVALID_FILE_FORMAT	(EESPWIFI_BASE + 0x22)
#define EESPWIFI_FOTA_FLASH_FAILED			(EESPWIFI_BASE + 0x23)
#define EESPWIFI_FOTA_VERIFY_FAILED			(EESPWIFI_BASE + 0x24)
#define EESPWIFI_FOTA_DEPLOY_FAILED			(EESPWIFI_BASE + 0x25)
#define EESPWIFI_GET_FAILED					(EESPWIFI_BASE + 0x26)
#define EESPWIFI_GET_NOT_SUPPORTED			(EESPWIFI_BASE + 0x27)
#define EESPWIFI_POST_FAILED				(EESPWIFI_BASE + 0x28)
#define EESPWIFI_POST_NOT_SUPPORTED			(EESPWIFI_BASE + 0x29)
#define EESPWIFI_PROTOCOL_ERROR				(EESPWIFI_BASE + 0x2A)
#define EESPWIFI_INVALID_CONTENT_LENGTH		(EESPWIFI_BASE + 0x2B)
#define EESPWIFI_GET_FW_VERSION_FAILED		(EESPWIFI_BASE + 0x2C)
#define EESPWIFI_SET_DEVICE_ID_FAILED		(EESPWIFI_BASE + 0x2E)
#define EESPWIFI_INVALID_PWR_LVL			(EESPWIFI_BASE + 0x2F)
#define EESPWIFI_PROG_ENABLED				(EESPWIFI_BASE + 0x30)
#define EESPWIFI_SET_BAUDRATE_FAILED		(EESPWIFI_BASE + 0x31)
#define EESPWIFI_SET_COUNTRY_CODE_FAILED	(EESPWIFI_BASE + 0x32)
/* 0x44c-0x63f reserved for server response codes. */
#define EESPWIFI_SERVER_REQUEST_FAILED_MIN	(EESPWIFI_BASE + 1100)
#define EESPWIFI_SERVER_REQUEST_FAILED_MAX	(EESPWIFI_BASE + 1599)

#define ESPWIFI_FW_VERSION_MAX_LEN			32
#define ESPWIFI_COUNTRY_CODE_LEN			2

struct espwifi_fw_version {
	char	str[ESPWIFI_FW_VERSION_MAX_LEN];
	uint8_t	len;
};

enum espwifi_pwr_active_level {
	ESPWIFI_PWR_ACT_LVL_LOW = 0,
	ESPWIFI_PWR_ACT_LVL_HIGH,
	ESPWIFI_NBR_OF_PWR_ACT_LVL
};

struct espwifi_pins {
	uint8_t	power;
	uint8_t	enable;
	uint8_t io0;
} __packed;

struct espwifi_board {
	struct espwifi_pins				pins;
	enum espwifi_pwr_active_level			power_act_state;
	const NRF_UARTE_Type * const	p_uarte;
};

#if (FEAT_HW_ESPWIFI == 1)

bool espwifi_is_power_on(void);
err_code espwifi_power_off(void);
err_code espwifi_detect(void);
err_code espwifi_scan(struct wifi_scan_result * const p_result);
err_code espwifi_connect(const struct wifi_ap_config * const p_ap_config);
err_code espwifi_disconnect(void);
err_code espwifi_query_connected(struct wifi_ap_info * const p_ap_info);
err_code espwifi_http_get(const struct wifi_http_get_params * const p_params);
err_code espwifi_http_post(const struct wifi_http_post_params * const p_params);
err_code espwifi_fota(const struct wifi_espfota_params * const p_params);
err_code espwifi_get_mac(struct wifi_mac * const p_mac);
err_code espwifi_get_fw_version(struct espwifi_fw_version * const p_fw_version);
err_code espwifi_set_prog_enabled(const bool prog_enabled);
err_code espwifi_set_country_code(const char * const p_country_code);
err_code espwifi_deinit(void);
err_code espwifi_init(const struct espwifi_board * const p_board);

#else

__STATIC_INLINE
bool espwifi_is_power_on(void)
	{ return false; }
__STATIC_INLINE
err_code espwifi_power_off(void)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_detect(void)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_scan(struct wifi_scan_result * const p_result)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_connect(const struct wifi_ap_config * const p_ap_config)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_disconnect(void)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_query_connected(struct wifi_ap_info * const p_ap_info)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_http_get(const struct wifi_http_get_params * const p_params)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_http_post(const struct wifi_http_post_params * const p_params)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_fota(const struct wifi_espfota_params * const p_params)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_get_mac(struct wifi_mac * const p_mac)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_get_fw_version(struct espwifi_fw_version * const p_fw_version)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_set_prog_enabled(const bool prog_enabled)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_set_country_code(const char * const p_country_code)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_deinit(void)
	{ return EESPWIFI_NO_INIT; }
__STATIC_INLINE
err_code espwifi_init(const struct espwifi_board * const p_board)
	{ return ERROR_OK; }

#endif
