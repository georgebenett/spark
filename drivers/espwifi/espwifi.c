#include <stdbool.h>
#include <string.h>

#include <app_config.h>
#include <nrf_gpio.h>
#include <nrf_uarte.h>

#include <FreeRTOS.h>
#include <freertos_static.h>
#include <semphr.h>
#include <task.h>

#include "boards.h"
#include "drivers/flash.h"
#include "drivers/power.h"
#include "drivers/uarte.h"
#include "drivers/wifi.h"
#include "esp_nrf_prot.h"
#include "espwifi.h"
#include "espwifi_cmd.h"
#include "espwifi_pkt.h"
#include "freertos_tasktracker.h"
#include "session_mgr.h"
#include "settings.h"
#include "util.h"
#include "wifi/wifi_types.h"

#define DRV_NAME						ESPWI
#define LOG_TAG							STRINGIFY(DRV_NAME)
#include "log.h"

#define BOOTUP_BAUDRATE_NRF				NRF_UARTE_BAUDRATE_115200
#define HISPEED_BAUDRATE_NRF			NRF_UARTE_BAUDRATE_1000000
#define HISPEED_BAUDRATE_ESP			1000000
#define MAX_POST_SIZE					(16*1024*1024)

/* At least 50 us must pass between PWR and EN. */
#define ENABLE_DELAY_TICKS				pdMS_TO_TICKS(1)
/* At least 50 us must pass to trigger a reset. Here we take a lot more than
 * that to avoid rapid changes in the power.
 */
#define DISABLE_DELAY_TICKS				pdMS_TO_TICKS(100)
#define REBOOT_TIMEOUT_TICKS			pdMS_TO_TICKS(3*1000)
#define STATUS_INTERVAL_TICKS			pdMS_TO_TICKS(1000)
/* The ESP socket timeout is 60s; set timeout to a bit longer than this. */
#define FOTA_STATUS_TIMEOUT_TICKS		pdMS_TO_TICKS(70*1000)
#define ESP_SOCKET_TIMEOUT_TICKS		pdMS_TO_TICKS(70*1000)
#define STD_TIMEOUT_TICKS				pdMS_TO_TICKS(500)
#define SCAN_TIMEOUT_TICKS				pdMS_TO_TICKS(5*1000)
#define CONNECT_WIFI_TIMEOUT_TICKS		pdMS_TO_TICKS(30*1000)

/* For sanity checks; value picked to allow some future additions. */
#define AUTHMODE_MAX					15

#define ESP_HTTPCLIENT_SERVER_RESP_MIN	(0x80080000 + 100)
#define ESP_HTTPCLIENT_SERVER_RESP_MAX	(0x80080000 + 599)

#define INIT_ERR_CODE_MAPPING(E, N)		{ .esp_code = (E), .nrf_code = (N) }

struct err_code_mapping {
	uint32_t	esp_code;
	err_code	nrf_code;
};

static const struct err_code_mapping esp_err_code_mappings[] = {
	INIT_ERR_CODE_MAPPING(0x80080002, EESPWIFI_INVALID_URL),
	INIT_ERR_CODE_MAPPING(0x80080003, EESPWIFI_UNSUPPORTED_URL),
	INIT_ERR_CODE_MAPPING(0x80080004, EESPWIFI_ESP_OUT_OF_MEMORY),
	INIT_ERR_CODE_MAPPING(0x80080005, EESPWIFI_CONNECT_HOST_FAILED),
	INIT_ERR_CODE_MAPPING(0x80080006, EESPWIFI_SOCKET_SEND_FAILED),
	INIT_ERR_CODE_MAPPING(0x80080007, EESPWIFI_SOCKET_RECV_FAILED),
	INIT_ERR_CODE_MAPPING(0x80080008, EESPWIFI_CONNECTION_CLOSED),
	INIT_ERR_CODE_MAPPING(0x80080009, EESPWIFI_SERVER_HEADERS_TOO_LONG),
	INIT_ERR_CODE_MAPPING(0x8008000A, EESPWIFI_INVALID_SERVER_RESPONSE),
	INIT_ERR_CODE_MAPPING(0x8008000B, EESPWIFI_SERVER_BODY_TOO_LONG),
	INIT_ERR_CODE_MAPPING(0x80090000, EESPWIFI_FOTA_PARTITION_NOT_FOUND),
	INIT_ERR_CODE_MAPPING(0x80090001, EESPWIFI_FOTA_INIT_FAILED),
	INIT_ERR_CODE_MAPPING(0x80090002, EESPWIFI_FOTA_INVALID_FILE_SIZE),
	INIT_ERR_CODE_MAPPING(0x80090003, EESPWIFI_FOTA_INVALID_FILE_FORMAT),
	INIT_ERR_CODE_MAPPING(0x80090004, EESPWIFI_FOTA_FLASH_FAILED),
	INIT_ERR_CODE_MAPPING(0x80090005, EESPWIFI_FOTA_VERIFY_FAILED),
	INIT_ERR_CODE_MAPPING(0x80090006, EESPWIFI_FOTA_DEPLOY_FAILED),
};

static const struct rbuf mime_types[NBR_OF_WIFI_MIME_TYPES] = {
	[WIFI_MIME_TYPE_OCTETSTREAM]	= RBUF_INIT_STR("application/octet-stream"),
	[WIFI_MIME_TYPE_MSGPACK]		= RBUF_INIT_STR("application/x-msgpack"),
};

STATIC_MUTEX(DRV_NAME);

static struct {
	const struct espwifi_board	*p_board;
	SemaphoreHandle_t			mutex_handle;
	bool						enabled;
	bool						prog_enabled;
	struct wifi_mac				mac;
	struct espwifi_fw_version	fw_version;
	uint8_t						regulator_5v_handle;
} me;

static bool country_code_in_settings(const char * const p_country_code)
{
	if ((p_country_code[0] == FLASH_EMPTY_BYTE) ||
		(p_country_code[1] == FLASH_EMPTY_BYTE))
		return false;
	return true;
}

static bool is_valid_headers(const struct wifi_http_header * const p_headers,
	const uint8_t num_headers)
{
	uint8_t i;

	if (num_headers == 0)
		return true;

	if (!p_headers)
		return false;

	for (i = 0; i < num_headers; i++)
		if (!p_headers[i].p_key || !p_headers[i].p_value)
			return false;

	return true;
}

static err_code activate_power(void)
{
	err_code r;

	r = power_set_regulator_5v_enable_state(true, me.regulator_5v_handle);
	if ((r != EPOWER_REG_EN_PIN_NOT_PRESENT) && (r != ERROR_OK))
		return r;

	nrf_gpio_pin_write(me.p_board->pins.power, me.p_board->power_act_state);

	return ERROR_OK;
}

static err_code deactivate_power(void)
{
	err_code r;

	nrf_gpio_pin_write(me.p_board->pins.power, !me.p_board->power_act_state);

	r = power_set_regulator_5v_enable_state(false, me.regulator_5v_handle);
	if (r == EPOWER_REG_EN_PIN_NOT_PRESENT)
		r = ERROR_OK;

	return r;
}

static err_code enable_esp(void)
{
	err_code r;

	if (me.enabled)
		return ERROR_OK;

	r = activate_power();
	ERR_CHECK(r);

	vTaskDelay(ENABLE_DELAY_TICKS);
	nrf_gpio_pin_set(me.p_board->pins.enable);

	me.enabled = true;

	return ERROR_OK;
}

static err_code disable_esp(void)
{
	err_code r;

	if (!me.enabled)
		return ERROR_OK;

	nrf_gpio_pin_clear(me.p_board->pins.enable);
	r = deactivate_power();

	vTaskDelay(DISABLE_DELAY_TICKS);

	me.enabled = false;

	return r;
}

static void handle_timeout(const err_code tmo_r)
{
	err_code r;
	/* Power off on receive timeouts so the ESP is reset. */
	if (tmo_r == EESPWIFI_RECV_TIMEOUT) {
		r = disable_esp();
		if (r != ERROR_OK) {
			LOGE("%s r = 0x%08lX", __func__, r);
			session_mgr_append_err_code(r, __func__);
		}
	}
}

static err_code translate_esp_err(const uint32_t esp_err_code,
	const err_code default_value)
{
	uint32_t i;

	if ((esp_err_code >= ESP_HTTPCLIENT_SERVER_RESP_MIN) &&
		(esp_err_code <= ESP_HTTPCLIENT_SERVER_RESP_MAX))
		return (EESPWIFI_SERVER_REQUEST_FAILED_MIN +
			(esp_err_code - ESP_HTTPCLIENT_SERVER_RESP_MIN));

	for (i = 0; i < ARRAY_SIZE(esp_err_code_mappings); i++)
		if (esp_err_code_mappings[i].esp_code == esp_err_code)
			return esp_err_code_mappings[i].nrf_code;

	LOGW("Missing translation: esp_err_code=0x%08lX default=0x%08lX",
		esp_err_code, default_value);
	return default_value;
}

static err_code get_reply(const uint32_t req_cmd,
	struct cmd_info * const p_cmd_info, const uint32_t timeout_ticks)
{
	uint32_t start_ticks;
	err_code r;

	start_ticks = util_ticks_now();

	while (true) {
		r = espwifi_cmd_recv(req_cmd, p_cmd_info, start_ticks, timeout_ticks);
		ERR_CHECK(r);

		if (p_cmd_info->cmd == ESP_NRF_PROT_SCC_REPLY_REQUEST_OK)
			return ERROR_OK;

		if (p_cmd_info->cmd == ESP_NRF_PROT_SCC_REPLY_REQUEST_FAILED)
			return EESPWIFI_REQUEST_FAILED;

		if (p_cmd_info->cmd == ESP_NRF_PROT_SCC_REPLY_UNKNOWN_REQUEST)
			return EESPWIFI_UNKNOWN_REQUEST;
	}
}

static err_code call_simple_cmd(const uint32_t cmd, const err_code err_reply,
	struct cmd_info * const p_cmd_info, const uint32_t timeout_ticks)
{
	err_code r;

	r = espwifi_cmd_send_simple(cmd);
	ERR_CHECK(r);

	r = get_reply(cmd, p_cmd_info, timeout_ticks);
	if (r == EESPWIFI_REQUEST_FAILED)
		r = err_reply;
	return r;
}

static err_code get_mac(struct wifi_mac * const p_mac)
{
	struct cmd_info cmd_info;
	struct rbuf mac_rbuf;
	err_code r;

	r = call_simple_cmd(ESP_NRF_PROT_CSC_REQUEST_GET_MAC,
		EESPWIFI_GET_MAC_FAILED, &cmd_info, STD_TIMEOUT_TICKS);
	ERR_CHECK(r);

	if (cmd_info.request_ok_reply.num_args != 1)
		return EESPWIFI_INVALID_REPLY;

	r = rbuf_get_msgpack_bin(&cmd_info.request_ok_reply.result, &mac_rbuf);
	ERR_CHECK(r);

	if (cmd_info.request_ok_reply.result.len != 0)
		return EESPWIFI_INVALID_REPLY;

	if (mac_rbuf.len != sizeof(struct wifi_mac))
		return EESPWIFI_INVALID_REPLY;

	memcpy(p_mac->bytes, mac_rbuf.p_data, mac_rbuf.len);
	return ERROR_OK;
}

static err_code get_fw_version(struct espwifi_fw_version * const p_fw_version)
{
	struct cmd_info cmd_info;
	struct rbuf rbuf;
	err_code r;

	r = call_simple_cmd(ESP_NRF_PROT_CSC_REQUEST_GET_FW_VERSION,
		EESPWIFI_GET_FW_VERSION_FAILED, &cmd_info, STD_TIMEOUT_TICKS);
	ERR_CHECK(r);

	if (cmd_info.request_ok_reply.num_args != 1)
		return EESPWIFI_INVALID_REPLY;

	r = rbuf_get_msgpack_str(&cmd_info.request_ok_reply.result, &rbuf);
	ERR_CHECK(r);

	if (cmd_info.request_ok_reply.result.len != 0)
		return EESPWIFI_INVALID_REPLY;

	/* Ensure room for null-terminator. */
	if (rbuf.len >= sizeof(p_fw_version->str))
		return EESPWIFI_INVALID_REPLY;

	memcpy(p_fw_version->str, rbuf.p_data, rbuf.len);
	p_fw_version->len = rbuf.len;
	return ERROR_OK;
}

static err_code set_device_id(void)
{
	struct cmd_info cmd_info;
	err_code r;

	r = espwifi_cmd_send_set_device_id();
	ERR_CHECK(r);

	r = get_reply(ESP_NRF_PROT_CSC_REQUEST_SET_DEVICE_ID, &cmd_info,
		STD_TIMEOUT_TICKS);
	/* Ignore this if it's not supported by the wifi chip. */
	if (r == EESPWIFI_UNKNOWN_REQUEST)
		return ERROR_OK;
	if (r == EESPWIFI_REQUEST_FAILED)
		return EESPWIFI_SET_DEVICE_ID_FAILED;
	return r;
}

static err_code set_baudrate(const uint32_t baudrate)
{
	struct cmd_info cmd_info;
	err_code r;

	r = espwifi_cmd_send_set_baudrate(baudrate);
	ERR_CHECK(r);

	r = get_reply(ESP_NRF_PROT_CSC_REQUEST_SET_BAUDRATE, &cmd_info,
		STD_TIMEOUT_TICKS);
	if (r == EESPWIFI_REQUEST_FAILED)
		return EESPWIFI_SET_BAUDRATE_FAILED;
	return r;
}

static err_code set_country_code(const char * const p_country_code)
{
	struct cmd_info cmd_info;
	err_code r;

	r = espwifi_cmd_send_set_country_code(p_country_code);
	ERR_CHECK(r);

	r = get_reply(ESP_NRF_PROT_CSC_REQUEST_SET_COUNTRY_CODE, &cmd_info,
		STD_TIMEOUT_TICKS);
	if (r == EESPWIFI_REQUEST_FAILED)
		return EESPWIFI_SET_COUNTRY_CODE_FAILED;
	return r;
}

static err_code configure_country_code(void)
{
	struct settings_espwifi settings;
	err_code r;

	r = settings_get_espwifi_settings(&settings);
	ERR_CHECK(r);

	if (!country_code_in_settings(&settings.country_code[0]))
		return ERROR_OK;
	return set_country_code(&settings.country_code[0]);
}

static err_code get_connected_wifi(struct wifi_ap_info * const p_ap_info)
{
	struct cmd_info cmd_info;
	struct msgpack_item item;
	uint32_t authmode;
	struct rbuf ssid;
	uint32_t rssi;
	err_code r;

	r = call_simple_cmd(ESP_NRF_PROT_CSC_REQUEST_GET_CONNECTED_WIFI,
		EESPWIFI_GET_CONNECTED_WIFI_FAILED, &cmd_info, STD_TIMEOUT_TICKS);
	ERR_CHECK(r);

	if (cmd_info.request_ok_reply.num_args < 1)
		return EESPWIFI_INVALID_REPLY;

	r = rbuf_get_msgpack_item(&cmd_info.request_ok_reply.result, &item);
	ERR_CHECK(r);

	if (item.type != MSGPACK_TYPE_ARRAY)
		return EESPWIFI_WRONG_MSGPACK_TYPE;

	if (item.value < 3)
		return EESPWIFI_INVALID_REPLY;

	r = rbuf_get_msgpack_bin(&cmd_info.request_ok_reply.result, &ssid);
	ERR_CHECK(r);

	if (ssid.len > WIFI_MAX_SSID_LEN)
		return EESPWIFI_INVALID_REPLY;

	r = rbuf_get_msgpack_uint32(&cmd_info.request_ok_reply.result, &rssi);
	ERR_CHECK(r);

	r = rbuf_get_msgpack_uint32(&cmd_info.request_ok_reply.result,
		&authmode);
	ERR_CHECK(r);

	if (authmode > AUTHMODE_MAX)
		return EESPWIFI_INVALID_REPLY;

	memcpy(p_ap_info->ssid.data, ssid.p_data, ssid.len);
	p_ap_info->ssid.len = ssid.len;
	p_ap_info->rssi = (int8_t)rssi;
	p_ap_info->authmode = authmode;
	return ERROR_OK;
}

static err_code connect_wifi(const struct wifi_ap_config * const p_ap_config)
{
	struct cmd_info cmd_info;
	err_code r;

	r = espwifi_cmd_send_connect_wifi(p_ap_config);
	ERR_CHECK(r);

	r = get_reply(ESP_NRF_PROT_CSC_REQUEST_CONNECT_WIFI, &cmd_info,
		CONNECT_WIFI_TIMEOUT_TICKS);
	if (r == EESPWIFI_REQUEST_FAILED)
		return EWIFI_CONNECT_WIFI_FAILED; /* Don't log */
	return r;
}

static err_code scan_wifi(struct wifi_scan_result * const p_scan_result)
{
	struct cmd_info cmd_info;
	struct rbuf ssid_rbuf;
	struct rbuf *p_result;
	uint32_t array_len;
	uint32_t authmode;
	uint32_t num_aps;
	uint32_t rssi;
	uint32_t i;
	err_code r;

	r = call_simple_cmd(ESP_NRF_PROT_CSC_REQUEST_SCAN_WIFI,
		EESPWIFI_SCAN_WIFI_FAILED, &cmd_info, SCAN_TIMEOUT_TICKS);
	ERR_CHECK(r);

	if (cmd_info.request_ok_reply.num_args != 1)
		return EESPWIFI_INVALID_REPLY;

	/* Temporary variable used to reduce line noise. */
	p_result = &cmd_info.request_ok_reply.result;

	r = rbuf_get_msgpack_array_start(p_result, &num_aps);
	ERR_CHECK(r);

	memset(p_scan_result, 0, sizeof(*p_scan_result));

	for (i = 0; (i < num_aps) && (i < WIFI_MAX_AP_INFOS); i++) {
		r = rbuf_get_msgpack_array_start(p_result, &array_len);
		ERR_CHECK(r);

		if (array_len < 3)
			return EESPWIFI_INVALID_REPLY;

		r = rbuf_get_msgpack_bin(p_result, &ssid_rbuf);
		ERR_CHECK(r);

		if (ssid_rbuf.len > WIFI_MAX_SSID_LEN)
			return EESPWIFI_INVALID_REPLY;

		r = rbuf_get_msgpack_uint32(p_result, &rssi);
		ERR_CHECK(r);

		r = rbuf_get_msgpack_uint32(p_result, &authmode);
		ERR_CHECK(r);

		if (authmode > AUTHMODE_MAX)
			return EESPWIFI_INVALID_REPLY;

		memcpy(p_scan_result->ap_infos[i].ssid.data, ssid_rbuf.p_data,
			ssid_rbuf.len);
		p_scan_result->ap_infos[i].ssid.len = ssid_rbuf.len;
		p_scan_result->ap_infos[i].rssi = (int8_t)(int32_t)rssi;
		p_scan_result->ap_infos[i].authmode = authmode;
		p_scan_result->num_ap_infos++;
	}
	return ERROR_OK;
}

static err_code await_reboot(void)
{
	struct cmd_info cmd_info;
	uint32_t start_ticks;
	err_code r;

	start_ticks = util_ticks_now();

	while (true) {
		r = espwifi_cmd_recv(ESP_NRF_PROT_SCC_NONE, &cmd_info, start_ticks,
			REBOOT_TIMEOUT_TICKS);
		/* Assume everything is OK if we timeout - we can't be sure. */
		if (r == EESPWIFI_RECV_TIMEOUT)
			return ERROR_OK;
		ERR_CHECK(r);

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_NOTIF_BOOT_FINISHED)
			return ERROR_OK;
	}
}

static err_code configure_esp(void)
{
	struct wifi_mac mac;
	err_code r;

	r = set_baudrate(HISPEED_BAUDRATE_ESP);
	ERR_CHECK(r);

	r = uarte_set_baudrate(me.p_board->p_uarte, HISPEED_BAUDRATE_NRF);
	ERR_CHECK(r);

	r = get_mac(&mac);
	ERR_CHECK(r);

	if (memcmp(&me.mac, &mac, sizeof(mac))) {
		me.mac = mac;

		LOGI("MAC: %02X:%02X:%02X:%02X:%02X:%02X",
			mac.bytes[0], mac.bytes[1], mac.bytes[2],
			mac.bytes[3], mac.bytes[4], mac.bytes[5]);
	}

	r = get_fw_version(&me.fw_version);
	ERR_CHECK(r);

	r = configure_country_code();
	ERR_CHECK(r);

	return set_device_id();
}

static err_code power_and_configure_esp(void)
{
	err_code r;

	r = espwifi_pkt_clear_recv_buffer();
	ERR_CHECK(r);

	if (me.enabled) {
		/* We can send any command here. If the ESP has rebooted then the
		 * command will fail due to different baud rates. If the command
		 * succeeds we know the ESP must have been configured already.
		 */
		r = set_device_id();
		if (r == ERROR_OK)
			return r;

		/* Power off to trigger a reset. */
		r = disable_esp();
		ERR_CHECK(r);

		r = espwifi_pkt_clear_recv_buffer();
		ERR_CHECK(r);
	}

	r = uarte_set_baudrate(me.p_board->p_uarte, BOOTUP_BAUDRATE_NRF);
	ERR_CHECK(r);

	r = enable_esp();
	ERR_CHECK(r);

	r = await_reboot();
	ERR_CHECK(r);

	return configure_esp();
}

bool espwifi_is_power_on(void)
{
	return me.enabled;
}

err_code espwifi_power_off(void)
{
	err_code r;

	r = ERROR_OK;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (!me.prog_enabled)
		r = disable_esp();

	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_detect(void)
{
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (me.prog_enabled) {
		r = EESPWIFI_PROG_ENABLED;
		goto exit;
	}

	r = power_and_configure_esp();
	handle_timeout(r);

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_scan(struct wifi_scan_result * const p_result)
{
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_result == NULL)
		return EESPWIFI_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (me.prog_enabled) {
		r = EESPWIFI_PROG_ENABLED;
		goto exit;
	}

	r = power_and_configure_esp();
	ERR_CHECK_GOTO(r, exit);

	r = scan_wifi(p_result);
	ERR_CHECK_GOTO(r, exit);

exit:
	handle_timeout(r);
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_connect(const struct wifi_ap_config * const p_ap_config)
{
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_ap_config == NULL)
		return EESPWIFI_INVALID_ARG;

	if (!wifi_ap_config_is_valid(p_ap_config))
		return EESPWIFI_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (me.prog_enabled) {
		r = EESPWIFI_PROG_ENABLED;
		goto exit;
	}

	r = power_and_configure_esp();
	ERR_CHECK_GOTO(r, exit);

	r = connect_wifi(p_ap_config);

exit:
	handle_timeout(r);
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_disconnect(void)
{
	struct cmd_info cmd_info;
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (me.prog_enabled) {
		r = EESPWIFI_PROG_ENABLED;
		goto exit;
	}

	if (!me.enabled) {
		r = ERROR_OK;
		goto exit;
	}

	r = call_simple_cmd(ESP_NRF_PROT_CSC_REQUEST_DISCONNECT_WIFI,
		EESPWIFI_DISCONNECT_WIFI_FAILED, &cmd_info, STD_TIMEOUT_TICKS);
	ERR_CHECK_GOTO(r, exit);

exit:
	handle_timeout(r);
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_query_connected(struct wifi_ap_info * const p_ap_info)
{
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_ap_info == NULL)
		return EESPWIFI_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (me.prog_enabled) {
		r = EESPWIFI_PROG_ENABLED;
		goto exit;
	}

	if (!me.enabled) {
		memset(p_ap_info, 0, sizeof(*p_ap_info));
		r = ERROR_OK;
		goto exit;
	}

	r = get_connected_wifi(p_ap_info);
	ERR_CHECK_GOTO(r, exit);

exit:
	handle_timeout(r);
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_http_get(const struct wifi_http_get_params * const p_params)
{
	struct cmd_info cmd_info;
	uint32_t content_len;
	bool got_content_len;
	uint32_t bytes_recv;
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_params == NULL)
		return EESPWIFI_INVALID_ARG;

	if ((p_params->p_url == NULL) ||
		(p_params->url_len == 0) ||
		(p_params->url_len > WIFI_MAX_URL_LEN) ||
		(p_params->accept >= NBR_OF_WIFI_MIME_TYPES) ||
		(p_params->len_cb == NULL) ||
		(p_params->data_cb == NULL))
		return EESPWIFI_INVALID_ARG;

	if (!is_valid_headers(p_params->p_headers, p_params->num_headers))
		return EESPWIFI_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (me.prog_enabled) {
		r = EESPWIFI_PROG_ENABLED;
		goto exit;
	}

	if (!me.enabled) {
		r = EESPWIFI_POWER_OFF;
		goto exit;
	}

	content_len = 0;
	got_content_len = false;
	bytes_recv = 0;

	r = espwifi_cmd_send_http_get(p_params->p_url,
		p_params->url_len, &mime_types[p_params->accept],
		p_params->p_headers, p_params->num_headers);
	ERR_CHECK_GOTO(r, exit);

	while (true) {
		r = espwifi_cmd_recv(ESP_NRF_PROT_CSC_REQUEST_HTTP_GET,
			&cmd_info, util_ticks_now(), ESP_SOCKET_TIMEOUT_TICKS);
		ERR_CHECK_GOTO(r, exit);

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_REQUEST_OK)
			break; /* Download complete */

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_REQUEST_FAILED) {
			r = translate_esp_err(cmd_info.request_failed_reply.error_code,
				EESPWIFI_GET_FAILED);
			goto exit; /* Don't log */
		}

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_UNKNOWN_REQUEST) {
			r = EESPWIFI_GET_NOT_SUPPORTED;
			ERR_CHECK_GOTO(r, exit);
		}

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REQUEST_DOWNLOAD_METADATA) {
			if (!got_content_len) {
				got_content_len = true;
				content_len = cmd_info.download_metadata_request.content_length;
				r = p_params->len_cb(content_len);
				if (r != ERROR_OK)
					goto exit; /* Don't log */

			} else if (content_len !=
				cmd_info.download_metadata_request.content_length) {
				r = EESPWIFI_PROTOCOL_ERROR;
				ERR_CHECK_GOTO(r, exit);
			}
			r = espwifi_cmd_send_download_metadata_reply();
			ERR_CHECK_GOTO(r, exit);
			continue;
		}

		if ((cmd_info.cmd != ESP_NRF_PROT_SCC_REQUEST_DOWNLOAD_DATA) ||
			(cmd_info.download_data_request.offset > bytes_recv) ||
			!got_content_len) {
			r = EESPWIFI_PROTOCOL_ERROR;
			ERR_CHECK_GOTO(r, exit);
		}

		/* Is this a resend? */
		if (cmd_info.download_data_request.offset < bytes_recv) {
			if ((cmd_info.download_data_request.offset +
				cmd_info.download_data_request.data.len) != bytes_recv) {
				r = EESPWIFI_PROTOCOL_ERROR;
				ERR_CHECK_GOTO(r, exit);
			}

			LOGV("Data resent: ofs=%lu len=%u",
				cmd_info.download_data_request.offset,
				cmd_info.download_data_request.data.len);
			r = espwifi_cmd_send_download_data_reply(
				cmd_info.download_data_request.offset);
			ERR_CHECK_GOTO(r, exit);
			continue;
		}

		bytes_recv += cmd_info.download_data_request.data.len;

		if (bytes_recv > content_len) {
			r = EESPWIFI_PROTOCOL_ERROR;
			ERR_CHECK_GOTO(r, exit);
		}

		r = p_params->data_cb(cmd_info.download_data_request.data.p_data,
			cmd_info.download_data_request.data.len);
		if (r != ERROR_OK)
			goto exit; /* Don't log */

		r = espwifi_cmd_send_download_data_reply(
			cmd_info.download_data_request.offset);
		ERR_CHECK_GOTO(r, exit);
	}

	if (!got_content_len || (bytes_recv != content_len)) {
		r = EESPWIFI_PROTOCOL_ERROR;
		ERR_CHECK_GOTO(r, exit);
	}

exit:
	/* We don't have to check if/why something failed here; the ESP won't mind
	 * if we send an extra cancel command and will silently ignore it.
	 */
	(void) espwifi_cmd_send_cancel_reply();

	handle_timeout(r);
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_http_post(const struct wifi_http_post_params * const p_params)
{
	struct cmd_info cmd_info;
	uint32_t bytes_to_send;
	uint32_t content_len;
	struct wbuf tx_wbuf;
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_params == NULL)
		return EESPWIFI_INVALID_ARG;

	if ((p_params->p_url == NULL) ||
		(p_params->url_len == 0) ||
		(p_params->url_len > WIFI_MAX_URL_LEN) ||
		(p_params->content_type >= NBR_OF_WIFI_MIME_TYPES) ||
		(p_params->len_cb == NULL) ||
		(p_params->data_cb == NULL))
		return EESPWIFI_INVALID_ARG;

	if (!is_valid_headers(p_params->p_headers, p_params->num_headers))
		return EESPWIFI_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (me.prog_enabled) {
		r = EESPWIFI_PROG_ENABLED;
		goto exit;
	}

	if (!me.enabled) {
		r = EESPWIFI_POWER_OFF;
		goto exit;
	}

	content_len = 0;
	r = p_params->len_cb(&content_len);
	if (r != ERROR_OK)
		goto exit; /* Don't log */

	if ((content_len == 0) || (content_len >= MAX_POST_SIZE)) {
		r = EESPWIFI_INVALID_CONTENT_LENGTH;
		ERR_CHECK_GOTO(r, exit);
	}

	r = espwifi_cmd_send_http_post(p_params->p_url, p_params->url_len,
		&mime_types[p_params->content_type], content_len,
		p_params->p_headers, p_params->num_headers);
	ERR_CHECK_GOTO(r, exit);

	while (true) {
		r = espwifi_cmd_recv(ESP_NRF_PROT_CSC_REQUEST_HTTP_POST, &cmd_info,
			util_ticks_now(), ESP_SOCKET_TIMEOUT_TICKS);
		ERR_CHECK_GOTO(r, exit);

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_REQUEST_OK)
			break; /* Upload complete */

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_REQUEST_FAILED) {
			r = translate_esp_err(cmd_info.request_failed_reply.error_code,
				EESPWIFI_POST_FAILED);
			goto exit; /* Don't log */
		}

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_UNKNOWN_REQUEST) {
			r = EESPWIFI_PROTOCOL_ERROR;
			ERR_CHECK_GOTO(r, exit);
		}

		if ((cmd_info.cmd != ESP_NRF_PROT_SCC_REQUEST_UPLOAD_DATA) ||
			(cmd_info.upload_data_request.offset >= content_len)) {
			r = EESPWIFI_PROTOCOL_ERROR;
			ERR_CHECK_GOTO(r, exit);
		}

		r = espwifi_cmd_prepare_upload_data_reply(&tx_wbuf,
			cmd_info.upload_data_request.offset,
			(content_len - cmd_info.upload_data_request.offset),
			&bytes_to_send);
		ERR_CHECK_GOTO(r, exit);

		r = p_params->data_cb(tx_wbuf.p_data, bytes_to_send,
			cmd_info.upload_data_request.offset);
		if (r != ERROR_OK)
			goto exit; /* Don't log */

		wbuf_advance(&tx_wbuf, bytes_to_send);
		r = espwifi_cmd_submit_upload_data_reply(&tx_wbuf);
		ERR_CHECK_GOTO(r, exit);
	}

exit:
	/* We don't have to check if/why something failed here; the ESP won't mind
	 * if we send an extra cancel command and will silently ignore it.
	 */
	(void) espwifi_cmd_send_cancel_reply();

	handle_timeout(r);
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_fota(const struct wifi_espfota_params * const p_params)
{
	struct cmd_info cmd_info;
	uint32_t start_ticks;
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_params == NULL)
		return EESPWIFI_INVALID_ARG;

	if ((p_params->p_url == NULL) ||
		(p_params->url_len == 0) ||
		(p_params->url_len > WIFI_MAX_URL_LEN) ||
		(p_params->status_cb == NULL))
		return EESPWIFI_INVALID_ARG;

	if (!is_valid_headers(p_params->p_headers, p_params->num_headers))
		return EESPWIFI_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (me.prog_enabled) {
		r = EESPWIFI_PROG_ENABLED;
		goto exit;
	}

	if (!me.enabled) {
		r = EESPWIFI_POWER_OFF;
		goto exit;
	}

	LOGI("FOTA started");

	r = espwifi_cmd_send_fota(p_params->p_url, p_params->url_len,
		p_params->p_headers, p_params->num_headers);
	ERR_CHECK_GOTO(r, exit);

	start_ticks = util_ticks_now();

	while (true) {
		r = espwifi_cmd_recv(ESP_NRF_PROT_CSC_REQUEST_FOTA_URL, &cmd_info,
			start_ticks, FOTA_STATUS_TIMEOUT_TICKS);
		ERR_CHECK_GOTO(r, exit);

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_REQUEST_OK)
			break;

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_REQUEST_FAILED) {
			r = translate_esp_err(cmd_info.request_failed_reply.error_code,
				EESPWIFI_FOTA_FAILED);
			goto exit; /* Result is logged at end of the function. */
		}

		if (cmd_info.cmd == ESP_NRF_PROT_SCC_REPLY_UNKNOWN_REQUEST) {
			r = EESPWIFI_FOTA_NOT_SUPPORTED;
			ERR_CHECK_GOTO(r, exit);
		}

		if (cmd_info.cmd != ESP_NRF_PROT_SCC_NOTIF_FOTA_STATUS)
			continue;

		LOGV("FOTA status: %lu of %lu",
			cmd_info.fota_status_notif.received_bytes,
			cmd_info.fota_status_notif.total_bytes);

		p_params->status_cb(cmd_info.fota_status_notif.received_bytes,
			cmd_info.fota_status_notif.total_bytes);

		/* Reset timeout while we're getting updates. */
		start_ticks = util_ticks_now();
	}

	r = uarte_set_baudrate(me.p_board->p_uarte, BOOTUP_BAUDRATE_NRF);
	ERR_CHECK(r);

	r = await_reboot();
	ERR_CHECK_GOTO(r, exit);

	r = configure_esp();
	ERR_CHECK_GOTO(r, exit);

exit:
	handle_timeout(r);
	util_mutex_unlock(me.mutex_handle);
	LOGI("FOTA result: r=0x%08lX", r);
	return r;
}

err_code espwifi_get_mac(struct wifi_mac * const p_mac)
{
	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_mac == NULL)
		return EESPWIFI_INVALID_ARG;

	*p_mac = me.mac;
	return ERROR_OK;
}

err_code espwifi_get_fw_version(struct espwifi_fw_version * const p_fw_version)
{
	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_fw_version == NULL)
		return EESPWIFI_INVALID_ARG;

	*p_fw_version = me.fw_version;
	return ERROR_OK;
}

err_code espwifi_set_prog_enabled(const bool prog_enabled)
{
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	if (prog_enabled) {
		memset(&me.fw_version, 0, sizeof(me.fw_version));
		nrf_gpio_cfg_default(me.p_board->pins.enable);
		nrf_gpio_cfg_default(me.p_board->pins.io0);
		r = activate_power();
		ERR_CHECK_GOTO(r, error);
	} else {
		nrf_gpio_pin_clear(me.p_board->pins.enable);
		nrf_gpio_cfg_output(me.p_board->pins.enable);
		nrf_gpio_cfg_output(me.p_board->pins.io0);
		r = deactivate_power();
		ERR_CHECK_GOTO(r, error);
	}

	r = espwifi_pkt_set_enabled(!prog_enabled);
	ERR_CHECK_GOTO(r, error);

	me.enabled = prog_enabled;
	me.prog_enabled = prog_enabled;
	LOGI("Progamming enabled: %u", prog_enabled);
	goto exit;

error:
	nrf_gpio_pin_clear(me.p_board->pins.enable);
	nrf_gpio_cfg_output(me.p_board->pins.enable);
	nrf_gpio_cfg_output(me.p_board->pins.io0);
	me.enabled = false;
	me.prog_enabled = false;

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_set_country_code(const char * const p_country_code)
{
	struct settings_espwifi settings;
	err_code r;

	if (me.p_board == NULL)
		return EESPWIFI_NO_INIT;

	if (p_country_code == NULL)
		return EESPWIFI_INVALID_ARG;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	r = settings_get_espwifi_settings(&settings);
	ERR_CHECK_GOTO(r, exit);

	r = power_and_configure_esp();
	ERR_CHECK_GOTO(r, exit);

	r = set_country_code(p_country_code);
	ERR_CHECK_GOTO(r, exit);

	memcpy(&settings.country_code[0], p_country_code, ESPWIFI_COUNTRY_CODE_LEN);

	r = settings_set_espwifi_settings(&settings);

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_deinit(void)
{
	err_code r;

	if (me.p_board == NULL)
		return ERROR_OK;

	if (!util_mutex_lock(me.mutex_handle))
		return EESPWIFI_MUTEX;

	r = uarte_tx_disable(me.p_board->p_uarte);
	ERR_CHECK_GOTO(r, exit);

	r = uarte_rx_disable(me.p_board->p_uarte);
	ERR_CHECK_GOTO(r, exit);

	me.p_board = NULL;

exit:
	util_mutex_unlock(me.mutex_handle);
	return r;
}

err_code espwifi_init(const struct espwifi_board * const p_board)
{
	err_code r;

	if (me.p_board != NULL)
		return ERROR_OK;

	if ((p_board == NULL) || (p_board->p_uarte == NULL))
		return EESPWIFI_INVALID_ARG;

	if (p_board->power_act_state >= ESPWIFI_NBR_OF_PWR_ACT_LVL)
		return EESPWIFI_INVALID_PWR_LVL;

	r = util_validate_pins((uint8_t *)&p_board->pins, sizeof(p_board->pins));
	ERR_CHECK(r);

	r = power_register_regulator_5v(&me.regulator_5v_handle);
	ERR_CHECK(r);

	nrf_gpio_pin_write(p_board->pins.power, p_board->power_act_state);
	nrf_gpio_cfg_output(p_board->pins.power);

	nrf_gpio_pin_clear(p_board->pins.enable);
	nrf_gpio_cfg_output(p_board->pins.enable);

	nrf_gpio_pin_set(p_board->pins.io0);
	nrf_gpio_cfg_output(p_board->pins.io0);

	r = espwifi_pkt_init(p_board->p_uarte);
	ERR_CHECK(r);

	me.mutex_handle = CREATE_STATIC_MUTEX(DRV_NAME);
	me.p_board = p_board;

	return ERROR_OK;
}
