#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "error.h"
#include "wifi/wifi_types.h"

#define EWIFI_NO_INIT				(EWIFI_BASE + 0x00)
#define EWIFI_INVALID_ARG			(EWIFI_BASE + 0x01)
#define EWIFI_CONNECT_WIFI_FAILED	(EWIFI_BASE + 0x02)

typedef bool (*wifi_is_power_on_fnc)(void);
typedef err_code (*wifi_power_off_fnc)(void);
typedef err_code (*wifi_detect_fnc)(void);
typedef err_code (*wifi_connect_fnc)(
	const struct wifi_ap_config * const p_ap_config);
typedef err_code (*wifi_disconnect_fnc)(void);
typedef err_code (*wifi_query_connected_fnc)
	(struct wifi_ap_info * const p_ap_info);
typedef err_code (*wifi_http_get_fnc)
	(const struct wifi_http_get_params * const p_params);
typedef err_code (*wifi_http_post_fnc)
	(const struct wifi_http_post_params * const p_params);
typedef err_code (*wifi_espfota_fnc)
	(const struct wifi_espfota_params * const p_params);
typedef err_code (*wifi_scan_fnc)(
	struct wifi_scan_result * const p_scan_result);

struct wifi_board {
	wifi_is_power_on_fnc		fp_is_power_on;
	wifi_power_off_fnc			fp_power_off;
	wifi_detect_fnc				fp_detect;
	wifi_scan_fnc				fp_scan;
	wifi_connect_fnc			fp_connect;
	wifi_disconnect_fnc			fp_disconnect;
	wifi_query_connected_fnc	fp_query_connected;
	wifi_http_get_fnc			fp_http_get;
	wifi_http_post_fnc			fp_http_post;
	wifi_espfota_fnc			fp_espfota;
};

#if (FEAT_HW_WIFI == 1)

bool wifi_is_power_on(void);
err_code wifi_power_off(void);
err_code wifi_detect(void);
err_code wifi_scan(struct wifi_scan_result * const p_scan_result);
err_code wifi_connect(const struct wifi_ap_config * const p_ap_config);
err_code wifi_disconnect(void);
err_code wifi_query_connected(struct wifi_ap_info * const p_ap_info);
err_code wifi_http_get(const struct wifi_http_get_params * const p_params);
err_code wifi_http_post(const struct wifi_http_post_params * const p_params);
err_code wifi_espfota(const struct wifi_espfota_params * const p_params);
err_code wifi_init(const struct wifi_board * const p_board);

#else

__STATIC_INLINE
bool wifi_is_power_on(void)
	{ return false; }
__STATIC_INLINE
err_code wifi_power_off(void)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_detect(void)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_scan(struct wifi_scan_result * const p_scan_result)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_connect(const struct wifi_ap_config * const p_ap_config)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_disconnect(void)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_query_connected(struct wifi_ap_info * const p_ap_info)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_http_get(const struct wifi_http_get_params * const p_params)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_http_post(const struct wifi_http_post_params * const p_params)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_espfota(const struct wifi_espfota_params * const p_params)
	{ return EWIFI_NO_INIT; }
__STATIC_INLINE
err_code wifi_init(const struct wifi_board * const p_board)
	{ return ERROR_OK; }

#endif
