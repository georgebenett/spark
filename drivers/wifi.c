#include "wifi.h"

static struct {
	const struct wifi_board	*p_board;
} me;

bool wifi_is_power_on(void)
{
	return (me.p_board && me.p_board->fp_is_power_on());
}

err_code wifi_power_off(void)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_power_off();
}

err_code wifi_detect(void)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_detect();
}

err_code wifi_scan(struct wifi_scan_result * const p_scan_result)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_scan(p_scan_result);
}

err_code wifi_connect(const struct wifi_ap_config * const p_ap_config)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_connect(p_ap_config);
}

err_code wifi_disconnect(void)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_disconnect();
}

err_code wifi_query_connected(struct wifi_ap_info * const p_ap_info)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_query_connected(p_ap_info);
}

err_code wifi_http_get(const struct wifi_http_get_params * const p_params)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_http_get(p_params);
}

err_code wifi_http_post(const struct wifi_http_post_params * const p_params)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_http_post(p_params);
}

err_code wifi_espfota(const struct wifi_espfota_params * const p_params)
{
	if (!me.p_board)
		return EWIFI_NO_INIT;

	return me.p_board->fp_espfota(p_params);
}

err_code wifi_init(const struct wifi_board * const p_board)
{
	if (me.p_board != NULL)
		return ERROR_OK;

	if ((p_board->fp_is_power_on == NULL) ||
		(p_board->fp_power_off == NULL) ||
		(p_board->fp_detect == NULL) ||
		(p_board->fp_scan == NULL) ||
		(p_board->fp_connect == NULL) ||
		(p_board->fp_disconnect == NULL) ||
		(p_board->fp_query_connected == NULL) ||
		(p_board->fp_http_get == NULL) ||
		(p_board->fp_http_post == NULL) ||
		(p_board->fp_espfota == NULL))
		return EWIFI_INVALID_ARG;

	me.p_board = p_board;
	return ERROR_OK;
}
