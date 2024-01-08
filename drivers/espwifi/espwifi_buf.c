#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "espwifi.h"
#include "espwifi_buf.h"

#define MAX_MSGPACK_UINT16_BYTES		3
#define MAX_MSGPACK_UINT32_BYTES		5

err_code rbuf_get_data(struct rbuf *p_rbuf, struct rbuf * const p_dst,
	const uint32_t len)
{
	if ((p_rbuf == NULL) || (p_dst == NULL))
		return EESPWIFI_INVALID_ARG;

	if (p_rbuf->len < len)
		return EESPWIFI_BUF_UNDERFLOW;

	p_dst->p_data = p_rbuf->p_data;
	p_dst->len = len;
	rbuf_advance(p_rbuf, len);
	return ERROR_OK;
}

err_code rbuf_get_msgpack_item(struct rbuf * const p_msgpack,
	struct msgpack_item * const p_item)
{
	uint32_t index;
	err_code r;

	if (p_msgpack == NULL)
		return EESPWIFI_INVALID_ARG;

	index = 0;
	r = msgpack_decode_item(p_msgpack->p_data, &index, p_msgpack->len, p_item);
	if (r != ERROR_OK)
		return r;

	rbuf_advance(p_msgpack, index);
	return ERROR_OK;
}

err_code rbuf_get_msgpack_uint32(struct rbuf *p_msgpack,
	uint32_t * const p_value)
{
	struct msgpack_item item;
	err_code r;

	r = rbuf_get_msgpack_item(p_msgpack, &item);
	if (r != ERROR_OK)
		return r;

	if (item.type != MSGPACK_TYPE_NUMBER)
		return EESPWIFI_WRONG_MSGPACK_TYPE;

	*p_value = item.value;
	return ERROR_OK;
}

err_code rbuf_get_msgpack_array_start(struct rbuf *p_msgpack,
	uint32_t * const p_len)
{
	struct msgpack_item item;
	err_code r;

	r = rbuf_get_msgpack_item(p_msgpack, &item);
	if (r != ERROR_OK)
		return r;

	if (item.type != MSGPACK_TYPE_ARRAY)
		return EESPWIFI_WRONG_MSGPACK_TYPE;

	*p_len = item.value;
	return ERROR_OK;
}

err_code rbuf_get_msgpack_bin_start(struct rbuf *p_msgpack,
	uint32_t * const p_len)
{
	struct msgpack_item item;
	err_code r;

	r = rbuf_get_msgpack_item(p_msgpack, &item);
	if (r != ERROR_OK)
		return r;

	if (item.type != MSGPACK_TYPE_BINARY)
		return EESPWIFI_WRONG_MSGPACK_TYPE;

	*p_len = item.value;
	return ERROR_OK;
}

err_code rbuf_get_msgpack_bin(struct rbuf *p_msgpack,
	struct rbuf * const p_dst)
{
	uint32_t len;
	err_code r;

	r = rbuf_get_msgpack_bin_start(p_msgpack, &len);
	if (r != ERROR_OK)
		return r;

	return rbuf_get_data(p_msgpack, p_dst, len);
}

err_code rbuf_get_msgpack_str(struct rbuf *p_msgpack,
	struct rbuf * const p_dst)
{
	struct msgpack_item item;
	err_code r;

	r = rbuf_get_msgpack_item(p_msgpack, &item);
	if (r != ERROR_OK)
		return r;

	if (item.type != MSGPACK_TYPE_STRING)
		return EESPWIFI_WRONG_MSGPACK_TYPE;

	return rbuf_get_data(p_msgpack, p_dst, item.value);
}

err_code wbuf_add_buf(struct wbuf * const p_wbuf, const uint8_t * const p_buf,
	const uint32_t buf_len)
{
	if (p_wbuf == NULL)
		return EESPWIFI_INVALID_ARG;

	if (buf_len >= p_wbuf->cap)
		return EESPWIFI_BUF_OVERFLOW;

	memcpy(p_wbuf->p_data, p_buf, buf_len);
	wbuf_advance(p_wbuf, buf_len);
	return ERROR_OK;
}

err_code wbuf_add_rbuf(struct wbuf * const p_wbuf,
	const struct rbuf * const p_rbuf)
{
	return wbuf_add_buf(p_wbuf, p_rbuf->p_data, p_rbuf->len);
}

err_code wbuf_add_msgpack_array_start(struct wbuf * const p_wbuf,
	const uint32_t len)
{
	uint8_t *p;

	if (p_wbuf == NULL)
		return EESPWIFI_INVALID_ARG;

	if (p_wbuf->cap < MAX_MSGPACK_UINT32_BYTES)
		return EESPWIFI_BUF_OVERFLOW;

	p = p_wbuf->p_data;
	msgpack_add_array(&p, len);
	wbuf_advance(p_wbuf, (uint16_t)(p - p_wbuf->p_data));
	return ERROR_OK;
}

err_code wbuf_add_msgpack_map_start(struct wbuf * const p_wbuf,
	const uint16_t len)
{
	uint8_t *p;

	if (p_wbuf == NULL)
		return EESPWIFI_INVALID_ARG;

	if (p_wbuf->cap < MAX_MSGPACK_UINT16_BYTES)
		return EESPWIFI_BUF_OVERFLOW;

	p = p_wbuf->p_data;
	msgpack_add_map(&p, len);
	wbuf_advance(p_wbuf, (uint16_t)(p - p_wbuf->p_data));
	return ERROR_OK;
}

err_code wbuf_add_msgpack_uint32(struct wbuf * const p_wbuf,
	const uint32_t value)
{
	uint8_t *p;

	if (p_wbuf == NULL)
		return EESPWIFI_INVALID_ARG;

	if (p_wbuf->cap < MAX_MSGPACK_UINT32_BYTES)
		return EESPWIFI_BUF_OVERFLOW;

	p = p_wbuf->p_data;
	msgpack_add_uint(&p, value);
	wbuf_advance(p_wbuf, (uint16_t)(p - p_wbuf->p_data));
	return ERROR_OK;
}

err_code wbuf_add_msgpack_bin_start(struct wbuf * const p_wbuf,
	const uint32_t len)
{
	uint8_t *p;

	if (p_wbuf == NULL)
		return EESPWIFI_INVALID_ARG;

	if (p_wbuf->cap < MAX_MSGPACK_UINT32_BYTES)
		return EESPWIFI_BUF_OVERFLOW;

	p = p_wbuf->p_data;
	msgpack_add_bin_start(&p, len);
	wbuf_advance(p_wbuf, (uint16_t)(p - p_wbuf->p_data));

	if (p_wbuf->cap < len)
		return EESPWIFI_BUF_OVERFLOW;

	return ERROR_OK;
}

err_code wbuf_add_msgpack_bin(struct wbuf * const p_wbuf,
	const uint8_t * const p_buf, const uint32_t len)
{
	uint8_t *p;

	if ((p_wbuf == NULL) || (p_buf == NULL))
		return EESPWIFI_INVALID_ARG;

	if (p_wbuf->cap < (MAX_MSGPACK_UINT32_BYTES + len))
		return EESPWIFI_BUF_OVERFLOW;

	p = p_wbuf->p_data;
	msgpack_add_bin(&p, p_buf, len);
	wbuf_advance(p_wbuf, (uint16_t)(p - p_wbuf->p_data));
	return ERROR_OK;
}

err_code wbuf_add_msgpack_str(struct wbuf * const p_wbuf,
	const char * const p_buf, const uint16_t len)
{
	uint8_t *p;

	if ((p_wbuf == NULL) || (p_buf == NULL))
		return EESPWIFI_INVALID_ARG;

	if (p_wbuf->cap < (MAX_MSGPACK_UINT32_BYTES + len))
		return EESPWIFI_BUF_OVERFLOW;

	p = p_wbuf->p_data;
	msgpack_add_str(&p, p_buf, len);
	wbuf_advance(p_wbuf, (uint16_t)(p - p_wbuf->p_data));
	return ERROR_OK;
}
