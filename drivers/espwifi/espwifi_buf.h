/* Internal driver file; should not be included by other users. */
#pragma once

#include <stdint.h>

#include <nrf.h>

#include "msgpack.h"

struct rbuf {
	const uint8_t	*p_data;
	uint16_t		len;
};

#define RBUF_INIT_STR(STR)	\
	{ .p_data = (const uint8_t *)STR, .len = sizeof(STR)-1 }

struct wbuf {
	uint8_t			*p_data;
	uint16_t		cap;
};

err_code rbuf_get_data(struct rbuf *p_rbuf, struct rbuf * const p_dst,
	const uint32_t len);

err_code rbuf_get_msgpack_item(struct rbuf * const p_rbuf,
	struct msgpack_item * const p_item);
err_code rbuf_get_msgpack_uint32(struct rbuf *p_msgpack,
	uint32_t * const p_value);
err_code rbuf_get_msgpack_array_start(struct rbuf *p_msgpack,
	uint32_t * const p_len);
err_code rbuf_get_msgpack_bin_start(struct rbuf *p_msgpack,
	uint32_t * const p_len);
err_code rbuf_get_msgpack_bin(struct rbuf *p_msgpack,
	struct rbuf * const p_dst);
err_code rbuf_get_msgpack_str(struct rbuf *p_msgpack,
	struct rbuf * const p_dst);

err_code wbuf_add_buf(struct wbuf * const p_wbuf, const uint8_t * const p_buf,
	const uint32_t buf_len);
err_code wbuf_add_rbuf(struct wbuf * const p_wbuf,
	const struct rbuf * const p_rbuf);
err_code wbuf_add_msgpack_array_start(struct wbuf * const p_wbuf,
	const uint32_t len);
err_code wbuf_add_msgpack_map_start(struct wbuf * const p_wbuf,
	const uint16_t len);
err_code wbuf_add_msgpack_uint32(struct wbuf * const p_wbuf,
	const uint32_t value);
err_code wbuf_add_msgpack_bin_start(struct wbuf * const p_wbuf,
	const uint32_t len);
err_code wbuf_add_msgpack_bin(struct wbuf * const p_wbuf,
	const uint8_t * const p_buf, const uint32_t len);
err_code wbuf_add_msgpack_str(struct wbuf * const p_wbuf,
	const char * const p_buf, const uint16_t len);

__STATIC_INLINE
void rbuf_advance(struct rbuf * const p_rbuf, const uint16_t cnt)
{
	p_rbuf->p_data += cnt;
	p_rbuf->len -= cnt;
}

__STATIC_INLINE
void wbuf_advance(struct wbuf * const p_wbuf, const uint16_t cnt)
{
	p_wbuf->p_data += cnt;
	p_wbuf->cap -= cnt;
}
