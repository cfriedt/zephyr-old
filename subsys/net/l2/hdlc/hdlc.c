/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <net/hdlc.h>
#include <sys/crc.h>
#include <zephyr.h>

/* The nominal 32-bit FCS initializer, similar to the 16-bit FCS
 * initializer, is all ones, so 0xfffffffff. However, there is a quirk
 * in crc32_ieee_update() and crc32_ieee() where the input and output
 * are pre-inverted. That may be for convenience purposes. In any case,
 * as a result, the 32-bit FCS must be inverted here.
 * 
 * Aside from the line below, the only other places where the fcs must
 * be inverted is inside of the DECLARE_HDLC_CONTEXT() and in
 * hdlc_context_fcs_is_good().
 */
#define INVERTED_FCS32_INIT (~((uint32_t)HDLC_FCS32_INIT))

bool hdlc_context_is_valid(struct hdlc_context *hdlc_context)
{
	if (hdlc_context == NULL) {
		return false;
	}

	if (hdlc_context->rx_arg != NULL && hdlc_context->rx == NULL) {
		return false;
	}

	if (hdlc_context->tx_arg != NULL && hdlc_context->tx == NULL) {
		return false;
	}

	if (hdlc_context->erase_arg != NULL && hdlc_context->erase == NULL) {
		return false;
	}

	if (!(HDLC_STATE_START <= hdlc_context->state && hdlc_context->state <= HDLC_STATE_ESC)) {
		return false;
	}

	return true;
}

static void hdlc_context_update_state(struct hdlc_context *hdlc_context, enum hdlc_state state, uint32_t fcs, uint16_t frame_len)
{
	__ASSERT_NO_MSG(hdlc_context != NULL);
	__ASSERT_NO_MSG((hdlc_context->fcs32 && true) || (!hdlc_context->fcs32 && ((hdlc_context->fcs & 0xffff0000) == 0)));

	hdlc_context->state = state;
	hdlc_context->fcs = fcs;
	hdlc_context->frame_len = frame_len;
}

int hdlc_context_init(struct hdlc_context *hdlc_context, bool fcs32, hdlc_rx rx,
	void *rx_arg, hdlc_tx tx, void *tx_arg, hdlc_erase erase, void *erase_arg)
{
	DECLARE_HDLC_CONTEXT(tmp, fcs32, rx, rx_arg, tx, tx_arg, erase, erase_arg);

	if (hdlc_context == NULL) {
		return -EINVAL;
	}

	if (!hdlc_context_is_valid(&tmp)) {
		return -EINVAL;
	}

	hdlc_context_update_state(&tmp, HDLC_STATE_START, tmp.fcs, 0);

	memcpy(hdlc_context, &tmp, sizeof(tmp));

	return 0;
}

void hdlc_context_reset(struct hdlc_context *hdlc_context)
{
	if (hdlc_context == NULL) {
		return;
	}

	if (hdlc_context->erase != NULL) {
		hdlc_context->erase(hdlc_context->erase_arg,
			hdlc_context->frame_len);
	}

	hdlc_context_init(hdlc_context, hdlc_context->fcs32,
		hdlc_context->rx, hdlc_context->rx_arg, hdlc_context->tx,
		hdlc_context->tx_arg, hdlc_context->erase,
		hdlc_context->erase_arg);
}

static void hdlc_update_crc(struct hdlc_context *hdlc_context, uint8_t byte)
{
	if (hdlc_context->fcs32) {
		hdlc_context->fcs = crc32_ieee_update(hdlc_context->fcs, &byte, 1);
	} else {
		__ASSERT_NO_MSG((hdlc_context->fcs & 0xffff0000) == 0);
		hdlc_context->fcs = crc16_ccitt(hdlc_context->fcs, &byte, 1);
	}
}

static bool hdlc_context_fcs_is_good(struct hdlc_context *hdlc_context)
{
	if (hdlc_context->fcs32) {
		/* Quirk with Zephyr's crc32 implementation.
		 * Need to invert the 32-bit FCS here
		 */
		return ~hdlc_context->fcs == HDLC_FCS32_GOOD;
	} else {
		__ASSERT_NO_MSG((hdlc_context->fcs & 0xffff0000) == 0);
		return hdlc_context->fcs == HDLC_FCS16_GOOD;
	}
}

int hdlc_process_byte(struct hdlc_context *hdlc_context, uint8_t byte)
{
	int r;

	if (!hdlc_context_is_valid(hdlc_context)) {
		return -EINVAL;
	}

	switch (hdlc_context->state) {
	case HDLC_STATE_START:

		if (byte == HDLC_FLAG) {
			hdlc_context_update_state(hdlc_context, HDLC_STATE_DATA, hdlc_context->fcs32 ? INVERTED_FCS32_INIT : HDLC_FCS16_INIT, 0);
		}
		return 0;

	case HDLC_STATE_DATA:

		if (byte == HDLC_ESC) {
			hdlc_context_update_state(hdlc_context, HDLC_STATE_ESC, hdlc_context->fcs, hdlc_context->frame_len);
			return 0;
		}

		if (byte == HDLC_FLAG) {

			if (!hdlc_context_fcs_is_good(hdlc_context)) {
				/* Corrupt / invalid frame data */
				hdlc_context_reset(hdlc_context);
				return -EINVAL;
			}

			/* Received a valid frame \o/ */
			r = hdlc_context->frame_len;

			hdlc_context_update_state(hdlc_context, HDLC_STATE_DATA, hdlc_context->fcs32 ? INVERTED_FCS32_INIT : HDLC_FCS16_INIT, 0);
			return r;
		}

		if (hdlc_context->rx) {
			r = hdlc_context->rx(hdlc_context->rx_arg, byte);
			if (r != 0) {
				/* Failed to receive a byte.
				* Transition to START state
				*/
				hdlc_context_reset(hdlc_context);
				return r;
			}
		}

		hdlc_update_crc(hdlc_context, byte);
		hdlc_context_update_state(hdlc_context, HDLC_STATE_DATA, hdlc_context->fcs, hdlc_context->frame_len + 1);
		return 0;

	case HDLC_STATE_ESC:
		if (hdlc_is_reserved(byte)) {
			/* Reserved characters may not appear after ESC.
			 * Transition back to START.
			 */
			hdlc_context_reset(hdlc_context);
			return -EINVAL;
		}
		byte ^= HDLC_XOR;

		if (hdlc_context->rx) {
			r = hdlc_context->rx(hdlc_context->rx_arg, byte);
			if (r != 0) {
				/* Failed to receive a byte.
				* Transition to START state
				*/
				hdlc_context_reset(hdlc_context);
				return r;
			}
		}

		hdlc_update_crc(hdlc_context, byte);
		hdlc_context_update_state(hdlc_context, HDLC_STATE_DATA, hdlc_context->fcs, hdlc_context->frame_len + 1);
		return 0;

	default:
		break;
	}

	/* worst-case scenario: state has been corrupted */
	/* return -ENOTRECOVERABLE; */
	__ASSERT(false, "Unrecoverable state");

	return -EINVAL;
}

static int hdlc_write_inner(struct hdlc_context *hdlc_context, bool should_escape, const uint8_t *buf, uint16_t buf_size)
{
	int r;
	uint8_t byte;
	const uint8_t esc = HDLC_ESC;

	if (!hdlc_context_is_valid(hdlc_context)) {
		return -EINVAL;
	}

	for (; buf_size > 0; --buf_size, ++buf) {
		byte = *buf;

		if (hdlc_context->tx == NULL) {
			continue;
		}

		if (should_escape && hdlc_is_reserved(byte)) {
			r = hdlc_context->tx(hdlc_context->tx_arg, esc);
			if (r != 0) {
				return r;
			}

			byte ^= HDLC_XOR;
		}

		r = hdlc_context->tx(hdlc_context->tx_arg, byte);
		if (r != 0) {
			return r;
		}
	}

	return 0;
}

int hdlc_write(struct hdlc_context *hdlc_context, const uint8_t *addr,
	       uint8_t addr_size, const uint8_t *ctrl, uint8_t ctrl_size,
	       const uint8_t *data, uint16_t data_size)
{
	int r;
	uint16_t fcs16;
	uint32_t fcs32;
	uint8_t fcs[4];
	uint8_t fcs_size;
	const uint8_t flag = HDLC_FLAG;

	if (!hdlc_context_is_valid(hdlc_context)) {
		return -EINVAL;
	}

	if (NULL == addr || NULL == ctrl || 0 == addr_size || 0 == ctrl_size) {
		return -EINVAL;
	}

	if (NULL == data && data_size != 0) {
		return -EINVAL;
	}

	if (hdlc_context->fcs32) {
		fcs32 = crc32_ieee_update(INVERTED_FCS32_INIT, addr, addr_size);
		fcs32 = crc32_ieee_update(fcs32, ctrl, ctrl_size);
		fcs32 = crc32_ieee_update(fcs32, data, data_size);
		fcs_size = 4;
		fcs[0] = fcs32 >> 0;
		fcs[1] = fcs32 >> 8;
		fcs[2] = fcs32 >> 16;
		fcs[3] = fcs32 >> 24;
	} else {
		fcs16 = crc16_ccitt(HDLC_FCS16_INIT, addr, addr_size);
		fcs16 = crc16_ccitt(fcs16, ctrl, ctrl_size);
		fcs16 = crc16_ccitt(fcs16, data, data_size);
		fcs16 = ~fcs16;
		fcs_size = 2;
		fcs[0] = fcs16 >> 0;
		fcs[1] = fcs16 >> 8;
	}

	struct buf {
		bool esc;
		const uint8_t *d;
		uint16_t l;
	} bufs[] = {
		{false, &flag, 1},
		{true, addr, addr_size},
		{true, ctrl, ctrl_size},
		{true, data, data_size},
		{true, fcs, fcs_size},
		{false, &flag, 1},
	};

	if (hdlc_context->tx) {
		
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(bufs); ++i) {
		r = hdlc_write_inner(hdlc_context, bufs[i].esc, bufs[i].d, bufs[i].l);
		if (r != 0) {
			return r;
		}
	}

	return 0;
}
