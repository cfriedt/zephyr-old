/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>

#include <net/hdlc.h>
#include <ztest.h>

#include "hdlc_test.h"

static void test_state_start(bool fcs32)
{
    struct hdlc_context ctx;
    struct hdlc_context pre;
    uint32_t fcs_init;

    if (fcs32) {
        /* Quirk in crc32_ieee_update() */
	    fcs_init = ~HDLC_FCS32_INIT;
	    get_ctx32(&ctx);
    } else {
		fcs_init = HDLC_FCS16_INIT;
        get_ctx16(&ctx);
    }

    /* state should be the same */
    zassert_equal(ctx.fcs, fcs_init, "wrong initial fcs");
    zassert_equal(ctx.state, HDLC_STATE_START, "wrong initial state");
    zassert_equal(ctx.frame_len, 0, "wrong initial length");

    /* write a non-HDLC_FLAG byte that should be discarded */
    memcpy(&pre, &ctx, sizeof(pre));
    zassert_equal(0, hdlc_process_byte(&ctx, 0x42), "hdlc_process_byte() failed");

    /* fcs should be the same */
    zassert_equal(ctx.fcs, pre.fcs, "wrong fcs");
    zassert_equal(ctx.state, HDLC_STATE_START, "wrong state");
    zassert_equal(ctx.frame_len, pre.frame_len, "wrong length");
    zassert_equal(rx_buf.offs, 0, "wrong buffer offset: %u %u", rx_buf.offs, 0);

    /* write a reserved HDLC byte that should be discarded */
    hdlc_process_byte(&ctx, HDLC_ESC);

    /* state should be the same */
    zassert_equal(ctx.fcs, pre.fcs, "wrong fcs");
    zassert_equal(ctx.state, HDLC_STATE_START, "wrong state");
    zassert_equal(ctx.frame_len, pre.frame_len, "wrong length");
    zassert_equal(rx_buf.offs, 0, "wrong buffer offset: %u %u", rx_buf.offs, 0);

    /* write a HDLC_FLAG byte that should be discarded, but state should change */
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_FLAG), "hdlc_process_byte() failed");
    zassert_equal(ctx.fcs, pre.fcs, "wrong fcs: %08x %08x", ctx.fcs, pre.fcs);
    zassert_equal(ctx.state, HDLC_STATE_DATA, "wrong state");
    zassert_equal(ctx.frame_len, pre.frame_len, "wrong length");
    zassert_equal(rx_buf.offs, 0, "wrong buffer offset: %u %u", rx_buf.offs, 0);
}

/** Test @ref HDLC_STATE_START for 16-bit */
void test_state_start_16(void)
{
    test_state_start(false);
}

/** Test @ref HDLC_STATE_START for 32-bit */
void test_state_start_32(void)
{
    test_state_start(true);
}

