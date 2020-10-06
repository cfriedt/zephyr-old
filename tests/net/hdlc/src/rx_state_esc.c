/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>

#include <net/hdlc.h>
#include <sys/crc.h>
#include <ztest.h>

#include "hdlc_test.h"

static void test_state_esc(bool fcs32)
{
    const uint8_t forty_two = 0x42;
    struct hdlc_context ctx;
    struct hdlc_context pre;
    uint32_t fcs_init;

    if (fcs32) {
        fcs_init = ~HDLC_FCS32_INIT;
        get_ctx32(&ctx);
    } else {
        fcs_init = HDLC_FCS16_INIT;
    	get_ctx16(&ctx);
    }

    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_FLAG), "expected success");
    zassert_equal(ctx.state, HDLC_STATE_DATA, "wrong state");

    /* write ESC */
    memcpy(&pre, &ctx, sizeof(pre));
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_ESC), "expected success");
    zassert_equal(ctx.state, HDLC_STATE_ESC, "wrong state");
    zassert_equal(ctx.frame_len, pre.frame_len, "wrong length");
    zassert_equal(ctx.fcs, pre.fcs, "wrong fcs");

    /* write FLAG ^ XOR */
    memcpy(&pre, &ctx, sizeof(pre));
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_FLAG ^ HDLC_XOR), "hdlc_process_byte() failed");
    zassert_equal(ctx.state, HDLC_STATE_DATA, "wrong state");
    zassert_equal(ctx.frame_len, pre.frame_len + 1, "wrong length");
    zassert_not_equal(ctx.fcs, pre.fcs, "wrong fcs");
    /* received data should be XOR'ed */
    zassert_equal(rx_buf.offs, 1, "wrong buffer length");
    zassert_equal(rx_buf.data[0], HDLC_FLAG, "wrong buffer value");

    hdlc_context_reset(&ctx);
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_FLAG), "expected success");
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_ESC), "expected success");
    zassert_equal(ctx.state, HDLC_STATE_ESC, "wrong state");

    /* write FLAG */
    memcpy(&pre, &ctx, sizeof(pre));
    zassert_equal(-EINVAL, hdlc_process_byte(&ctx, HDLC_FLAG), "expected -EINVAL");
    zassert_equal(ctx.state, HDLC_STATE_START, "wrong state");
    zassert_equal(ctx.fcs, fcs_init, "wrong fcs");
    zassert_equal(ctx.frame_len, 0, "wrong length");
    /* erase() should have been called by reset() */
    zassert_equal(rx_buf.offs, 0, "wrong buffer offset");

    hdlc_context_reset(&ctx);
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_FLAG), "expected success");
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_ESC), "expected success");

    /* escape arbitrary not-invalid byte while in ESC */
    memcpy(&pre, &ctx, sizeof(pre));
    zassert_equal(0, hdlc_process_byte(&ctx, forty_two), "expected success");
    zassert_equal(ctx.state, HDLC_STATE_DATA, "wrong state");
    zassert_not_equal(ctx.fcs, pre.fcs, "wrong fcs");
    zassert_equal(ctx.frame_len, pre.frame_len + 1, "wrong length");
    /* erase() should have been called by reset() */
    zassert_equal(rx_buf.offs, 1, "wrong buffer offset");

    /* hdlc_process_byte() returns an error if the underlying rx function fails.
     * Note: the goal here is tho show that it returns the application-specific error.
     * Depending on the receive buffer, this could be very different.
     */
    hdlc_context_reset(&ctx);
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_FLAG), "expected success");
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_ESC), "expected success");

    /* simulate a full buffer */
    rx_buf.offs = sizeof(rx_buf.data);
    ctx.frame_len = sizeof(rx_buf.data);
    zassert_equal(-ENOSPC, hdlc_process_byte(&ctx, 0x42), "expected -ENOSPC");
    zassert_equal(ctx.state, HDLC_STATE_START, "wrong state");
    zassert_equal(ctx.fcs, fcs_init, "wrong fcs");
    zassert_equal(ctx.frame_len, 0, "wrong length");
    /* erase() should have been called by reset() */
    zassert_equal(rx_buf.offs, 0, "wrong buffer offset");
}

/** Test @ref HDLC_STATE_ESC for 16-bit */
void test_state_esc_16(void)
{
    test_state_esc(false);
}

/** Test @ref HDLC_STATE_ESC for 32-bit */
void test_state_esc_32(void)
{
    test_state_esc(true);
}
