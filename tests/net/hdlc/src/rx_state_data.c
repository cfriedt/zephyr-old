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

static void test_state_data(bool fcs32)
{
	const uint8_t forty_two = 42;

    struct hdlc_context ctx;
    uint32_t fcs_init;

    if (fcs32) {
        fcs_init = ~HDLC_FCS32_INIT;
        get_ctx32(&ctx);
    } else {
        fcs_init = HDLC_FCS16_INIT;
        get_ctx16(&ctx);
    }

    /* state should be START */
    zassert_equal(ctx.state, HDLC_STATE_START, "wrong state");
    zassert_equal(ctx.frame_len, 0, "wrong length");
    zassert_equal(ctx.fcs, fcs_init, "wrong fcs");

    /* write 0x42 - since it is not a flag, state should remain the same */
    zassert_equal(0, hdlc_process_byte(&ctx, forty_two), "hdlc_process_byte() failed");
    zassert_equal(ctx.state, HDLC_STATE_START, "wrong state");
    zassert_equal(ctx.frame_len, 0, "wrong length");
    zassert_equal(ctx.fcs, fcs_init, "wrong fcs");
    /* erase() should have been called by reset() */
    zassert_equal(rx_buf.offs, 0, "wrong buffer offset");

    /* write flag, only the state should change to DATA */
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_FLAG), "hdlc_process_byte() failed");
    zassert_equal(ctx.state, HDLC_STATE_DATA, "wrong state");
    zassert_equal(ctx.frame_len, 0, "wrong length");
    zassert_equal(ctx.fcs, fcs_init, "wrong fcs");
    /* erase() should have been called by reset() */
    zassert_equal(rx_buf.offs, 0, "wrong buffer offset");

    /* hdlc_process_byte() returns an error if the underlying rx function fails.
     * Note: the goal here is tho show that it returns the application-specific error.
     * Depending on the receive buffer, this could be very different.
     */
    hdlc_context_reset(&ctx);
    zassert_equal(0, hdlc_process_byte(&ctx, HDLC_FLAG), "expected success");

    /* simulate a full buffer */
    rx_buf.offs = sizeof(rx_buf.data);
    ctx.frame_len = sizeof(rx_buf.data);
    zassert_equal(-ENOSPC, hdlc_process_byte(&ctx, 0x42), "expected -ENOSPC");
    zassert_equal(ctx.state, HDLC_STATE_START, "wrong state");
    zassert_equal(ctx.fcs, fcs_init, "wrong fcs");
    zassert_equal(ctx.frame_len, 0, "wrong length");
    /* erase() should have been called by reset() */
    zassert_equal(rx_buf.offs, 0, "wrong buffer offset: %u %u", rx_buf.offs, 0);
}

/** Test @ref HDLC_STATE_DATA for 16-bit */
void test_state_data_16(void)
{
    test_state_data(false);
}

/** Test @ref HDLC_STATE_DATA for 32-bit */
void test_state_data_32(void)
{
    test_state_data(true);
}
