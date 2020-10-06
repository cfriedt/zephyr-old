/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>

#include <net/hdlc.h>
#include <ztest.h>

#include "hdlc_test.h"

static void test_init_common(bool fcs32)
{
    struct hdlc_context ctx;
    uint32_t fcs_init;

    if (fcs32) {
	    get_ctx32(&ctx);
        /* Quirk in Zephyr's crc32 */
	    fcs_init = ~HDLC_FCS32_INIT;
    } else {
		get_ctx16(&ctx);
		fcs_init = HDLC_FCS16_INIT;
    }

    /* test declaration (happy path) */
    zassert_equal(true, hdlc_context_is_valid(&ctx), "declaration not valid");
    zassert_equal(0, ctx.frame_len, "wrong frame len");
    zassert_equal(fcs_init, ctx.fcs, "wrong init value");
    zassert_equal(HDLC_STATE_START, ctx.state, "wrong state");

    /* test initialization (happy path) */
    zassert_equal(0, hdlc_context_init(&ctx, fcs32, rx, &rx_buf, tx, &tx_buf, erase, &rx_buf), "initialization failed");
    zassert_equal(true, hdlc_context_is_valid(&ctx), "initialization not valid");
    zassert_equal(0, ctx.frame_len, "wrong frame len");
    zassert_equal(fcs_init, ctx.fcs, "wrong init value: exp: %x act: %x", fcs_init, ctx.fcs);
    zassert_equal(HDLC_STATE_START, ctx.state, "wrong state");

    /* test initialization (degenerate case) */
    zassert_equal(0, hdlc_context_init(&ctx, fcs32, NULL, NULL, NULL, NULL, NULL, NULL), "expected success");

    /* test invalid arguments */
    zassert_equal(-EINVAL, hdlc_context_init(NULL, fcs32, rx, &rx_buf, tx, &tx_buf, erase, &rx_buf), "expected -EINVAL");
    zassert_equal(-EINVAL, hdlc_context_init(&ctx, fcs32, NULL, &rx_buf, tx, &tx_buf, erase, &rx_buf), "expected -EINVAL");
    zassert_equal(-EINVAL, hdlc_context_init(&ctx, fcs32, rx, &rx_buf, NULL, &tx_buf, erase, &rx_buf), "expected -EINVAL");
    zassert_equal(-EINVAL, hdlc_context_init(&ctx, fcs32, rx, &rx_buf, tx, &tx_buf, NULL, &rx_buf), "expected -EINVAL");
}

/**  Check that initialization produces a valid 16-bit context */
void test_init_16(void)
{
    test_init_common(false);
}

/**  Check that initialization produces a valid 32-bit context */
void test_init_32(void)
{
    test_init_common(true);
}
