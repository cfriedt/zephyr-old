#include <net/hdlc.h>

#include <ztest.h>

#include "hdlc_test.h"

void test_tx_hello_world_common(bool fcs32)
{
    const uint8_t addr = 1;
    const uint8_t ctrl = 3;
    const uint8_t *hello_world = HELLO_WORLD_MSG;
    uint16_t len = strlen(hello_world);
    const uint8_t *hello_world_frame;
    uint16_t n;

    struct hdlc_context ctx;
    if (fcs32) {
        get_ctx32(&ctx);
        hello_world_frame = hello_world_frame_32;
        n = sizeof(hello_world_frame_32);
    } else {
        get_ctx16(&ctx);
        hello_world_frame = hello_world_frame_16;
        n = sizeof(hello_world_frame_16);
    }

    zassert_equal(0, hdlc_write(&ctx, &addr, 1, &ctrl, 1, hello_world, len), "expected success");
    zassert_equal(tx_buf.offs, n, "wrong size");
    zassert_equal(0, memcmp(tx_buf.data, hello_world_frame, n), "wrong data");
}

void test_tx_hello_world_16(void)
{
    test_tx_hello_world_common(false);
}

void test_tx_hello_world_32(void)
{
    test_tx_hello_world_common(true);
}
