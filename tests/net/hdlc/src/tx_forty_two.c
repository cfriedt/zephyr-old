#include <net/hdlc.h>

#include <ztest.h>

#include "hdlc_test.h"

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(test_net_hdlc);

void test_tx_forty_two_common(bool fcs32)
{
    const uint8_t addr = 1;
    const uint8_t ctrl = 3;
    const uint8_t forty_two = 0x42;
    const uint8_t *forty_two_frame;
    uint16_t n;

    struct hdlc_context ctx;
    if (fcs32) {
        get_ctx32(&ctx);
        forty_two_frame = forty_two_frame_32;
        n = sizeof(forty_two_frame_32);
    } else {
        get_ctx16(&ctx);
        forty_two_frame = forty_two_frame_16;
        n = sizeof(forty_two_frame_16);
    }

    zassert_equal(0, hdlc_write(&ctx, &addr, 1, &ctrl, 1, &forty_two, 1), "expected success");
    zassert_equal(tx_buf.offs, n, "wrong size:P %u %u", tx_buf.offs, n);
    zassert_equal(0, memcmp(tx_buf.data, forty_two_frame, n), "wrong data");
}

void test_tx_forty_two_16(void)
{
    test_tx_forty_two_common(false);
}

void test_tx_forty_two_32(void)
{
    test_tx_forty_two_common(true);
}
