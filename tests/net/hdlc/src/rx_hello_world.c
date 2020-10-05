#include <string.h>

#include <net/hdlc.h>
#include <sys/crc.h>
#include <sys/util.h>
#include <ztest.h>

#include "hdlc_test.h"

/** Test receiving a simple HDLC message */
void test_rx_hello_world_common(bool fcs32)
{
    /* Number of fcs bytes */
    const uint16_t m = 2;
    uint16_t n;
    const uint8_t *msg;
    struct hdlc_context ctx;

    if (fcs32) {
	    get_ctx32(&ctx);
        n = sizeof(hello_world_frame_32);
        msg = hello_world_frame_32;
    } else {
	    get_ctx16(&ctx);
        n = sizeof(hello_world_frame_16);
        msg = hello_world_frame_16;
    }

    /*
     * Assert that no reserved bytes exist in the message.
     * Otherwise, they would need to be escaped and we are being lazy here.
     */
    for (size_t i = 1; i < n - 1; ++i) {
	    zassert_false(hdlc_is_reserved(msg[i]), "found a reserved byte in the message");
    }

    /* process each byte escept the last flag */
    for (size_t i = 0; i < n - 1; ++i) {
	    zassert_equal(0, hdlc_process_byte(&ctx, msg[i]), "failed to process byte");
    }

    /* process the last flag. this should result in a finished frame */
    int r = hdlc_process_byte(&ctx, msg[n-1]);

    zassert_equal(r, n - m, "wrong message size: r: %d (n-m): %d", r, n - m);

    /* check the message contents are the same */
    zassert_equal(0, memcmp(rx_buf.data, &msg[1], n - m), "wrong message contents");
}

void test_rx_hello_world_16(void)
{
	return test_rx_hello_world_common(false);
}

void test_rx_hello_world_32(void)
{
	return test_rx_hello_world_common(true);
}
