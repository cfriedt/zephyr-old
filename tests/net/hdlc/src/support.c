#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <sys/crc.h>
#include <ztest.h>

#include "hdlc_test.h"

#define FLAGSTR "\x7e"

BUILD_ASSERT(0x7e == HDLC_FLAG, "FLAGSTR and HDLC_FLAG are inconsistent");

/* buffers */
struct buf rx_buf;
struct buf tx_buf;

/* These buffers were created using the sample code in sections
 * C.2 and C.3 of RFC 1662
 */
const uint8_t forty_two_frame_16[7] =
    /* clang-format off */
    FLAGSTR
    "\x01" /* addr */
    "\x03" /* ctrl */
    "\x42" /* forty-two */
    "\x6e" /* LSB of FCS-16 */
    "\xd7" /* LSB of FCS-16 */
    FLAGSTR
    /* clang-format on */
    ;

const uint8_t forty_two_frame_32[9] =
    /* clang-format off */
    FLAGSTR
    "\x01" /* addr */
    "\x03" /* ctrl */
    "\x42" /* forty-two */
    "\x5a" /* LSB of FCS-32 */
    "\xc0" /* 2nd LSB of FCS-32 */
    "\x7c" /* 2nd MSB of FCS-32 */
    "\x4d" /* MSB of FCS-32 */
    FLAGSTR
    /* clang-format on */
    ;

const uint8_t hello_world_frame_16[24] =
    /* clang-format off */
    FLAGSTR
    "\x01" /* addr */
    "\x03" /* ctrl */
    HELLO_WORLD_MSG
    "\x9e" /* LSB of FCS-16 */
    "\x50" /* LSB of FCS-16 */
    FLAGSTR
    /* clang-format on */
    ;

const uint8_t hello_world_frame_32[26] =
    /* clang-format off */
    FLAGSTR
    "\x01" /* addr */
    "\x03" /* ctrl */
    HELLO_WORLD_MSG
    "\x57" /* LSB of FCS-32 */
    "\x89" /* 2nd LSB of FCS-32 */
    "\xf2" /* 2nd MSB of FCS-32 */
    "\xfa" /* MSB of FCS-32 */
    FLAGSTR
    /* clang-format on */
    ;

/* contexts */
static void get_ctx(struct hdlc_context *ctx, bool fcs32)
{
	DECLARE_HDLC_CONTEXT(tmp, fcs32, rx, &rx_buf, tx, &tx_buf, erase, &rx_buf);

	memset(&rx_buf, 0, sizeof(rx_buf));
	memset(&tx_buf, 0, sizeof(tx_buf));
    memcpy(ctx, &tmp, sizeof(tmp));
}

void get_ctx16(struct hdlc_context *ctx)
{
    get_ctx(ctx, false);
}

void get_ctx32(struct hdlc_context *ctx)
{
    get_ctx(ctx, true);
}

/* callbacks for contexts */
int rx(void *arg, uint8_t byte)
{
    struct buf *buf;

    zassert_equal(&rx_buf, arg, "expected: %p actual: %p", &rx_buf, arg);
    buf = (struct buf *)arg;

    if (buf->offs >= sizeof(buf->data)) {
	    return -ENOSPC;
    }

    buf->data[buf->offs++] = byte;

    return 0;
}

int tx(void *arg, uint8_t byte)
{
    struct buf *buf;

    zassert_equal(&tx_buf, arg, "expected: %p actual: %p", &tx_buf, arg);
    buf = (struct buf *)arg;

    if (buf->offs >= sizeof(buf->data)) {
	return -ENOSPC;
    }

    buf->data[buf->offs++] = byte;
    return 0;
}

int erase(void *arg, uint16_t n)
{
    struct buf *buf;

    zassert_equal(&rx_buf, arg, "expected: %p actual: %p", &rx_buf, arg);
    buf = (struct buf *)arg;

    if (n > buf->offs) {
	    return -E2BIG;
    }

    buf->offs -= n;
    return 0;
}
