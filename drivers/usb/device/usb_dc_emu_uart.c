#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <sys/util.h>
#include <drivers/usb/usb_dc.h>

#include <zephyr.h>
#include "../../../include/net/hdlc.h"

#include "usb_dc_native_posix_adapt.h"

//#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(native_posix_adapt_uart);

enum {
	/* TODO: should probably be a configurable */
	HDLC_MAX_FRAME_SIZE = 256,
	HDLC_FIFO_NUM_FRAMES = 4,

	USB_MIN_SIZE = 8,

	/* Unnumbered Information frames */
	/* See https://en.wikipedia.org/wiki/High-Level_Data_Link_Control#Unnumbered_frames */
	HDLC_UI = 0x03,
	HDLC_UI_MASK = ~0x10,

	/* 1 address byte, 1 control byte, and 2 fcs bytes */
	HDLC_MIN_SIZE = 4,

	/* The address byte contains the USB endpoint (index only).
	* The USB direction bit is inferred from I/O.
	* This is primarily to accomodate the address encoding below
	* although the C/R bit is ignored in this case.
	* https://erg.abdn.ac.uk/users/gorry/eg3567/dl-pages/hdlc-address.html
	*/
	HDLC_ADDR_SHIFT = 1,
	HDLC_ADDR_MASK = 1,
};

struct fifo_entry {
        sys_snode_t snode;
        atomic_t in_use;
        uint32_t data_len;
        uint8_t data[HDLC_MAX_FRAME_SIZE];
};

/* Forward Declarations */
static void uart_work_fn(struct k_work *work);
static void hdlc_work_fn(struct k_work *work);

/* UART device and buffer */
static const struct device *dev;
RING_BUF_DECLARE(uart_rb, 2 * HDLC_MAX_FRAME_SIZE);

/* HDLC I/O functions */
static int hdlc_rx_ring_buf(void *arg, uint8_t byte);
static int hdlc_tx_uart(void *arg, uint8_t byte);
static int hdlc_clear_ring_buf(void *arg);

static DECLARE_HDLC_CONTEXT( \
	hdlc_context, \
	false,
	hdlc_rx_ring_buf, \
	&uart_rb, \
	hdlc_tx_uart, \
	&dev, \
	hdlc_clear_ring_buf, \
	&uart_rb \
);

static struct fifo_entry* hdlc_fifo_entry_get(void);
static void hdlc_fifo_entry_put(struct fifo_entry* ent);

static K_WORK_DEFINE(uart_work, uart_work_fn);
static K_WORK_DEFINE(hdlc_work, hdlc_work_fn);

/* FIFO of HDLC frames */
static K_FIFO_DEFINE(hdlc_fifo);
static struct fifo_entry hdlc_fifo_entry[HDLC_FIFO_NUM_FRAMES];

static int hdlc_rx_ring_buf(void *arg, uint8_t byte)
{
	struct ring_buf *const rb = (struct ring_buf *)arg;

	if (rb == NULL) {
		return -EINVAL;
	}

	if (1 != ring_buf_put(rb, (const uint8_t *)&byte, 1)) {
		return -EIO;
	}

	return 0;
}

static int hdlc_clear_ring_buf(void *arg)
{
	struct ring_buf *const rb = (struct ring_buf *) arg;

	if (rb == NULL) {
		LOG_ERR("NULL argument is invalid");
		return -EINVAL;
	}

	ring_buf_reset(rb);
}

static int hdlc_tx_uart(void *arg, uint8_t byte)
{
	struct device **dev = (struct device **)arg;

	if (dev == NULL || *dev == NULL ) {
		return -EINVAL;
	}

	uart_poll_out(*dev, byte);

	return 0;
}

static void hdlc_process_frame(struct hdlc_context *hdlc_context, size_t buf_len)
{
	#if 0	
	uint8_t ep;
	uint8_t idx;
	uint8_t addr;
	uint8_t control;
	uint8_t *usb_data;
	uint16_t usb_data_len;
	uint16_t expected_fcs;
	uint16_t actual_fcs;

	struct usbip_header usbip_header = {};
	struct usbip_submit *req = &usbip_header.u.submit;

	LOG_HEXDUMP_DBG(buf, buf_len, "> ");

	if (buf_len < HDLC_MIN_SIZE) {
		LOG_ERR("Dropped HDLC frame that is too small (only %u bytes)", buf_len);
		return;
	}

	expected_fcs =
		0
		| (buf[buf_len - 2] << 0)
		| (buf[buf_len - 1] << 8)
		;
	expected_fcs ^= 0xffff;
	actual_fcs = crc16_ccitt(0xffff, buf, buf_len - 2);

	if (actual_fcs != expected_fcs) {
		LOG_ERR("Dropped HDLC frame with FCS mismatch: expected: %04x actual: %04x", expected_fcs, actual_fcs);
		return;
	}

	addr = buf[0];
	if (0 == (HDLC_ADDR_MASK & addr)) {
		LOG_ERR("Dropped HDLC frame with unexpected HDLC address byte: 0x%02x", addr);
		return;
	}

	control = buf[1];
	if (HDLC_UI != (control & HDLC_UI_MASK) ) {
		LOG_ERR("Dropped HDLC frame with unexpected HDLC control byte: 0x%02x", control);
		return;
	}

	usb_data = &buf[2];
	/* subtract 1 byte for addr, 1 byte for control, and 2 bytes for fcs */
	usb_data_len = buf_len - 4;

	if (usb_data_len < USB_MIN_SIZE) {
		LOG_ERR("Dropped USB packet < %u bytes", USB_MIN_SIZE);
		return;
	}

	idx = addr >> HDLC_ADDR_SHIFT;
	ep = USB_EP_DIR_IN | idx;

	usbip_header.common.ep = ep;
	memcpy(req, usb_data, sizeof(*req));

	if (0 == idx) {
		handle_usb_control(&usbip_header);
	} else {
		handle_usb_data(&usbip_header);
	}
	#endif
}

static struct fifo_entry* hdlc_fifo_entry_get(void)
{
	for(size_t i = 0; i < ARRAY_SIZE(hdlc_fifo_entry); ++i) {
		struct fifo_entry* ent = &hdlc_fifo_entry[i];
		if (atomic_cas(&ent->in_use, false, true)) {
			return ent;
		}
	}

	return NULL;
}

static void hdlc_fifo_entry_put(struct fifo_entry* ent)
{
	if (ent == NULL) {
		return;
	}

	__ASSERT(true == atomic_get(&ent->in_use),
		"attempt to put fifo entry that is not in use");

	ent->data_len = 0;
	memset(ent->data, 0, sizeof(ent->data));

	atomic_set(&ent->in_use, false);
}

static void hdlc_work_fn(struct k_work *work)
{
	for(;!k_fifo_is_empty(&hdlc_fifo);) {
		struct fifo_entry *ent = k_fifo_get(&hdlc_fifo, K_NO_WAIT);
		//hdlc_process_frame(ent->data, ent->data_len);
		LOG_DBG("Processing entry");
		hdlc_fifo_entry_put(ent);
	}
}

static void write_uart(const struct device *dev, const uint8_t *data, size_t len)
{
	size_t i;
	for(i = 0; i < len; ++i, ++data) {
		uart_poll_out(dev, *data);
	}
}

static void uart_isr(const struct device *uart_dev, void *user_data)
{
	int r;
	uint8_t byte;
	uint8_t ovflw;
	size_t count = 0;

	while (uart_irq_update(uart_dev) &&
	       uart_irq_is_pending(uart_dev)) {

		if (!uart_irq_rx_ready(uart_dev)) {
			continue;
		}

		r = uart_fifo_read(dev, &byte, 1);
		if (r < 0) {
			LOG_ERR("uart_fifo_read() failed (%d)", r);
			uart_irq_rx_disable(dev);
			return;
		}

		if (0 == ring_buf_space_get(&uart_rb)) {
			r = ring_buf_get(&uart_rb, &ovflw, 1);
			if (r != 1) {
				LOG_ERR("failed to remove head of ring buffer");
				uart_irq_rx_disable(dev);
				return;
			}
			LOG_ERR("overflow occurred");
		}

		if (1 != ring_buf_put(&uart_rb, &byte, 1)) {
			LOG_ERR("ring_buf_put() failed");
			uart_irq_rx_disable(dev);
			return;
		}
	}

	if (count > 0) {
		k_work_submit(&uart_work);
	}
}

void usbip_start(void)
{
	const char *dev_name = "UART_1";
	dev = device_get_binding(dev_name);

	uint8_t c;

	LOG_DBG("");

	if (dev == NULL) {
		LOG_ERR("failed to open '%s'", dev_name);
		return;
	}

	uart_irq_rx_disable(dev);
	uart_irq_tx_disable(dev);
	uart_irq_callback_user_data_set(dev, uart_isr, (void *)dev);
	while (uart_irq_rx_ready(dev)) {
			uart_fifo_read(dev, &c, 1);
	}
	uart_irq_rx_enable(dev);
}

int usbip_recv(uint8_t *buf, size_t len)
{
#if 0
	return recv(connfd_global, hdlc_rb, len, 0);
#else
	LOG_DBG("");
	return -ENOSYS;
#endif
}

int usbip_send(uint8_t ep, const uint8_t *data, size_t len)
{
	uint8_t addr = (USB_EP_GET_IDX(ep) << HDLC_ADDR_SHIFT) | HDLC_ADDR_MASK;
	uint8_t control = HDLC_UI;

	LOG_DBG("");

	hdlc_write(&hdlc_context, &addr, 1, &control, 1, data, len);

	return len;
}

bool usbip_send_common(uint8_t ep, uint32_t data_len)
{
	ARG_UNUSED(ep);
	ARG_UNUSED(data_len);

	/* AFAICT, this is really just used for a usbip specific header */
	LOG_DBG("");

	return true;
}

bool usbip_skip_setup(void)
{
	uint64_t setup;

	LOG_DBG("Skip 8 bytes");

	if (usbip_recv((void *)&setup, sizeof(setup)) != sizeof(setup)) {
		return false;
	}

	return true;
}
