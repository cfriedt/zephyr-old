#ifndef DRIVERS_USB_DEVICE_USB_DC_EMU_H_
#define DRIVERS_USB_DEVICE_USB_DC_EMU_H_

#include <stdbool.h>
#include <stdint.h>

#include <spinlock.h>
#include <sys/ring_buffer.h>
#include <usb/usb_device.h>

enum usb_dc_emu_xport {
	USB_DC_EMU_XPORT_UART,
};

struct usb_dc_emu_api {
	int (*usb_rx_packet)(const struct device *dev, uint8_t ep, uint8_t *data, uint16_t len);
};

struct usb_dc_emu_config {
	uint8_t num_in_eps;
	uint8_t num_out_eps;
	enum usb_dc_emu_xport xport;
	char *xport_dev_name;
};

struct usb_dc_emu_ep_data {
	bool enabled;
	bool stalled;
	bool halted;
	struct usb_dc_ep_cfg_data cfg;
	usb_dc_ep_callback ep_callback;
};

struct usb_dc_emu_data {
	bool attached;
	uint8_t addr;
	/* 0 := OUT, 1 := IN */
	struct usb_dc_emu_ep_data *ep_data[2];
	usb_dc_status_callback status_callback;

	/* Transport (xport) fields */
	const struct device *xport_dev;

	/* Transmit a usb packet */
	int (*xport_tx_packet)(const struct device *dev, uint8_t ep, const uint8_t *usb_data, uint16_t usb_data_len);

	/* RX Interrupt buffer */
	struct k_spinlock xport_rb_lock;
	struct ring_buf xport_rb;
	uint8_t xport_rb_buf[32];
	/* Back-reference for k_work */
	const struct device *dev;

	/* HDLC fields */
	struct k_work hdlc_work;
	uint8_t xport_state;
	uint16_t xport_buf_offs;
	uint8_t xport_buf[256];

	/* TX Statistics */
	uint16_t xport_num_tx;

	/* RX Statistics */
	uint16_t xport_num_rx;
	uint16_t xport_overflow;
	uint16_t xport_isr_overflow;
	uint16_t xport_err_hdlc_min_size;
	uint16_t xport_err_usb_min_size;
	uint16_t xport_fcs_mismatch;
	uint16_t xport_err_hdlc_addr;
	uint16_t xport_err_hdlc_control;
};

struct usb_dc_emu_rx_fifo_entry {
	sys_snode_t snode;
	const struct device *dev;
	uint8_t ep;
	uint16_t data_len;
	uint8_t data[0];
};

int usb_dc_emu_uart_init(const struct device *dev);

#endif /* DRIVERS_USB_DEVICE_USB_DC_EMU_H_ */
