#define DT_DRV_COMPAT zephyr_usb_dc_emu

#include <stdint.h>
#include <stdlib.h>

#include <device.h>
#include <zephyr.h>
#include <usb/usb_device.h>
#include <usb/usbstruct.h>

#include "usb_dc_emu.h"

//#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(usb_dc_emu);

#define DBG_EP(ep) LOG_DBG("ep: %02x %s", USB_EP_GET_IDX(ep), USB_EP_DIR_IS_IN(ep) ? "IN" : "OUT")

static const struct usb_dc_emu_config *drv_get_config();
static struct usb_dc_emu_data *drv_get_data();

static void usb_dc_emu_rx_fifo_work_fn(struct k_work *work);

static K_FIFO_DEFINE(usb_dc_emu_rx_fifo);
static K_WORK_DEFINE(usb_dc_emu_rx_fifo_work, usb_dc_emu_rx_fifo_work_fn);

static void describe_usb_dc_ep_cfg_data(const struct usb_dc_ep_cfg_data * const cfg)
{
	const char *type_str = "";
	switch(cfg->ep_type) {
	case USB_DC_EP_CONTROL: type_str = "CONTROL"; break;
	case USB_DC_EP_ISOCHRONOUS: type_str = "ISOCHRONOUS"; break;
	case USB_DC_EP_BULK: type_str = "BULK"; break;
	case USB_DC_EP_INTERRUPT: type_str = "INTERRUPT"; break;
	}
	LOG_DBG("ep_addr: %02x %s ep_mps: %u ep_type: %s",
		USB_EP_GET_IDX(cfg->ep_addr),
		USB_EP_DIR_IS_IN(cfg->ep_addr) ? "IN" : "OUT",
		cfg->ep_mps, type_str);
}

int usb_dc_attach(void)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	LOG_DBG("");
	data->attached = true;

	return 0;
}

int usb_dc_detach(void)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	LOG_DBG("");
	data->attached = false;

	return 0;
}

int usb_dc_reset(void)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	LOG_DBG("");
	data->status_callback(USB_DC_RESET, NULL);

	return 0;
}

int usb_dc_set_address(const uint8_t addr)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	LOG_DBG("addr: %02x", addr);

	data->addr = addr;

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	LOG_DBG("cb: %p", cb);

	data->status_callback = cb;
}

static bool usb_dc_ep_valid(uint8_t ep)
{
	const struct usb_dc_emu_config *const config = drv_get_config();

	uint8_t idx = USB_EP_GET_IDX(ep);
	bool in = USB_EP_DIR_IS_IN(ep);
	uint8_t num_eps = in ? config->num_in_eps : config->num_out_eps;

	if (idx >= num_eps) {
		LOG_ERR("ep %02x %s is >= max allowed %u", idx, USB_EP_DIR_IS_IN(ep) ? "IN" : "OUT", num_eps);
		return false;
	}

	return true;
}

static bool usb_dc_ep_enabled(uint8_t ep)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	uint8_t idx = USB_EP_GET_IDX(ep);
	bool in = USB_EP_DIR_IS_IN(ep);

	if (!usb_dc_ep_valid(ep)) {
		return false;
	}

	return data->ep_data[in][idx].enabled;
}

static inline int usb_dc_ep_check(const struct usb_dc_ep_cfg_data * const cfg)
{
	return usb_dc_ep_valid(cfg->ep_addr) ? 0 : -EINVAL;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data * const cfg)
{
	LOG_DBG("");
	describe_usb_dc_ep_cfg_data(cfg);
	return usb_dc_ep_check(cfg);
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const cfg)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	uint8_t ep = cfg->ep_addr;

	LOG_DBG("");

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	describe_usb_dc_ep_cfg_data(cfg);

	data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].cfg = *cfg;
	//data->status_callback(USB_DC_CONFIGURED, (uint8_t *)cfg);

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	DBG_EP(ep);

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].stalled = true;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	DBG_EP(ep);

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].stalled = false;

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	DBG_EP(ep);

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	*stalled = data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].stalled;

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	DBG_EP(ep);

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].halted = true;

	return 0;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	DBG_EP(ep);

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].enabled = true;

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	DBG_EP(ep);

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].enabled = false;

	return 0;
}

int usb_dc_ep_flush(const uint8_t ep)
{
	LOG_DBG("ep: %02x %s", USB_EP_GET_IDX(ep), USB_EP_DIR_IS_IN(ep) ? "IN" : "OUT");
	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
		    const uint32_t data_len, uint32_t * const ret_bytes)
{
	LOG_DBG("ep: %02x data: %p data_len: %u ret_bytes: %p",
		USB_EP_GET_IDX(ep), data, data_len, ret_bytes);
	return -ENOSYS;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data,
		   const uint32_t max_data_len, uint32_t *const read_bytes)
{
	LOG_DBG("ep: %02x data: %p max_data_len: %u ret_bytes: %p",
		USB_EP_GET_IDX(ep), data, max_data_len, read_bytes);
	return -ENOSYS;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	LOG_DBG("ep: %02x %s cb: %p", USB_EP_GET_IDX(ep), USB_EP_DIR_IS_IN(ep) ? "IN" : "OUT", cb);

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].ep_callback = cb;

	return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len,
			uint32_t *read_bytes)
{
	LOG_DBG("ep: %02x data: %p max_data_len: %u",
		USB_EP_GET_IDX(ep), data, max_data_len);
	return -ENOSYS;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	LOG_DBG("ep: %02x", USB_EP_GET_IDX(ep));
	return -ENOSYS;
}

int usb_dc_ep_mps(uint8_t ep)
{
	struct usb_dc_emu_data *const data = drv_get_data();

	DBG_EP(ep);

	if (!usb_dc_ep_valid(ep)) {
		return -EINVAL;
	}

	return data->ep_data[USB_EP_DIR_IS_IN(ep)][USB_EP_GET_IDX(ep)].cfg.ep_mps;
}

int usb_dc_wakeup_request(void)
{
	LOG_DBG("");
	return 0;
}

static void usb_dc_emu_rx_fifo_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	bool in;
	uint8_t idx;
	struct usb_dc_emu_rx_fifo_entry *ent;
	const struct device *dev;
	struct usb_dc_emu_data *data;

	for (ent = (struct usb_dc_emu_rx_fifo_entry *) k_fifo_get(&usb_dc_emu_rx_fifo, K_NO_WAIT);
		ent != NULL; ent = (struct usb_dc_emu_rx_fifo_entry *) k_fifo_get(&usb_dc_emu_rx_fifo, K_NO_WAIT)) {

		idx = USB_EP_GET_IDX(ent->ep);
		in = USB_EP_DIR_IS_IN(ent->ep);

		data = ent->dev->data;
		//data->ep_data[in][idx].ep_callback(ent->ep, usb_dc_ep_cb_status_code cb_status);

		free(ent);
	}
}

int usb_dc_emu_usb_rx_packet(const struct device *dev, uint8_t ep, uint8_t *usb_data, uint16_t usb_data_len)
{
	struct usb_dc_emu_rx_fifo_entry *ent;

	if (!usb_dc_ep_enabled(ep)) {
		LOG_ERR("USB ep %02x %s is not enabled", USB_EP_GET_IDX(ep), USB_EP_DIR_IS_IN(ep) ? "IN" : "OUT");
		return -EINVAL;
	}

	ent = malloc(sizeof(*ent) + usb_data_len);
	if (NULL == ent) {
		LOG_ERR("failed to allocate %u bytes", sizeof(*ent) + usb_data_len);
		return -ENOMEM;
	}

	ent->dev = dev;
	ent->ep = ep;
	ent->data_len = usb_data_len;
	memcpy(ent->data, usb_data, usb_data_len);

	k_fifo_put(&usb_dc_emu_rx_fifo, ent);

	k_work_submit(&usb_dc_emu_rx_fifo_work);

	return 0;
}

static const struct usb_dc_emu_api usb_dc_emu_api = {
	.usb_rx_packet = usb_dc_emu_usb_rx_packet,
};

static int usb_dc_emu_init(const struct device *dev)
{
	const struct usb_dc_emu_config *const config = dev->config;
	struct usb_dc_emu_data *const data = dev->data;

	LOG_DBG("");

	data->dev = dev;

	switch(config->xport) {
	case USB_DC_EMU_XPORT_UART:
		return usb_dc_emu_uart_init(dev);

	default:
		break;
	}

	return -EINVAL;
}

#define DEFINE_USB_DC_EMU(_num)						\
	BUILD_ASSERT(_num == 0, "only instance 0 is currently supported"); \
	\
	static const struct usb_dc_emu_config usb_dc_emu_config_##_num = {	\
		.num_in_eps = DT_PROP(DT_DRV_INST(_num), num_in_endpoints), \
		.num_out_eps = DT_PROP(DT_DRV_INST(_num), num_out_endpoints), \
		.xport_dev_name = DT_LABEL(DT_PHANDLE(DT_DRV_INST(_num), xport_dev)), \
	};								\
									\
	static struct usb_dc_emu_ep_data usb_dc_emu_ep_data_out_##_num[DT_PROP(DT_DRV_INST(_num), num_out_endpoints)]; \
	static struct usb_dc_emu_ep_data usb_dc_emu_ep_data_in_##_num[DT_PROP(DT_DRV_INST(_num), num_in_endpoints)]; \
	\
	static struct usb_dc_emu_data usb_dc_emu_data_##_num = {		\
		.ep_data = { \
			usb_dc_emu_ep_data_out_##_num, \
			usb_dc_emu_ep_data_in_##_num, \
		}, \
	};								\
									\
	DEVICE_AND_API_INIT(usb_dc_emu_##_num, DT_INST_LABEL(_num),	\
			    usb_dc_emu_init, &usb_dc_emu_data_##_num,		\
			    &usb_dc_emu_config_##_num, POST_KERNEL,	\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    &usb_dc_emu_api)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_USB_DC_EMU);

static const struct usb_dc_emu_config *drv_get_config() {
	return &usb_dc_emu_config_0;
}

static struct usb_dc_emu_data *drv_get_data() {
	return &usb_dc_emu_data_0;
}
