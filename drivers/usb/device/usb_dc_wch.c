
#define DT_DRV_COMPAT wch_wch_usbhd

#include <soc.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/usb/usb_dc.h>

#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_dc_wch);

//#define DUAL_BUFFER

#include "usb_dc_wch.h"

#define USB_REG_BASE    DT_INST_REG_ADDR(0)
#define USB_IRQ			DT_INST_IRQ(0, irq)
#define USB_IRQ_PRI		DT_INST_IRQ(0, priority)
#define USB_NUM_BIDIR_ENDPOINTS	DT_INST_PROP(0, num_bidir_endpoints)

#define EP0_MPS 64U
#define EP_MPS 64U

#define EP0_IDX 0
#define EP0_IN (EP0_IDX | USB_EP_DIR_IN)
#define EP0_OUT (EP0_IDX | USB_EP_DIR_OUT)

struct usb_dc_wch_ep_state {
	uint16_t ep_mps;		/** Endpoint max packet size */
	uint8_t ep_type;		/** Endpoint type */
    /** Endpoint flags */
	uint8_t ep_stalled : 1;
    uint8_t ep_enabled : 1;
    uint8_t ep_data1 : 1;
	usb_dc_ep_callback cb;	/** Endpoint callback function */
	uint32_t read_count;	/** Number of bytes in read buffer  */
	uint32_t read_offset;	/** Current offset in read buffer */
	struct k_sem write_sem;	/** Write boolean semaphore */
};

struct usb_dc_wch_state {
    /* 4 byte aligned */
    union {
        struct {
            uint8_t ep0_buf[EP_MPS];
            uint8_t ep4_rx_buf[EP_MPS];
            uint8_t ep4_tx_buf[EP_MPS];
        };
	    uint8_t ep_0_4_buf[EP_MPS * 3];
    };
#ifdef DUAL_BUFFER
    union {
        struct {
            uint8_t rx_buf0[EP_MPS];
            uint8_t rx_buf1[EP_MPS];
            uint8_t tx_buf0[EP_MPS];
            uint8_t tx_buf1[EP_MPS];
        };
        uint8_t ep_buf[EP_MPS * 4];
    } ep123_buf[3];
#else
    union {
        struct {
            uint8_t rx_buf0[EP_MPS];
            uint8_t tx_buf0[EP_MPS];
        };
        uint8_t ep_buf[EP_MPS * 2];
    } ep123_buf[3];
#endif
    volatile struct wch_usb *usb;
    uint8_t address;
	uint8_t attached : 1;
	uint8_t update_address :1;	/* set address after IN transaction */
	usb_dc_status_callback status_cb; /* Status callback */
	struct usb_dc_wch_ep_state out_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	struct usb_dc_wch_ep_state in_ep_state[USB_NUM_BIDIR_ENDPOINTS];
} __aligned(4);

static struct usb_dc_wch_state usb_dc_wch_state;

static struct usb_dc_wch_ep_state *usb_dc_wch_get_ep_state(uint8_t ep)
{
	struct usb_dc_wch_ep_state *ep_state_base;

	if (USB_EP_GET_IDX(ep) >= USB_NUM_BIDIR_ENDPOINTS) {
		return NULL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_state_base = usb_dc_wch_state.out_ep_state;
	} else {
		ep_state_base = usb_dc_wch_state.in_ep_state;
	}

	return ep_state_base + USB_EP_GET_IDX(ep);
}

static inline void reenable_all_endpoints(void)
{
	for (uint8_t ep_idx = 0; ep_idx < USB_NUM_BIDIR_ENDPOINTS ; ep_idx++) {
		if (usb_dc_wch_state.out_ep_state[ep_idx].ep_enabled) {
			usb_dc_ep_enable(ep_idx);
		}
		if (usb_dc_wch_state.in_ep_state[ep_idx].ep_enabled) {
			usb_dc_ep_enable(ep_idx | USB_EP_DIR_IN);
		}
	}
}

static inline uint8_t *get_ep_buf(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	switch (ep_idx) {
		case 0:
			return usb_dc_wch_state.ep0_buf;
		case 1:
		case 2:
		case 3:
			if (USB_EP_GET_DIR(ep) == USB_EP_DIR_OUT)
				return usb_dc_wch_state.ep123_buf[ep_idx - 1].rx_buf0;
			else
				return usb_dc_wch_state.ep123_buf[ep_idx - 1].tx_buf0;
		case 4:
			if (USB_EP_GET_DIR(ep) == USB_EP_DIR_OUT)
				return usb_dc_wch_state.ep4_rx_buf;
			else
				return usb_dc_wch_state.ep4_tx_buf;
	}

	LOG_ERR("invalid ep 0x%02x", ep);
	return NULL;
}

static int setup_req_to_host;

static void usb_dc_wch_isr(const void *arg)
{
	ARG_UNUSED(arg);


	if (!usb_dc_wch_state.attached) {
		LOG_ERR("USB interrupt occurred while device is detached");
	}

	while(1) {
		volatile union wch_usb_int_flag int_flag = usb_dc_wch_state.usb->int_flag;
		volatile union wch_usb_int_status int_status = usb_dc_wch_state.usb->int_status;
		volatile union wch_usb_misc_status misc_status = usb_dc_wch_state.usb->misc_status;

		if (int_flag.val & 0x1f) {
			//usb_dc_wch_state.usb->int_flag.val = int_flag.val;
			*(volatile uint32_t *)0x400010a8 &= ~(1UL << 5);
		} else {
			break;
		}

		LOG_DBG("int flag: 0x%02x, int_status: 0x%02x, misc status: 0x%02x", int_flag.val, int_status.val, misc_status.val);
		
		if (int_flag.xmit_done) {
			LOG_DBG("XMIT event");

			if (int_status.token_pid != WCH_USB_TOKEN_NONE) {
				uint8_t ep_idx = int_status.ep_idx;
				switch (int_status.token_pid) {
					case WCH_USB_TOKEN_OUT:
						LOG_DBG("epnum 0x%02x, rx_count %u", ep_idx, usb_dc_wch_state.usb->rx_len);

						if (ep_idx == 0 && setup_req_to_host) {
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_tog = 0;
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_tog = 0;
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_resp = USB_RESP_NAK;
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_resp = USB_RESP_ACK;
						} else {
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_tog = !usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_tog;
						}
						if (ep_idx != 0 && !int_status.toggle_ok) {
							LOG_ERR("Toggle not OK!");
						}
						usb_dc_wch_state.out_ep_state[ep_idx].read_count = usb_dc_wch_state.usb->rx_len;
						usb_dc_wch_state.out_ep_state[ep_idx].read_offset = 0;

						if (usb_dc_wch_state.out_ep_state[ep_idx].cb) {
							usb_dc_wch_state.out_ep_state[ep_idx].cb(ep_idx | USB_EP_DIR_OUT, USB_DC_EP_DATA_OUT);
						}
						break;
					case WCH_USB_TOKEN_IN:
						LOG_DBG("epnum 0x%02x tx", ep_idx);
						k_sem_give(&(usb_dc_wch_state.in_ep_state->write_sem));		/* allow next ep write */

						if (usb_dc_wch_state.update_address) {
							usb_dc_wch_state.usb->adr.addr = usb_dc_wch_state.address;
							usb_dc_wch_state.update_address = 0;
						}

						if (ep_idx == 0 && !setup_req_to_host) {
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_tog = 0;
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_tog = 0;
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_resp = USB_RESP_NAK;
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_resp = USB_RESP_ACK;
						} else {
							usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_tog = !usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_tog;
						}

						if (usb_dc_wch_state.in_ep_state[ep_idx].cb) {
							usb_dc_wch_state.in_ep_state[ep_idx].cb(ep_idx | USB_EP_DIR_IN, USB_DC_EP_DATA_IN);
						}
						break;
					case WCH_USB_TOKEN_SOF:
						usb_dc_wch_state.status_cb(USB_DC_SOF, NULL);
						/* TODO */
						break;
				}
			} else if (int_status.setup_ack) {
				/* setup transaction force datax to data1, don't check toggle ok here */
				usb_dc_wch_state.usb->ep_tx_len_ctrl[0].ep_ctrl.tx_tog = 1;
				usb_dc_wch_state.usb->ep_tx_len_ctrl[0].ep_ctrl.rx_tog = 1;

				setup_req_to_host = *get_ep_buf(USB_CONTROL_EP_OUT) & 0x80;		/* assume little endian */

				usb_dc_wch_state.out_ep_state[0].read_count = 8;
				usb_dc_wch_state.out_ep_state[0].read_offset = 0;
				if (usb_dc_wch_state.out_ep_state[0].cb) {
					usb_dc_wch_state.out_ep_state[0].cb(USB_CONTROL_EP_OUT, USB_DC_EP_SETUP);
				}
			}
		}

		if (int_flag.bus_reset) {
			LOG_DBG("USB RESET event");

			/* Inform upper layers */
			if (usb_dc_wch_state.status_cb) {
				usb_dc_wch_state.status_cb(USB_DC_RESET, NULL);
			}

			/* Clear device address during reset. */
			usb_dc_wch_state.usb->adr.addr = 0;
			usb_dc_wch_state.address = 0;

			reenable_all_endpoints();
		}

#if 0
		if (int_flag.suspended) {
			if (misc_status.suspended) {
				LOG_DBG("USB Suspend event");

				if (usb_dc_wch_state.status_cb) {
					usb_dc_wch_state.status_cb(USB_DC_SUSPEND, NULL);
				}
			} else {
				LOG_DBG("USB Wakeup event");

				if (usb_dc_wch_state.status_cb) {
					usb_dc_wch_state.status_cb(USB_DC_RESUME, NULL);
				}
			}

			/* clear it */
			usb_dc_wch_state.usb->int_flag.suspended = 1;
		}
#endif

#if 0
		if (int_flag.sof) {
			usb_dc_wch_state.usb->int_flag.sof = 1;

			usb_dc_wch_state.status_cb(USB_DC_SOF, NULL);
		}
#endif

		/* host sof, host nak, fifo ov */

		usb_dc_wch_state.usb->int_flag.val = int_flag.val;
		*(volatile uint32_t *)0x400010a8 |= (1UL << 5);
	}

}

int usb_dc_attach(void)
{
	LOG_DBG("try to attach");
    usb_dc_wch_state.usb->adr.val = 0;

	usb_dc_wch_state.usb->dev_ctrl.port_en = 1;

	usb_dc_wch_state.attached = 1;
    //pull up dp, then interrupts is coming
    usb_dc_wch_state.usb->ctrl.sys_ctrl =  WCH_USB_SYSCTL_ENABLE_USB_AND_DP_PU;

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	ep_state->cb = cb;

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	LOG_DBG("set status_cb");

	usb_dc_wch_state.status_cb = cb;
}

int usb_dc_set_address(const uint8_t addr)
{
	LOG_DBG("addr %u (0x%02x)", addr, addr);

	usb_dc_wch_state.address = addr;
	usb_dc_wch_state.update_address = 1;

    return 0;
}

#if 0
static int usb_dc_ep_start_read(uint8_t ep, uint8_t *data, uint32_t max_data_len)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	LOG_DBG("ep 0x%02x, len %u", ep, max_data_len);

	/* we flush EP0_IN by doing a 0 length receive on it */
	if (!USB_EP_DIR_IS_OUT(ep) && (ep != EP0_IN || max_data_len)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_resp = USB_RESP_NAK;

	return 0;
}
#endif

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data * const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
		LOG_ERR("invalid endpoint configuration");
		return -1;
	}

	if (cfg->ep_mps > EP_MPS) {
		LOG_WRN("unsupported packet size");
		return -1;
	}

	if (ep_idx > (USB_NUM_BIDIR_ENDPOINTS - 1)) {
		LOG_ERR("endpoint index/address out of range");
		return -1;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const ep_cfg)
{
	uint8_t ep = ep_cfg->ep_addr;
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	if (!ep_state)
		return -EINVAL;

	LOG_DBG("ep 0x%02x, previous ep_mps %u, ep_mps %u, ep_type %u",
		ep_cfg->ep_addr, ep_state->ep_mps, ep_cfg->ep_mps,
		ep_cfg->ep_type);

    if (ep_cfg->ep_mps > EP_MPS)
		return -EINVAL;

	ep_state->ep_mps = ep_cfg->ep_mps;

	switch (ep_cfg->ep_type) {
	case USB_DC_EP_CONTROL:
		ep_state->ep_type = WCH_EP_TYPE_CTRL;
		break;
	case USB_DC_EP_ISOCHRONOUS:
		ep_state->ep_type = WCH_EP_TYPE_ISOC;
		break;
	case USB_DC_EP_BULK:
		ep_state->ep_type = WCH_EP_TYPE_BULK;
		break;
	case USB_DC_EP_INTERRUPT:
		ep_state->ep_type = WCH_EP_TYPE_INTR;
		break;
	default:
		return -EINVAL;
	}

    return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

    int ep_idx = USB_EP_GET_IDX(ep);

	if (USB_EP_DIR_IS_OUT(ep)) {
        usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_resp = USB_RESP_STALL;
    } else {
        usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_resp = USB_RESP_STALL;
    }

	ep_state->ep_stalled = 1U;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

    int ep_idx = USB_EP_GET_IDX(ep);

	if (USB_EP_DIR_IS_OUT(ep)) {
        usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_resp = USB_RESP_ACK;
    } else {
        usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_resp = USB_RESP_NAK;
    }

	ep_state->ep_stalled = 0U;
	ep_state->read_count = 0U;

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state || !stalled) {
		return -EINVAL;
	}

	*stalled = ep_state->ep_stalled;

	return 0;

}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("enable ep (0x%02x, %u, %u)", ep, ep_state->ep_mps, ep_state->ep_type);

    int ep_idx = USB_EP_GET_IDX(ep);

	/* all eps are always enabled. just tweak the repsonse here */

	if (USB_EP_DIR_IS_OUT(ep)) {
		usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_resp = USB_RESP_ACK;
        usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_tog = 0;
    } else {
		if (ep_state->ep_type == WCH_EP_TYPE_ISOC) {
			usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_resp = USB_RESP_NO_RESP;
		} else {
			usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_resp = USB_RESP_NAK;
		}
        usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_tog = 0;
    }

	ep_state->ep_enabled = 1;
	/* clear stall, and reset to data0 */
	ep_state->ep_stalled = 0;
	ep_state->ep_data1 = 0;

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("enable ep (0x%02x, %u, %u)", ep, ep_state->ep_mps, ep_state->ep_type);

    int ep_idx = USB_EP_GET_IDX(ep);

	/* all eps are always enabled, so we just let them response NAK/Stall? */
	if (USB_EP_DIR_IS_OUT(ep)) {
		usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_resp = USB_RESP_NAK;
    } else {
		usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_resp = USB_RESP_NAK;
    }

	ep_state->ep_enabled = 0;

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
		    const uint32_t data_len, uint32_t * const ret_bytes)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);
	uint8_t *buf = get_ep_buf(ep);
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	int len = data_len;
	int ret = 0;

	LOG_DBG("ep 0x%02x, len %u", ep, len);

	if (!ep_state || !USB_EP_DIR_IS_IN(ep)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	ret = k_sem_take(&ep_state->write_sem, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Unable to get write lock (%d)", ret);
		return -EAGAIN;
	}

	if (!k_is_in_isr()) {
		irq_disable(USB_IRQ);
	}

	if (ep == EP0_IN && len > USB_MAX_CTRL_MPS) {
		len = USB_MAX_CTRL_MPS;
	}

	memcpy(buf, data, len);
	usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].tx_len = len;
	usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.tx_resp = USB_RESP_ACK;

#if 0	// TODO
	if (!ret && ep == EP0_IN && len > 0) {
		/* Wait for an empty package as from the host.
		 * This also flushes the TX FIFO to the host.
		 */
		usb_dc_ep_start_read(ep, NULL, 0);
	}
#endif

	if (!k_is_in_isr()) {
		irq_enable(USB_IRQ);
	}

	if (!ret && ret_bytes) {
		*ret_bytes = len;
	}

	return ret;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len,
			uint32_t *read_bytes)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);
	uint32_t read_count;

	if (!ep_state) {
		LOG_ERR("Invalid Endpoint %x", ep);
		return -EINVAL;
	}

	read_count = ep_state->read_count;

	LOG_DBG("ep 0x%02x, %u bytes, %u+%u, %p", ep, max_data_len,
		ep_state->read_offset, read_count, data);

	if (!USB_EP_DIR_IS_OUT(ep)) {
		LOG_ERR("Wrong endpoint direction: 0x%02x", ep);
		return -EINVAL;
	}

    if (!ep_state->ep_enabled) {
		LOG_ERR("Not enabled endpoint");
		return -EINVAL;
    }

	/* When both buffer and max data to read are zero, just ignore reading
	 * and return available data in buffer. Otherwise, return data
	 * previously stored in the buffer.
	 */
	if (data) {
		read_count = read_count < max_data_len ? read_count : max_data_len;
		memcpy(data, get_ep_buf(ep) + ep_state->read_offset, read_count);
		ep_state->read_count -= read_count;
		ep_state->read_offset += read_count;
	} else if (max_data_len) {
		LOG_ERR("Wrong arguments");
	}

	if (read_bytes) {
		*read_bytes = read_count;
	}

	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	usb_dc_wch_state.usb->ep_tx_len_ctrl[ep_idx].ep_ctrl.rx_resp = USB_RESP_ACK;

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t * const read_bytes)
{
	if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
		return -EINVAL;
	}

	if (usb_dc_ep_read_continue(ep) != 0) {
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_flush(const uint8_t ep)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	struct usb_dc_wch_ep_state *ep_state = usb_dc_wch_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	return ep_state->ep_mps;
}

int usb_dc_detach(void)
{
    usb_dc_wch_state.usb->adr.val = 0;

	usb_dc_wch_state.attached = 0;
    //pull dp down, while leave usb enabled
    usb_dc_wch_state.usb->ctrl.sys_ctrl =  WCH_USB_SYSCTL_ENABLE_USB;

	usb_dc_wch_state.usb->dev_ctrl.port_en = 0;

	return 0;
}

int usb_dc_reset(void)
{
	LOG_ERR("Not implemented");

	return 0;
}

/* device initcall before device stack init */
static int usb_dc_wch_init(const struct device *dev)
{
	int i;

	ARG_UNUSED(dev);
	LOG_DBG("init wch usb device controller");
	printk("init wch usb device controller\n");

    /* no clock cfg */

    /* TODO pinctrl, hardcoded in syscon */

    usb_dc_wch_state.usb = (struct wch_usb *)USB_REG_BASE;

	/* reset and deassert */
	union wch_usb_ctrl wch_usb_ctrl = {0};
	wch_usb_ctrl.clr_fifo = 1;
	wch_usb_ctrl.reset_sie = 1;
	usb_dc_wch_state.usb->ctrl = wch_usb_ctrl;

	wch_usb_ctrl.clr_fifo = 0;
	wch_usb_ctrl.reset_sie = 0;
	usb_dc_wch_state.usb->ctrl = wch_usb_ctrl;

	/* For a pair of IN/OUT eps, disabling OUT ep changes dma buf addr of IN
	 * ep, and they may be used in different interfaces and decoupled modules,
	 * so we always enable all eps */
	union wch_usb_ep4_1_mod wch_usb_ep4_1_mod = {0};
	wch_usb_ep4_1_mod.ep4_tx_en = 1;
	wch_usb_ep4_1_mod.ep4_rx_en = 1;
#ifdef DUAL_BUFFER
	wch_usb_ep4_1_mod.ep1_dual_buf = 1;
#else
	wch_usb_ep4_1_mod.ep1_dual_buf = 0;
#endif
	wch_usb_ep4_1_mod.ep1_tx_en = 1;
	wch_usb_ep4_1_mod.ep1_rx_en = 1;
	usb_dc_wch_state.usb->ep4_1_mod = wch_usb_ep4_1_mod;

	union wch_usb_ep2_3_mod wch_usb_ep2_3_mod = {0};
	wch_usb_ep2_3_mod.ep2_tx_en = 1;
	wch_usb_ep2_3_mod.ep2_rx_en = 1;
	wch_usb_ep2_3_mod.ep3_tx_en = 1;
	wch_usb_ep2_3_mod.ep3_rx_en = 1;
#ifdef DUAL_BUFFER
	wch_usb_ep2_3_mod.ep2_dual_buf = 1;
	wch_usb_ep2_3_mod.ep3_dual_buf = 1;
#else
	wch_usb_ep2_3_mod.ep2_dual_buf = 0;
	wch_usb_ep2_3_mod.ep3_dual_buf = 0;
#endif
	usb_dc_wch_state.usb->ep2_3_mod = wch_usb_ep2_3_mod;

	/* set up all endpoints' dma buffers */
	usb_dc_wch_state.usb->ep_dma_addr[0].val = (uint16_t)(uint32_t)&(usb_dc_wch_state.ep_0_4_buf[0]);
	usb_dc_wch_state.usb->ep_dma_addr[1].val = (uint16_t)(uint32_t)&(usb_dc_wch_state.ep123_buf[0]);
	usb_dc_wch_state.usb->ep_dma_addr[2].val = (uint16_t)(uint32_t)&(usb_dc_wch_state.ep123_buf[1]);
	usb_dc_wch_state.usb->ep_dma_addr[3].val = (uint16_t)(uint32_t)&(usb_dc_wch_state.ep123_buf[2]);

	for (i = 0; i < 5; i++) {
		struct wch_usb_ep_tx_len_ctrl wch_usb_ep_tx_len_ctrl = {0};
		wch_usb_ep_tx_len_ctrl.ep_ctrl.tx_resp = USB_RESP_NAK;
		wch_usb_ep_tx_len_ctrl.ep_ctrl.rx_resp = USB_RESP_NAK;
#ifdef DUAL_BUFFER
		wch_usb_ep_tx_len_ctrl.ep_ctrl.auto_toggle = 1;		/* TODO, auto toggling is not associated with DUAL BUFFER */
#else
		wch_usb_ep_tx_len_ctrl.ep_ctrl.auto_toggle = 0;
#endif
		wch_usb_ep_tx_len_ctrl.ep_ctrl.tx_tog = 0;
		wch_usb_ep_tx_len_ctrl.ep_ctrl.rx_tog = 0;
		usb_dc_wch_state.usb->ep_tx_len_ctrl[i] = wch_usb_ep_tx_len_ctrl;
	}

    usb_dc_wch_state.usb->adr.val = 0;

	/* enable usb, but don't attach */
    wch_usb_ctrl.int_busy = 1;	/* resp NAK after entering interrupt. How about set resp ACK in ISR */
    wch_usb_ctrl.sys_ctrl = WCH_USB_SYSCTL_ENABLE_USB;
    wch_usb_ctrl.dma_en = 1;
	usb_dc_wch_state.usb->ctrl = wch_usb_ctrl;

	/* clear stale interrupts */
    usb_dc_wch_state.usb->int_flag.val = 0xff;

	union wch_usb_dev_ctrl wch_usb_dev_ctrl = {0};
    wch_usb_dev_ctrl.port_en = 0;
    wch_usb_dev_ctrl.low_speed = 0;
    wch_usb_dev_ctrl.disable_pd = 1;
    usb_dc_wch_state.usb->dev_ctrl = wch_usb_dev_ctrl;

	union wch_usb_int_en wch_usb_int_en = {0};
    wch_usb_int_en.bus_reset = 1;
    wch_usb_int_en.xmit_done = 1;
    //wch_usb_int_en.suspend = 1;
    wch_usb_int_en.fifo_ov = 1;
    wch_usb_int_en.nak = 1;
#ifdef CONFIG_USB_DEVICE_SOF
    wch_usb_int_en.sof = 1;
#endif
	usb_dc_wch_state.usb->int_en = wch_usb_int_en;

	/* use the semaphore to control refilling of IN buf */
	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		k_sem_init(&usb_dc_wch_state.in_ep_state[i].write_sem, 1, 1);
	}

	/* clear stale interrupts */
    usb_dc_wch_state.usb->int_flag.val = 0xff;

	IRQ_CONNECT(USB_IRQ, USB_IRQ_PRI, usb_dc_wch_isr, 0, 0);
	irq_enable(USB_IRQ);

	return 0;
}

SYS_INIT(usb_dc_wch_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
