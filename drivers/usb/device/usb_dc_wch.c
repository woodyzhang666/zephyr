
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

__ramfunc static void usb_dc_wch_isr(const void *arg)
{
	ARG_UNUSED(arg);
	uint8_t rx_len;


	if (!usb_dc_wch_state.attached) {
		LOG_ERR("USB interrupt occurred while device is detached");
	}

	while(1) {
		uint32_t status = *(volatile uint32_t *)(USB_REG_BASE + R32_USB_STATUS);
		uint8_t stat = (uint8_t)(status >> 24);
		uint8_t flag = (uint8_t)(status >> 16);
		//uint8_t misc_stat = (uint8_t)(status >> 8);

		if (flag & MASK_UIF_FLAGS) {
			// pull up gpio a5 */
			//*(volatile uint32_t *)0x400010a8 &= ~(1UL << 5);
			;
		} else {
			break;
		}

		LOG_DBG("usb status: 0x%08x", status);
		
		if (flag & RB_UIF_TRANSFER) {
			LOG_DBG("XMIT event");

			if ((stat & MASK_UIS_TOKEN) != (WCH_USB_TOKEN_NONE << POS_UIS_TOKEN)) {
				if (flag & RB_U_IS_NAK)
					goto out;

				uint8_t ep_idx = (stat & MASK_UIS_ENDP) >> POS_UIS_ENDP;

				switch (stat & MASK_UIS_TOKEN) {
					uint8_t ep_ctrl;
					case WCH_USB_TOKEN_OUT << POS_UIS_TOKEN:
						/* close tx/rx. let this tranfer handler decide to open next transfer */
						ep_ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx));
						ep_ctrl = (ep_ctrl & (~MASK_UEP_R_RES)) | (USB_RESP_NAK << POS_UEP_R_RES);
						*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) = ep_ctrl;

						rx_len = *(volatile uint8_t *)(USB_REG_BASE + R8_USB_RX_LEN);

						LOG_DBG("epnum 0x%02x, rx_count %u", ep_idx, rx_len);

						if (ep_idx == 0 && setup_req_to_host) {
							/* status stage */
							*(volatile uint8_t *)(USB_REG_BASE + R8_UEP0_CTRL) = (USB_RESP_NAK << POS_UEP_T_RES) | (USB_RESP_ACK << POS_UEP_R_RES);
						} else {
							*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) ^= RB_UEP_R_TOG;
						}
						if (ep_idx != 0 && !(stat & RB_UIS_TOG_OK)) {
							LOG_ERR("Toggle not OK!");
						}
						usb_dc_wch_state.out_ep_state[ep_idx].read_count = rx_len;
						usb_dc_wch_state.out_ep_state[ep_idx].read_offset = 0;

						if (usb_dc_wch_state.out_ep_state[ep_idx].cb) {
							usb_dc_wch_state.out_ep_state[ep_idx].cb(ep_idx | USB_EP_DIR_OUT, USB_DC_EP_DATA_OUT);
						}
						break;
					case WCH_USB_TOKEN_IN << POS_UIS_TOKEN:
						/* close tx/rx. let this tranfer handler decide to open next transfer */
						ep_ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx));
						ep_ctrl = (ep_ctrl & (~MASK_UEP_T_RES)) | (USB_RESP_NAK << POS_UEP_T_RES);
						*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) = ep_ctrl;

						LOG_DBG("epnum 0x%02x tx", ep_idx);
						k_sem_give(&(usb_dc_wch_state.in_ep_state[ep_idx].write_sem));		/* allow next ep write */

						if (usb_dc_wch_state.update_address) {
							uint8_t address = *(volatile uint8_t *)(USB_REG_BASE + R8_USB_DEV_AD);
							address = (address & (~MASK_USB_ADD)) | (usb_dc_wch_state.address & MASK_USB_ADD);
							*(volatile uint8_t *)(USB_REG_BASE + R8_USB_DEV_AD) = address;
							usb_dc_wch_state.update_address = 0;
						}

						if (ep_idx == 0 && !setup_req_to_host) {
							*(volatile uint8_t *)(USB_REG_BASE + R8_UEP0_CTRL) = (USB_RESP_NAK << POS_UEP_T_RES) | (USB_RESP_ACK << POS_UEP_R_RES);
						} else {
							*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) ^= RB_UEP_T_TOG;
						}

						if (usb_dc_wch_state.in_ep_state[ep_idx].cb) {
							usb_dc_wch_state.in_ep_state[ep_idx].cb(ep_idx | USB_EP_DIR_IN, USB_DC_EP_DATA_IN);
						}
						break;
					case WCH_USB_TOKEN_SOF << POS_UIS_TOKEN:
						usb_dc_wch_state.status_cb(USB_DC_SOF, NULL);
						/* TODO */
						break;
				}
			} else if (stat & RB_UIS_SETUP_ACT) {
				/* setup transaction force datax to data1, don't check toggle ok here */
				*(volatile uint8_t *)(USB_REG_BASE + R8_UEP0_CTRL) |= RB_UEP_T_TOG | RB_UEP_R_TOG;

				setup_req_to_host = *get_ep_buf(USB_CONTROL_EP_OUT) & 0x80;		/* assume little endian */

				usb_dc_wch_state.out_ep_state[0].read_count = 8;
				usb_dc_wch_state.out_ep_state[0].read_offset = 0;
				if (usb_dc_wch_state.out_ep_state[0].cb) {
					usb_dc_wch_state.out_ep_state[0].cb(USB_CONTROL_EP_OUT, USB_DC_EP_SETUP);
				}
			}
		}

		if (flag & RB_UIF_BUS_RST) {
			LOG_DBG("USB RESET event");

			/* Inform upper layers */
			if (usb_dc_wch_state.status_cb) {
				usb_dc_wch_state.status_cb(USB_DC_RESET, NULL);
			}

			/* Clear device address during reset. */
			*(volatile uint8_t *)(USB_REG_BASE + R8_USB_DEV_AD) &= ~MASK_USB_ADD;
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
out:
		*(volatile uint8_t *)(USB_REG_BASE + R8_USB_INT_FG) = MASK_UIF_FLAGS << POS_UIF_FLAGS;
		/* pull up gpio a5, for debug */
		//*(volatile uint32_t *)0x400010a8 |= (1UL << 5);
	}

}

int usb_dc_attach(void)
{
	LOG_DBG("try to attach");

	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_DEV_AD) &= ~MASK_USB_ADD;

	*(volatile uint8_t *)(USB_REG_BASE + R8_UDEV_CTRL) |= RB_UD_PORT_EN;

	usb_dc_wch_state.attached = 1;
    //pull up dp, then interrupts is coming
	uint8_t ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_USB_CTRL);
	ctrl = (ctrl & (~MASK_UC_SYS_CTRL)) | (WCH_USB_SYSCTL_ENABLE_USB_AND_DP_PU << POS_UC_SYS_CTRL);
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_CTRL) = ctrl;

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

	uint8_t ep_ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx));
	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_ctrl = (ep_ctrl & (~MASK_UEP_R_RES)) | (USB_RESP_STALL << POS_UEP_R_RES);
    } else {
		ep_ctrl = (ep_ctrl & (~MASK_UEP_T_RES)) | (USB_RESP_STALL << POS_UEP_T_RES);
    }
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) = ep_ctrl;

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

	uint8_t ep_ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx));
	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_ctrl = (ep_ctrl & (~MASK_UEP_R_RES)) | (USB_RESP_ACK << POS_UEP_R_RES);
    } else {
		ep_ctrl = (ep_ctrl & (~MASK_UEP_T_RES)) | (USB_RESP_NAK << POS_UEP_T_RES);
    }
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) = ep_ctrl;

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

	uint8_t ep_ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx));
	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_ctrl = (ep_ctrl & (~(MASK_UEP_R_RES | RB_UEP_R_TOG))) | (USB_RESP_ACK << POS_UEP_R_RES);
    } else {
		if (ep_state->ep_type == WCH_EP_TYPE_ISOC) {
			ep_ctrl = (ep_ctrl & (~(MASK_UEP_T_RES | RB_UEP_T_TOG))) | (USB_RESP_NO_RESP << POS_UEP_T_RES);
		} else {
			ep_ctrl = (ep_ctrl & (~(MASK_UEP_T_RES | RB_UEP_T_TOG))) | (USB_RESP_NAK << POS_UEP_T_RES);
		}
    }
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) = ep_ctrl;

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
	uint8_t ep_ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx));
	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_ctrl = (ep_ctrl & (~MASK_UEP_R_RES)) | (USB_RESP_NAK << POS_UEP_R_RES);
    } else {
		ep_ctrl = (ep_ctrl & (~MASK_UEP_T_RES)) | (USB_RESP_NAK << POS_UEP_T_RES);
    }
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) = ep_ctrl;

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

	*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_T_LEN(ep_idx)) = len;
	uint8_t ep_ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx));
	ep_ctrl = (ep_ctrl & (~MASK_UEP_T_RES)) | (USB_RESP_ACK << POS_UEP_T_RES);
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) = ep_ctrl;

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

	uint8_t ep_ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx));
	ep_ctrl = (ep_ctrl & (~MASK_UEP_R_RES)) | (USB_RESP_ACK << POS_UEP_R_RES);
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(ep_idx)) = ep_ctrl;

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
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_DEV_AD) &= ~MASK_USB_ADD;

	usb_dc_wch_state.attached = 0;
    //pull dp down, while leave usb enabled
	uint8_t ctrl = *(volatile uint8_t *)(USB_REG_BASE + R8_USB_CTRL);
	ctrl = (ctrl & (~MASK_UC_SYS_CTRL)) | (WCH_USB_SYSCTL_ENABLE_USB << POS_UC_SYS_CTRL);
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_CTRL) = ctrl;

	*(volatile uint8_t *)(USB_REG_BASE + R8_UDEV_CTRL) &= ~RB_UD_PORT_EN;

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

	/* reset and deassert */
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_CTRL) = RB_UC_CLR_ALL | RB_UC_RESET_SIE;
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_CTRL) = 0;

	/* For a pair of IN/OUT eps, disabling OUT ep changes dma buf addr of IN
	 * ep, and they may be used in different interfaces and decoupled modules,
	 * so we always enable all eps */
#ifdef DUAL_BUFFER
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEP4_1_MOD) = RB_UEP4_TX_EN | RB_UEP4_RX_EN | RB_UEP1_BUF_MOD | RB_UEP1_TX_EN | RB_UEP1_RX_EN;
#else
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEP4_1_MOD) = RB_UEP4_TX_EN | RB_UEP4_RX_EN | RB_UEP1_TX_EN | RB_UEP1_RX_EN;
#endif

#ifdef DUAL_BUFFER
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEP2_3_MOD) = RB_UEP2_BUF_MOD | RB_UEP2_TX_EN | RB_UEP2_RX_EN | RB_UEP3_BUF_MOD | RB_UEP3_TX_EN | RB_UEP3_RX_EN;
#else
	*(volatile uint8_t *)(USB_REG_BASE + R8_UEP2_3_MOD) = RB_UEP2_TX_EN | RB_UEP2_RX_EN | RB_UEP3_TX_EN | RB_UEP3_RX_EN;
#endif

	/* set up all endpoints' dma buffers */
	*(volatile uint16_t *)(USB_REG_BASE + R16_UEP0_DMA) = (uint16_t)(uint32_t)&(usb_dc_wch_state.ep_0_4_buf[0]);
	*(volatile uint16_t *)(USB_REG_BASE + R16_UEP1_DMA) = (uint16_t)(uint32_t)&(usb_dc_wch_state.ep123_buf[0]);
	*(volatile uint16_t *)(USB_REG_BASE + R16_UEP2_DMA) = (uint16_t)(uint32_t)&(usb_dc_wch_state.ep123_buf[1]);
	*(volatile uint16_t *)(USB_REG_BASE + R16_UEP3_DMA) = (uint16_t)(uint32_t)&(usb_dc_wch_state.ep123_buf[2]);

	for (i = 0; i < 5; i++) {
#ifdef DUAL_BUFFER
		*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(i)) = (USB_RESP_NAK << POS_UEP_T_RES) | (USB_RESP_NAK << POS_UEP_R_RES) | RB_UEP_AUTO_TOG;
#else
		*(volatile uint8_t *)(USB_REG_BASE + R8_UEPx_CTRL(i)) = (USB_RESP_NAK << POS_UEP_T_RES) | (USB_RESP_NAK << POS_UEP_R_RES);
#endif
	}

	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_DEV_AD) = 0;

	/* enable usb, but don't attach */
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_CTRL) = RB_UC_DMA_EN | RB_UC_INT_BUSY | (WCH_USB_SYSCTL_ENABLE_USB << UC_SYS_CTRL_POS);

	/* clear stale interrupts */
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_INT_FG) = 0xFF;

	*(volatile uint8_t *)(USB_REG_BASE + R8_UDEV_CTRL) =  RB_UD_PD_DIS;

#ifdef CONFIG_USB_DEVICE_SOF
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_INT_EN) = RB_UIE_BUS_RST | RB_UIE_TRANSFER | RB_UIE_FIFO_OV | RB_UIE_DEV_SOF;
#else
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_INT_EN) = RB_UIE_BUS_RST | RB_UIE_TRANSFER | RB_UIE_FIFO_OV;
#endif

	/* use the semaphore to control refilling of IN buf */
	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		k_sem_init(&usb_dc_wch_state.in_ep_state[i].write_sem, 1, 1);
	}

	/* clear stale interrupts */
	*(volatile uint8_t *)(USB_REG_BASE + R8_USB_INT_FG) = 0xFF;

	extern void __wch_irq_wrapper(void);
	/* vector table free interrupt */
	*(volatile uint8_t *)(0xE000E050) = 22;
	*(volatile uint32_t *)(0xE000E060) = (uint32_t)__wch_irq_wrapper + 1;

	IRQ_CONNECT(USB_IRQ, USB_IRQ_PRI, usb_dc_wch_isr, 0, 0);
	irq_enable(USB_IRQ);

	return 0;
}

SYS_INIT(usb_dc_wch_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
