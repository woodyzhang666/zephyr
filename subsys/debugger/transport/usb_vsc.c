
#include <zephyr/init.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <usb_descriptor.h>
#include "crb.h"

#define LOG_LEVEL CONFIG_USB_DEVICE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_debugger);

#define DEBUGGER_OUT_EP_ADDR		0x01
#define DEBUGGER_IN_EP_ADDR		0x81

#define DEBUGGER_OUT_EP_IDX		0
#define DEBUGGER_IN_EP_IDX		1

static uint8_t loopback_buf[128];   /* to be deleted */

static uint8_t *rx_data_ptr;    /* current pointer in a crb */
static uint32_t rx_data_len;
static uint8_t *tx_data_ptr;
static uint32_t tx_data_len;

static DAP_queue DAP_Cmd_queue;
static volatile uint8_t resp_idle;
//BUILD_ASSERT(sizeof(loopback_buf) == CONFIG_USB_REQUEST_BUFFER_SIZE);

struct usb_debugger_config {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_out_ep;
	struct usb_ep_descriptor if0_in_ep;
} __packed;

USBD_CLASS_DESCR_DEFINE(primary, 0) struct usb_debugger_config debugger_cfg = {
	/* Interface descriptor 0 */
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},

	/* Data Endpoint OUT */
	.if0_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = DEBUGGER_OUT_EP_ADDR,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(CONFIG_DEBUGGER_USB_EP_MPS),
		.bInterval = 0x00,
	},

	/* Data Endpoint IN */
	.if0_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = DEBUGGER_IN_EP_ADDR,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(CONFIG_DEBUGGER_USB_EP_MPS),
		.bInterval = 0x00,
	},
};

static void debugger_bulk_out_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	uint32_t bytes_read;
	int ret = 0;
	uint8_t *resp_buf;

	/* may fail ? */
	usb_read(ep, rx_data_ptr, CONFIG_DEBUGGER_CRB_BUF_SIZE - rx_data_len, &bytes_read);
	rx_data_ptr += bytes_read;
	rx_data_len += bytes_read;

    if (rx_data_len >= CONFIG_DEBUGGER_CRB_BUF_SIZE) {
        LOG_ERR("Debugger Rx Buffer is full, overflow may occurred!");
    }

    /* non-full packet is the last packet in a transfer */
	if (bytes_read < CONFIG_DEBUGGER_USB_EP_MPS) {
        struct crb *resp_crb = crb_queue_claim(resp_queue);
        if (resp_crb != NULL) {
            cmd_crb->len = rx_data_len;

            debugger_handle_cmd(cmd_crb, resp_crb);
            /* TODO add crb_finish to finalize claim */
            
            if (resp_idle) {
                /* trigger bulk in for the reply */
                debugger_bulk_in_cb(ep_cfg[1].ep_addr, USB_DC_EP_DATA_IN);
                resp_idle = 0;
            }
        }

		/* re-arm rx buf */
		rx_data_ptr = cmd_crb->buf;
		rx_data_len = 0;
	}
}

static void debugger_bulk_in_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	uint8_t *tx_buf = 0;
	int tx_len;
    struct crb *resp_crb;

	if ((resp_crb = crb_queue_get_resp(resp_queue)) != NULL) {
		usb_write(ep, resp_crb->buf, CONFIG_DEBUGGER_USB_EP_MPS, NULL);
	} else {
		resp_idle = 1;
	}
}

static struct usb_ep_cfg_data ep_cfg[] = {
	{
		.ep_cb = debugger_bulk_out_cb,
		.ep_addr = DEBUGGER_OUT_EP_ADDR,
	},
	{
		.ep_cb = debugger_bulk_in_cb,
		.ep_addr = DEBUGGER_IN_EP_ADDR,
	},
};

static void debugger_status_cb(struct usb_cfg_data *cfg,
			       enum usb_dc_status_code status,
			       const uint8_t *param)
{
	ARG_UNUSED(cfg);

	switch (status) {
	case USB_DC_INTERFACE:
		debugger_bulk_in_cb(ep_cfg[DEBUGGER_IN_EP_IDX].ep_addr, 0);
		LOG_DBG("USB interface configured");
		break;
	case USB_DC_SET_HALT:
		LOG_DBG("Set Feature ENDPOINT_HALT");
		break;
	case USB_DC_CLEAR_HALT:
		LOG_DBG("Clear Feature ENDPOINT_HALT");
		if (*param == ep_cfg[DEBUGGER_IN_EP_IDX].ep_addr) {
			debugger_bulk_in_cb(ep_cfg[DEBUGGER_IN_EP_IDX].ep_addr, 0);
		}
		break;
	default:
		break;
	}
}

static int debugger_vendor_handler(struct usb_setup_packet *setup,
				   int32_t *len, uint8_t **data)
{
	LOG_DBG("Class request: bRequest 0x%x bmRequestType 0x%x len %d",
		setup->bRequest, setup->bmRequestType, *len);

	if (setup->RequestType.recipient != USB_REQTYPE_RECIPIENT_DEVICE) {
		return -ENOTSUP;
	}

	if (usb_reqtype_is_to_device(setup) &&
	    setup->bRequest == 0x5b) {
		LOG_DBG("Host-to-Device, data %p", *data);
		/*
		 * Copy request data in loopback_buf buffer and reuse
		 * it later in control device-to-host transfer.
		 */
		memcpy(loopback_buf, *data,
		       MIN(sizeof(loopback_buf), setup->wLength));
		return 0;
	}

	if ((usb_reqtype_is_to_host(setup)) &&
	    (setup->bRequest == 0x5c)) {
		LOG_DBG("Device-to-Host, wLength %d, data %p",
			setup->wLength, *data);
		*data = loopback_buf;
		*len = MIN(sizeof(loopback_buf), setup->wLength);
		return 0;
	}

	return -ENOTSUP;
}

static void debugger_interface_config(struct usb_desc_header *head,
				      uint8_t bInterfaceNumber)
{
	ARG_UNUSED(head);

	debugger_cfg.if0.bInterfaceNumber = bInterfaceNumber;
}

USBD_DEFINE_CFG_DATA(debugger_config) = {
	.usb_device_description = NULL,
	.interface_config = debugger_interface_config,
	.interface_descriptor = &debugger_cfg.if0,
	.cb_usb_status = debugger_status_cb,
	.interface = {
		.class_handler = NULL,
		.custom_handler = NULL,
		.vendor_handler = debugger_vendor_handler,
	},
	.num_endpoints = ARRAY_SIZE(ep_cfg),
	.endpoint = ep_cfg,
};

int debugger_transport_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	rx_data_ptr = cmd_crb->buf;
	rx_data_len = 0;

	resp_idle = 1;
}
