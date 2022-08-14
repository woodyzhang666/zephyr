#define DT_DRV_COMPAT		"st,stm32-compat-usb"

#include "usb_dc_stm32_compat.h"

#define REG_BASE	DT_INST_REG_ADDR(0)


struct usb_dc_stm32_state {
	struct stm32_usb *usb;
	usb_dc_status_callback status_cb; /* Status callback */
	struct usb_dc_stm32_ep_state out_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	struct usb_dc_stm32_ep_state in_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	uint8_t ep_buf[USB_NUM_BIDIR_ENDPOINTS][EP_MPS];

#if defined(USB) || defined(USB_DRD_FS)
	uint32_t pma_offset;
#endif /* USB */
};

static int usb_dc_stm32_init(void)
{

}

int usb_dc_attach(void)
{

}
