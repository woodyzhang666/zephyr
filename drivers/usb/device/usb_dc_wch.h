
#ifndef DRIVER_USB_DEVICE_USB_DC_WCH_H_
#define DRIVER_USB_DEVICE_USB_DC_WCH_H_

enum wch_ep_type {
    WCH_EP_TYPE_CTRL,
    WCH_EP_TYPE_INTR,
    WCH_EP_TYPE_ISOC,
    WCH_EP_TYPE_BULK
};

union wch_usb_ctrl {
    struct {
        volatile uint8_t dma_en : 1;
        volatile uint8_t clr_fifo : 1;
        volatile uint8_t reset_sie : 1;  /* reset protocol processor */
        volatile uint8_t int_busy : 1;
        volatile uint8_t sys_ctrl : 2;
        volatile uint8_t set_low_speed : 1;
        volatile uint8_t set_host_mode : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

enum wch_usb_sysctl {
    WCH_USB_SYSCTL_DISABLE_USB = 0,
    WCH_USB_SYSCTL_ENABLE_USB = 1,
    WCH_USB_SYSCTL_ENABLE_USB_AND_DP_PU = 2
};

union wch_usb_int_en {
    struct {
        volatile uint8_t bus_reset : 1;
        volatile uint8_t xmit_done : 1;
        volatile uint8_t suspend : 1;
        uint8_t _rsvd0 : 1;;
        volatile uint8_t fifo_ov : 1;
        uint8_t _rsvd1 : 1;;
        volatile uint8_t nak : 1;
        volatile uint8_t sof : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

union wch_usb_addr {
    struct {
        volatile uint8_t addr : 7;
        volatile uint8_t user_define : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

union wch_usb_misc_status {
    struct {
        volatile uint8_t _rsvd2 : 1;
        volatile uint8_t _rsvd3 : 1;
        volatile uint8_t suspended : 1;
        volatile uint8_t bus_in_reset : 1;
        volatile uint8_t rx_fifo_ready : 1;
        volatile uint8_t sie_free : 1;
        uint8_t _rsvd0 : 1;
        uint8_t _rsvd1 : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

union wch_usb_int_flag {
    struct {
        volatile uint8_t bus_reset : 1;
        volatile uint8_t xmit_done : 1;
        volatile uint8_t suspended : 1;
        volatile uint8_t sof: 1;
        volatile uint8_t fifo_ov : 1;
		/* below are not int flags */
        volatile uint8_t sie_free : 1;
        volatile uint8_t toggle_ok : 1;
        volatile uint8_t resp_nak : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

union wch_usb_int_status {
    struct {
        volatile uint8_t ep_idx : 4;
        volatile uint8_t token_pid : 2;
        volatile uint8_t toggle_ok : 1;
        volatile uint8_t setup_ack : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

enum wch_usb_token_pid {
	WCH_USB_TOKEN_OUT = 0,
	WCH_USB_TOKEN_SOF = 1,
	WCH_USB_TOKEN_IN = 2,
	WCH_USB_TOKEN_NONE = 3
};

union wch_usb_dev_ctrl {
    struct {
        volatile uint8_t port_en : 1;
        volatile uint8_t user_bit : 1;
        volatile uint8_t low_speed : 1;
        uint8_t _rsvd0 : 1;
        volatile uint8_t dm_state : 1;
        volatile uint8_t dp_state : 1;
        uint8_t _rsvd1 : 1;
        volatile uint8_t disable_pd : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

/* ep4 dma buf follows the 64B for ep0 located by EP0_DMA */
union wch_usb_ep4_1_mod {
    struct {
        uint8_t _rsvd0 : 2;
        volatile uint8_t ep4_tx_en : 1;
        volatile uint8_t ep4_rx_en : 1;
        volatile uint8_t ep1_dual_buf : 1;
        uint8_t _rsvd1 : 1;
        volatile uint8_t ep1_tx_en : 1;
        volatile uint8_t ep1_rx_en : 1;
    };
    uint8_t val;
} __attribute__ ((packed));

union wch_usb_ep2_3_mod {
    struct {
        volatile uint8_t ep2_dual_buf : 1;
        uint8_t _rsvd0 : 1;
        volatile uint8_t ep2_tx_en : 1;
        volatile uint8_t ep2_rx_en : 1;
        volatile uint8_t ep3_dual_buf : 1;
        uint8_t _rsvd1 : 1;
        volatile uint8_t ep3_tx_en : 1;
        volatile uint8_t ep3_rx_en : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

struct wch_usb_ep_dma_addr {
    volatile uint16_t val;
    uint16_t _rsvd;
} __attribute__ ((packed));

struct wch_usb_ep_tx_len_ctrl {
    volatile uint8_t tx_len;
    uint8_t _rsvd0;
    union {
        struct {
            volatile uint8_t tx_resp : 2;
            volatile uint8_t rx_resp : 2;
            volatile uint8_t auto_toggle : 1;    /* only for dual buf ep */
            uint8_t _rsvd : 1;
            volatile uint8_t tx_tog : 1;
            volatile uint8_t rx_tog : 1;
        };
        volatile uint8_t val;
    } ep_ctrl;
    uint8_t _rsvd1;
} __attribute__ ((packed));

enum usb_resp {
    USB_RESP_ACK = 0,
    USB_RESP_NO_RESP,
    USB_RESP_NAK,
    USB_RESP_STALL,
};

struct wch_usb {
    volatile union wch_usb_ctrl ctrl;                /* 0x00 */
    volatile union wch_usb_dev_ctrl dev_ctrl;        /* 0x01 */
    volatile union wch_usb_int_en int_en;            /* 0x02 */
    volatile union wch_usb_addr adr;                 /* 0x03 */
    volatile union {
        struct {
            uint8_t _rsvd1;
            volatile union wch_usb_misc_status misc_status;  /* 0x05 */
            volatile union wch_usb_int_flag int_flag;        /* 0x06*/
            volatile union wch_usb_int_status int_status;    /* 0x07 */
        };
        volatile uint32_t status;
    };
    uint8_t volatile rx_len;                         /* 0x08 */
    uint8_t _rsvd2[3];
    volatile union wch_usb_ep4_1_mod ep4_1_mod;      /* 0x0c */
    volatile union wch_usb_ep2_3_mod ep2_3_mod;      /* 0x0d */
    uint8_t _rsvd3[2];
    volatile struct wch_usb_ep_dma_addr ep_dma_addr[4];              /* 0x10 */
    volatile struct wch_usb_ep_tx_len_ctrl ep_tx_len_ctrl[5];        /* 0x20 */

} __attribute__ ((packed));

BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->ctrl) == 0x00);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->dev_ctrl) == 0x01);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->int_en) == 0x02);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->adr) == 0x03);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->status) == 0x04);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->misc_status) == 0x05);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->int_flag) == 0x06);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->int_status) == 0x07);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->rx_len) == 0x08);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->ep4_1_mod) == 0x0c);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->ep2_3_mod) == 0x0d);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->ep_dma_addr[0]) == 0x10);
BUILD_ASSERT((uint32_t)(&((struct wch_usb *)0)->ep_tx_len_ctrl[0]) == 0x20);
BUILD_ASSERT((sizeof(struct wch_usb)) == 0x34);

#endif  /* DRIVER_USB_DEVICE_USB_DC_WCH_H_ */
