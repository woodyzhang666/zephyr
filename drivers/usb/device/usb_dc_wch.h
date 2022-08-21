
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

#define R8_USB_CTRL		0
#define RB_UC_DMA_EN		0x01
#define RB_UC_CLR_ALL		0x02
#define RB_UC_RESET_SIE		0x04
#define RB_UC_INT_BUSY		0x08	/* resp NAK after entering interrupt. How about set resp ACK in ISR */
#define MASK_UC_SYS_CTRL	0x30
#define POS_UC_SYS_CTRL		4
#define UC_SYS_CTRL_POS		0x4
#define RB_UC_LOW_SPEED		0x40
#define RB_UC_HOST_MODE		0x80

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

#define R8_USB_INT_EN	2
#define RB_UIE_BUS_RST		0x01
#define RB_UIE_DETECT		0x01	/* in host mode, device detected */
#define RB_UIE_TRANSFER		0x02
#define RB_UIE_SUSPEND		0x04
#define RB_UIE_HST_SOF		0x08
#define RB_UIE_FIFO_OV		0x10	/* in host mode, internal sof timer */
#define RB_UIE_DEV_NAK		0x40	/* don't generate int on NAKed IN. and OUT? */
#define RB_UIE_DEV_SOF		0x80	/* on receiving sof */

union wch_usb_addr {
    struct {
        volatile uint8_t addr : 7;
        volatile uint8_t user_define : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

#define R8_USB_DEV_AD	3
#define MASK_USB_ADD	0x7F
#define POS_USB_ADD		0
#define RB_UDA_GP_BIT		0x80

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

#define R32_USB_STATUS	4

#define R8_USB_MIS_ST	5
#define RB_UMS_DEV_ATTACH	0x01	/* in host mode, some device is attached */
#define RB_UMS_DM_LEVEL		0x02	/* in host mode, device is low speed */
#define RB_UMS_SUSPEND		0x04	/* suspending */
#define RB_UMS_BUS_RESET	0x08	/* reset asserting now */
#define RB_UMS_R_FIFO_RDY	0x10	/* rx fifo is ready */
#define RB_UMS_SIE_FREE		0x20	/* sie is free */
#define RB_UMS_SOF_ACT		0x40	/* in host mode, sof is sending out */
#define RB_UMS_SOF_PRES		0x80	/* in host mode, sof is about to send */

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

#define R8_USB_INT_FG	6
#define RB_UIF_BUS_RST		0x01
#define RB_UIF_DETECT		0x01	/* in host mode, device connected or disconnected */
#define RB_UIF_TRANSFER		0x02
#define RB_UIF_SUSPEND		0x04
#define RB_UIF_HST_SOF		0x08	/* in host mode, sof sent out */
#define RB_UIF_FIFO_OV		0x10
#define RB_U_SIE_FREE		0x20
#define RB_U_TOG_OK			0x40	/* datax tog ok */
#define RB_U_IS_NAK			0x80	/* current transfer is NAKed */
#define MASK_UIF_FLAGS		0x1F
#define POS_UIF_FLAGS		0

union wch_usb_int_status {
    struct {
        volatile uint8_t ep_idx : 4;
        volatile uint8_t token_pid : 2;
        volatile uint8_t toggle_ok : 1;
        volatile uint8_t setup_ack : 1;
    };
    volatile uint8_t val;
} __attribute__ ((packed));

#define R8_USB_INT_ST	7
#define MASK_UIS_ENDP		0x0F	/* ep index */
#define POS_UIS_ENDP		0
#define MASK_UIS_H_RES		0x0F
#define POS_UIS_H_RES		0
#define MASK_UIS_TOKEN		0x30
#define POS_UIS_TOKEN		4
#define RB_UIS_TOG_OK		0x40
#define RB_UIS_SETUP_ACT	0x80	/* setup transfer received */

enum wch_usb_token_pid {
	WCH_USB_TOKEN_OUT = 0,
	WCH_USB_TOKEN_SOF = 1,
	WCH_USB_TOKEN_IN = 2,
	WCH_USB_TOKEN_NONE = 3
};

#define R8_USB_RX_LEN	8

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

#define R8_UDEV_CTRL	0x1
#define RB_UD_PORT_EN		0x01
#define RB_UD_GP_BIT		0x02
#define RB_UD_LOW_SPEED		0x04
#define RB_UD_DM_PIN		0x10	/* current DM level */
#define RB_UD_DP_PIN		0x20	/* current DP level */
#define RB_UD_PD_DIS		0x80	/* disable internal pull down resistor */

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

#define R8_UEP4_1_MOD	0xC
#define RB_UEP4_TX_EN		0x04
#define RB_UEP4_RX_EN		0x08
#define RB_UEP1_BUF_MOD		0x10
#define RB_UEP1_TX_EN		0x40
#define RB_UEP1_RX_EN		0x80

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

#define R8_UEP2_3_MOD	0xD
#define RB_UEP2_BUF_MOD		0x01
#define RB_UEP2_TX_EN		0x04
#define RB_UEP2_RX_EN		0x08
#define RB_UEP3_BUF_MOD		0x10
#define RB_UEP3_TX_EN		0x40
#define RB_UEP3_RX_EN		0x80

#define R8_UEP567_MOD	0xE
#define RB_UEP5_TX_EN		0x01
#define RB_UEP5_RX_EN		0x02
#define RB_UEP6_TX_EN		0x04
#define RB_UEP6_RX_EN		0x08
#define RB_UEP7_TX_EN		0x10
#define RB_UEP7_RX_EN		0x20

struct wch_usb_ep_dma_addr {
    volatile uint16_t val;
    uint16_t _rsvd;
} __attribute__ ((packed));

#define R16_UEP0_DMA	0x10
#define R16_UEP1_DMA	0x14
#define R16_UEP2_DMA	0x18
#define R16_UEP3_DMA	0x1C

#define R16_UEP5_DMA	0x54
#define R16_UEP6_DMA	0x58
#define R16_UEP7_DMA	0x5C

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

#define R8_UPE0_T_LEN	0x20
#define R8_UEP0_CTRL	0x22
#define R8_UEP1_T_LEN	0x24
#define R8_UEP1_CTRL	0x26
#define R8_UEP2_T_LEN	0x28
#define R8_UEP2_CTRL	0x2A
#define R8_UEP3_T_LEN	0x2C
#define R8_UEP3_CTRL	0x2E
#define R8_UEP4_T_LEN	0x30
#define R8_UEP4_CTRL	0x32

#define R8_UEP5_T_LEN	0x64
#define R8_UEP5_CTRL	0x66
#define R8_UEP6_T_LEN	0x68
#define R8_UEP6_CTRL	0x6A
#define R8_UEP7_T_LEN	0x6C
#define R8_UEP7_CTRL	0x6E

#define R8_UEPx_T_LEN(x)	((x < 5) ? (0x20 + x * 0x4) : (0x50 + x * 0x4))
#define R8_UEPx_CTRL(x)		((x < 5) ? (0x22 + x * 0x4) : (0x52 + x * 0x4))

#define MASK_UEP_T_RES	0x03
#define POS_UEP_T_RES	0
#define MASK_UEP_R_RES	0x0C
#define POS_UEP_R_RES	2
#define RB_UEP_AUTO_TOG 0x10
#define RB_UEP_T_TOG	0x40
#define RB_UEP_R_TOG	0x80


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
