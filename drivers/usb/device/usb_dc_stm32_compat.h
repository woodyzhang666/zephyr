
#ifndef DRIVER_USB_DEVICE_USB_DC_STM32_COMPAT_H_
#define DRIVER_USB_DEVICE_USB_DC_STM32_COMPAT_H_

#define EP_COUNT		8	/* from dts ? */

enum stm32_ep_type {
	STM32_EP_TYPE_BULK = 0,
	STM32_EP_TYPE_CTRL = 1,
	STM32_EP_TYPE_ISO = 2,
	STM32_EP_TYPE_INT = 3
};

struct stm32_usb {
	struct {
		union {
			struct {
				uint16_t ep_addr : 4;
				uint16_t stat_tx : 2;
				uint16_t tog_tx : 1;
				uint16_t tx_ok : 1;
				uint16_t ep_kind : 1;
				uint16_t ep_type : 2;
				uint16_t setup : 1;
				uint16_t stat_rx : 2;
				uint16_t tog_rx : 1;
				uint16_t rx_ok : 1;
			};
			uint16_t val;
		} ep_ctrl;
		uint16_t _rsvd;
	} epx[EP_COUNT];
	uint8_t _rsvd0[0x40 - EP_COUNT * 4];
	union {
		struct {
			uint16_t force_reset : 1;
			uint16_t _rsvd0 : 1;
			uint16_t lpm : 1;	/* enter lpm when suspended, bus wakeup clear this */
			uint16_t suspend_not_detect : 1;
			uint16_t resume : 1;	/* ?? */
			uint16_t _rsvd1 : 3;
			uint16_t int_esof_en : 1;
			uint16_t int_sof_en : 1;
			uint16_t int_reset_en : 1;
			uint16_t int_suspend_en : 1;
			uint16_t int_wake_en : 1;
			uint16_t int_error_en : 1;
			uint16_t int_pma_ov_en : 1;
			uint16_t int_xmit_done_en : 1;
		};
		uint16_t val;
	} ctrl;
	uint16_t _rsvd1;
	union {
		struct {
			uint16_t ep_id : 4;
			uint16_t dir : 1;
			uint16_t _rsvd0 : 3;
			uint16_t esof : 1;	/* host does not send sof  */
			uint16_t sof : 1;	/* sof recevied */
			uint16_t reset : 1;		/* bus reseted */
			uint16_t suspend : 1;
			uint16_t wakeup : 1;
			uint16_t error : 1;
			uint16_t pma_ov : 1;
			uint16_t xmit_done : 1;
		};
		uint16_t val;
	} status;
	uint16_t _rsvd2;
	union {
		struct {
			uint16_t frame_num : 11;
			uint16_t lost_sof_nr : 2;
			uint16_t lock : 1;
			uint16_t dm_level : 1;
			uint16_t dp_level : 1;
		};
		uint16_t val;
	} frame;
	uint16_t _rsvd3;
	union {
		struct {
			uint16_t addr : 7;
			uint16_t usb_enable : 1;
			uint16_t _rsvd : 8;
		};
		uint16_t val;
	} adr;
	uint16_t _rsvd4;
	union {
		struct {
			uint16_t _rsvd : 3;
			uint16_t buf_table : 13;
		};
		uint16_t val;
	} btable;
	uint16_t _rsvd5;
} __attribute__ ((packed));

struct btable_entry {
	uint16_t addr_tx;
	uint16_t _rsvd0;
	uint16_t count_tx;
	uint16_t _rsvd1;
	uint16_t addr_rx;
	uint16_t _rsvd2;
	struct {
		uint16_t count_rx : 10;
		uint16_t buf_blocks : 5;
		uint16_t buf_block_size : 1;	/* 2B or 32B */
	};
	uint16_t _rsvd3;
} __attribute__ ((packed));

#endif /* DRIVER_USB_DEVICE_USB_DC_STM32_COMPAT_H_ */
