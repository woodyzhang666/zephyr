
#ifndef DRIVERS_SERIAL_UART_WCH_H_
#define DRIVERS_SERIAL_UART_WCH_H_

#define True	1
#define False	0

/* register definitions */

/* Modem Control Register */
union uart_mcr {
	struct {
		volatile uint8_t dtr : 1;
		volatile uint8_t rts	: 1;
		volatile uint8_t out1 : 1;
		volatile uint8_t int_oe : 1;			/* interrupt out enable, Not modem only, fxxk ! */
		volatile uint8_t loop : 1;				/* internal loopback */
		volatile uint8_t hw_flow_ctrl : 1;
		volatile uint8_t dtr_indicate_tx : 1;
		volatile uint8_t half_duplex : 1;		/* half duplex mode */
	};
	volatile uint8_t val;
} __attribute__ ((packed));

/* Interrupt Enabling Register */
union uart_ier {
	struct {
		volatile uint8_t recv_rdy_ie : 1;		/* recv data interrupt enable */
		volatile uint8_t thr_empty_ie : 1;		/* tx holding reg empty interrupt enable */
		volatile uint8_t line_stat_ie : 1;		/* rx link stat interrupt enable */
		volatile uint8_t modem_chg_ie : 1;		/* modem input state change interrupt enable */
		volatile uint8_t dtr_output_en : 1;
		volatile uint8_t rts_output_en : 1;
		volatile uint8_t txd_output_en : 1;
		volatile uint8_t uart_reset : 1;
	};
	volatile uint8_t val;
} __attribute__ ((packed));

/* FIFO Control Register */
union uart_fcr {
	struct {
		volatile uint8_t fifo_en : 1;
		volatile uint8_t rx_fifo_clr	: 1;
		volatile uint8_t tx_fifo_clr : 1;
		volatile uint8_t rsvd : 3;
		volatile uint8_t fifo_trig : 2;
	};
	volatile uint8_t val;
} __attribute__ ((packed));

enum uart_fcr_fifo_trig {
	UART_FIFO_TRIG_1_BYTES = 0,
	UART_FIFO_TRIG_2_BYTES = 1,
	UART_FIFO_TRIG_4_BYTES = 2,
	UART_FIFO_TRIG_7_BYTES = 3,
};

/* Link Control Register */
union uart_lcr {
	struct {
		volatile uint8_t word_sz : 2;
		volatile uint8_t stop_bits : 1;
		volatile uint8_t parity_en : 1;
		volatile uint8_t parity_mode : 2;
		volatile uint8_t break_en : 1;
		volatile uint8_t gp_bit : 1;
	};
	volatile uint8_t val;
} __attribute__ ((packed));

enum uart_lcr_word_sz {
	UART_LCR_WORD_SZ_5_BIT = 0,
	UART_LCR_WORD_SZ_6_BIT,
	UART_LCR_WORD_SZ_7_BIT,
	UART_LCR_WORD_SZ_8_BIT,
};

enum uart_lcr_stop_bit {
	UART_LCR_STOP_BIT_1_BIT = 0,
	UART_LCR_STOP_BIT_2_BIT = 1,
};

enum uart_lcr_parity_mode {
	UART_LCR_PARITY_MODE_ODD = 0,
	UART_LCR_PARITY_MODE_EVEN,
	UART_LCR_PARITY_MODE_MARK,
	UART_LCR_PARITY_MODE_SPACE,
};

/* Interrupt Id Register */
union uart_iir {
	struct {
		volatile uint8_t no_int : 1;
		volatile uint8_t int_type : 3;
		volatile uint8_t _rsvd : 2;
		volatile uint8_t fifo_status : 2;
	};
	volatile uint8_t val;
} __attribute__ ((packed));

enum uart_iir_int_type {
	UART_IIR_INT_TYPE_MODEL_INPUT_CHG = 0,
	UART_IIR_INT_TYPE_THR_EMPTY,
	UART_IIR_INT_TYPE_RX_TRIG,			/* data received in rx fifo match uart_fcr_fifo_trig */
	UART_IIR_INT_TYPE_RX_LINK_ERR,
	UART_IIR_INT_TYPE_RX_TRIG_TIMEOUT = 4,	/* data received in fifo, but not trigger 4 clocks */
	UART_IIR_INT_TYPE_ADDRESSED = 7,
};

enum uart_iir_fifo_status {
	uart_iir_fifo_status_disabled = 0,
	uart_iir_fifo_status_enabled = 3,
};

/* Link Status Regsiter: RO/RZ
 * Read LSR to clear RX LINK ERR interrupt.
 */
union uart_lsr {
	struct {
		volatile uint8_t data_rdy : 1;
		volatile uint8_t rx_overflow : 1;
		volatile uint8_t parity_err : 1;
		volatile uint8_t frame_err : 1;
		volatile uint8_t break_err : 1;
		volatile uint8_t tx_fifo_empty : 1;
		volatile uint8_t thr_tsr_both_emtpy : 1;
		volatile uint8_t err_in_rx_fifo : 1;
	};
	volatile uint8_t val;
} __attribute__ ((packed));

/* Modem Status Register */
union uart_msr {
	struct {
		volatile uint8_t cts_changed : 1;
		volatile uint8_t dsr_changed : 1;
		volatile uint8_t ri_changed : 1;
		volatile uint8_t dcd_changed : 1;
		volatile uint8_t cts_valid : 1;
		volatile uint8_t dsr_valid : 1;
		volatile uint8_t ri_valid : 1;
		volatile uint8_t dcd_valid : 1;
	};
	volatile uint8_t val;
} __attribute__ ((packed));

/* Baudrate Calculation Method:
 * Baudrate = Fsys * 2 / uart_div / 16 / uart_dl
 */

struct wch_uart {
	union uart_mcr mcr;			/* 0x00 */
	union uart_ier ier;			/* 0x01 */
	union uart_fcr fcr;			/* 0x02 */
	union uart_lcr lcr;			/* 0x03 */
	union uart_iir iir;			/* 0x04 */
	union uart_lsr lsr;			/* 0x05 */
	union uart_msr msr;			/* 0x06 */
	uint8_t _rsvd1;
	union {							/* 0x08 */
		volatile uint8_t rbr;			/* Receive Buffer Register: RO */
		volatile uint8_t thr;			/* Transmission Holding Register: WO */
	};
	uint8_t _rsvd2;
	volatile uint8_t rfc;				/* 0x0A Rx FIFO Count */
	volatile uint8_t tfc;				/* 0x0B Tx FIFO Count*/
	volatile uint16_t dl;				/* 0x0C Divisor lock */
	volatile uint8_t div;				/* 0x0E Pre-Divisor */
	volatile uint8_t adr;				/* 0x0F Slave address in 485 ? */
} __attribute__ ((packed));

BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->mcr) == 0x00);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->ier) == 0x01);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->fcr) == 0x02);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->lcr) == 0x03);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->iir) == 0x04);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->lsr) == 0x05);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->msr) == 0x06);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->rbr) == 0x08);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->thr) == 0x08);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->rfc) == 0x0A);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->tfc) == 0x0B);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->dl) == 0x0C);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->div) == 0x0E);
BUILD_ASSERT((uint32_t)(&((struct wch_uart *)0)->adr) == 0x0F);

struct uart_wch_config {
	/* base address of registers */
	void *base;
	/* clock device */
	const struct device *clock;
	/* initial hardware flow control, 1 for RTS/CTS */
	uint32_t clock_mask;	/* clock arg. should be parsed by clock driver */
	bool hw_flow_control;
	/* initial parity, 0 for none, 1 for odd, 2 for even */
	int  parity;
	/* switch to enable single wire / half duplex feature */
	bool single_wire;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	uart_irq_config_func_t irq_config_func;
#endif
};

struct uart_wch_data {
	uint32_t baudrate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif

#ifdef CONFIG_UART_ASYNC_API
	const struct device *uart_dev;
	uart_callback_t async_cb;
	void *async_user_data;
	struct uart_dma_stream dma_rx;
	struct uart_dma_stream dma_tx;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
#endif
};

#endif /* DRIVERS_SERIAL_UART_WCH_H_ */
