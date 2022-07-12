
#ifndef DRIVERS_SERIAL_UART_WCH_H_
#define DRIVERS_SERIAL_UART_WCH_H_

#define True	1
#define False	0

/* register definitions */

/* Modem Control Register */
struct uart_mcr {
	uint8_t dtr : 1;
	uint8_t rts	: 1;
	uint8_t out1 : 1;
	uint8_t int_oe : 1;				/* interrupt out enable, for modem only ? */
	uint8_t loop : 1;				/* internal loopback */
	uint8_t hw_flow_ctrl : 1;
	uint8_t dtr_indicate_tx : 1;
	uint8_t half_duplex : 1;		/* half duplex mode */
} __attribute__ ((packed));

/* Interrupt Enabling Register */
struct uart_ier {
	uint8_t recv_rdy_ie : 1;		/* recv data interrupt enable */
	uint8_t thr_empty_ie : 1;		/* tx holding reg empty interrupt enable */
	uint8_t line_stat_ie : 1;		/* rx link stat interrupt enable */
	uint8_t modem_chg_ie : 1;		/* modem input state change interrupt enable */
	uint8_t dtr_output_en : 1;
	uint8_t rts_output_en : 1;
	uint8_t txd_output_en : 1;
	uint8_t uart_reset : 1;
} __attribute__ ((packed));

/* FIFO Control Register */
struct uart_fcr {
	uint8_t fifo_en : 1;
	uint8_t rx_fifo_clr	: 1;
	uint8_t tx_fifo_clr : 1;
	uint8_t rsvd : 3;
	uint8_t fifo_trig : 2;
} __attribute__ ((packed));

enum uart_fcr_fifo_trig {
	UART_FIFO_TRIG_1_BYTE = 0,
	UART_FIFO_TRIG_2_BYTES = 1,
	UART_FIFO_TRIG_4_BYTES = 2,
	UART_FIFO_TRIG_7_BYTES = 3,
};

/* Link Control Register */
struct uart_lcr {
	uint8_t word_sz : 2;
	uint8_t stop_bits : 1;
	uint8_t parity_en : 1;
	uint8_t parity_mode : 2;
	uint8_t break_en : 1;
	uint8_t gp_bit : 1;
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
struct uart_iir {
	uint8_t no_int : 1;
	uint8_t int_type : 3;
	uint8_t _rsvd : 2;
	uint8_t fifo_status : 2;
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
struct uart_lsr {
	volatile uint8_t data_rdy : 1;
	volatile uint8_t rx_overflow : 1;
	volatile uint8_t parity_err : 1;
	volatile uint8_t frame_err : 1;
	volatile uint8_t break_err : 1;
	volatile uint8_t tx_fifo_empty : 1;
	volatile uint8_t thr_tsr_both_emtpy : 1;
	volatile uint8_t err_in_rx_fifo : 1;
} __attribute__ ((packed));

/* Modem Status Register */
struct uart_msr {
	uint8_t cts_changed : 1;
	uint8_t dsr_changed : 1;
	uint8_t ri_changed : 1;
	uint8_t dcd_changed : 1;
	uint8_t cts_valid : 1;
	uint8_t dsr_valid : 1;
	uint8_t ri_valid : 1;
	uint8_t dcd_valid : 1;
} __attribute__ ((packed));

/* Receive Buffer Register: RO */
struct uart_rbr {
	uint8_t val;					/* read to get 1 byte from rx fifo */
} __attribute__ ((packed));

/* Transmission Holding Register: WO */
struct uart_thr {
	uint8_t val;					/* write to put 1 byte into tx fifo */
} __attribute__ ((packed));

/* Receive FIFO Count */
struct uart_rfc {
	uint8_t val;					/* rx fifo data count */
} __attribute__ ((packed));

/* Transmission FIFO Count */
#define R8_UART_TFC					0x0B
struct uart_tfc {
	uint8_t val;					/* tx fifo data count */
} __attribute__ ((packed));

/* Divisor Lock */
struct uart_dl {
	uint16_t divisor2;
} __attribute__ ((packed));

/* Pre-Divisor */
struct uart_div {
	uint8_t divisor1;
} __attribute__ ((packed));

/* Baudrate Calculation Method:
 * Baudrate = Fsys * 2 / uart_div / 16 / uart_dl
 */

/* slave address in 485 ? */
struct uart_adr {
	uint8_t val;					/* slave address */
} __attribute__ ((packed));

struct wch_uart {
	struct uart_mcr mcr;			/* 0x00 */
	struct uart_ier ier;			/* 0x01 */
	struct uart_fcr fcr;			/* 0x02 */
	struct uart_lcr lcr;			/* 0x03 */
	struct uart_iir iir;			/* 0x04 */
	struct uart_lsr lsr;			/* 0x05 */
	struct uart_msr msr;			/* 0x06 */
	uint8_t _rsvd1;
	union {							/* 0x08 */
		struct uart_rbr rbr;
		struct uart_thr thr;
	};
	uint8_t _rsvd2;
	struct uart_rfc rfc;			/* 0x0A */
	struct uart_tfc tfc;			/* 0x0B */
	struct uart_dl  dl;				/* 0x0C two bytes */
	struct uart_div div;			/* 0x0E */
	struct uart_adr adr;			/* 0x0F */
} __attribute__ ((packed));


struct uart_wch_config {
	/* base address of registers */
	void *base;
	/* initial hardware flow control, 1 for RTS/CTS */
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
	/* clock device */
	const struct device *clock;
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
