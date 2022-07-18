
#ifndef DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_WCH_H_
#define DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_WCH_H_

#define False 0
#define True 1

/** Fixed clocks related symbols */

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_lsi), fixed_clock, okay)
#define WCH_LSI_ENABLED	1
#define WCH_LSI_FREQ		DT_PROP(DT_NODELABEL(clk_lse), clock_frequency)
#else
#define WCH_LSI_ENABLED	0
#define WCH_LSI_FREQ		0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_lse), fixed_clock, okay)
#define WCH_LSE_ENABLED	1
#define WCH_LSE_FREQ		DT_PROP(DT_NODELABEL(clk_lse), clock_frequency)
#else
#define WCH_LSE_ENABLED	0
#define WCH_LSE_FREQ		0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_hse), fixed_clock, okay)
#define WCH_HSE_ENABLED	1
#define WCH_HSE_FREQ		DT_PROP(DT_NODELABEL(clk_hse), clock_frequency)
#else
#define WCH_HSE_ENABLED	0
#define WCH_HSE_FREQ		0
#endif

/* ck32k source selection */

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_ck32k), fixed_clock, okay)
#define WCH_CK32K_ENABLED 1
#define WCH_CK32K_FREQ		DT_PROP(DT_NODELABEL(clk_ck32k), clock_frequency)

#define DT_CK32K_CLOCKS_CTRL	DT_CLOCKS_CTLR(DT_NODELABEL(clk_ck32k))

#if DT_SAME_NODE(DT_CK32K_CLOCKS_CTRL, DT_NODELABEL(clk_lsi))
#define WCH_CK32K_SRC_LSI	1
#if !WCH_LSI_ENABLED
	#error "CK32K selects LSI, but LSI is disabled!"
#endif
#if WCH_LSI_FREQ != WCH_CK32K_FREQ
	#error "CK32K frequency doesn't match against LSI"
#endif
#endif

#if DT_SAME_NODE(DT_CK32K_CLOCKS_CTRL, DT_NODELABEL(clk_lse))
#define WCH_CK32K_SRC_LSE	1
#if !WCH_LSE_ENABLED
	#error "CK32K selects LSE, but LSE is disabled!"
#endif
#if WCH_LSE_FREQ != WCH_CK32K_FREQ
	#error "CK32K frequency doesn't match against LSE"
#endif
#endif

#else
#define WCH_CK32K_ENABLED 0
#define WCH_CK32K_FREQ	0

#endif	/* DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_ck32k), fixed_clock, okay) */

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_pll), wch_ch57x_pll_clock, okay)
#define WCH_PLL_ENABLED		1
#define WCH_PLL_MUL			DT_PROP(DT_NODELABEL(clk_pll), mul)
#define WCH_PLL_USB_PRE		DT_PROP(DT_NODELABEL(clk_pll), usbpre)
#else
#define WCH_PLL_ENABLED		0
#define WCH_PLL_MUL			0
#define WCH_PLL_USB_PRE		0
#endif


#define SYSCLK_FREQ		DT_PROP(DT_NODELABEL(rcc), clock_frequency)
#define DT_RCC_CLOCKS_CTRL	DT_CLOCKS_CTLR(DT_NODELABEL(rcc))

/* To enable use of IS_ENABLED utility macro, these symbols
 * should not be defined directly using DT_SAME_NODE.
 */
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_pll))
#define WCH_SYSCLK_SRC_PLL	1
#if !WCH_PLL_ENABLED
	#error "SYSCLK selects PLL, but PLL is disabled!"
#endif
#endif
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_hse))
#define WCH_SYSCLK_SRC_HSE	1
#if !WCH_HSE_ENABLED
	#error "SYSCLK selects HSE, but HSE is disabled!"
#endif
#endif
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_ck32k))
#define WCH_SYSCLK_SRC_CK32K	1
#if !WCH_CK32K_ENABLED
	#error "SYSCLK selects CK32K, but CK32K is disabled!"
#endif
#endif



#define R16_CLK_SYS_CFG		0x08

union clk_sys_cfg {
	struct {
		uint16_t clk_div : 5;
		uint16_t _rsvd1 : 1;
		uint16_t clk_sys_src : 2;
		uint16_t _rsvd2 : 8;
	};
	uint16_t val;
} __attribute__ ((packed));

enum clk_sys_cfg_div {
	SYSCLK_DIV_32 = 0,
	SYSCLK_STOP = 1,
	/* others */
};

enum clk_sys_cfg_src {
	SYSCLK_SRC_HSE = 0,
	SYSCLK_SRC_PLL = 1,
	SYSCLK_SRC_CK32K = 3,
};

#define R8_HFCK_PWR_CTRL	0x0A

union hfck_pwr_ctrl {
	struct {
		uint8_t _rsvd1 : 2;
		uint8_t clk_xt32m_pon : 1;
		uint8_t _rsvd2 : 1;
		uint8_t clk_pll_pon : 1;
		uint8_t _rsvd3 : 3;
	};
	uint8_t val;
} __attribute__ ((packed));

#define R16_INT32K_TUNE		0x2C

union int32k_tune {
	struct {
		uint16_t int32k_tune : 12;
		uint16_t _rsvd : 4;
	};
	uint16_t val;
} __attribute__ ((packed));

#define R8_XT32K_TUNE		0x2E

union xt32k_tune {
	struct {
		uint8_t xt32k_current_bias : 2;
		uint8_t _rsvd : 2;
		uint8_t xt32k_cap_load : 4;
	};
	uint8_t val;
} __attribute__ ((packed));

#define R8_CK32K_CONFIG		0x2F

union ck32k_config {
	struct {
		uint8_t xt32k_pon : 1;
		uint8_t int32k_pon : 1;
		uint8_t ck32k_src : 1;
		uint8_t _rsvd : 4;
		uint8_t xt32k_pin : 1;
	};
	uint8_t val;
} __attribute__ ((packed));

enum ck32k_src_type {
	CK32K_SRC_LSI = 0,
	CK32K_SRC_LSE = 0,
};

#define R8_PLL_CONFIG		0x4B

union pll_config {
	struct {
		uint8_t pll_cfg_data : 7;
		uint8_t _rsvd : 1;
	};
	uint8_t val;
} __attribute__ ((packed));

#define R8_XT32M_TUNE		0x4E

union xt32m_tune {
	struct {
		uint8_t xt32m_current_bias : 2;
		uint8_t _rsvd1 : 2;
		uint8_t xt32m_cap_laod : 3;
		uint8_t _rsvd2 : 1;
	};
	uint8_t val;
} __attribute__ ((packed));

/* ck32 calibration */
#define R16_OSC_CAL_CNT		0x50
#define R8_OSC_CAL_CTRL		0x52

union osc_cal_cnt {
	struct {
		uint16_t sysclk_cnt_in_5_ck32k_cycles : 14;
		uint16_t _rsvd : 2;
	};
	uint16_t val;
} __attribute__ ((packed));


union osc_cal_ctrl {
	struct {
		uint8_t osc_cnt_en : 1;
		uint8_t osc_cnt_halt : 1;
		uint8_t _rsvd : 6;
	};
	uint8_t val;
} __attribute__ ((packed));

/* Set corresponding bit to 1 to stop clock for a periphral and put it into
 * sleep mode.
 */
#define R16_SLP_CLK_OFF		0x0C

union sleep_clk_off {
	struct {
		uint16_t sleep_clk_tmr0 : 1;
		uint16_t sleep_clk_tmr1 : 1;
		uint16_t sleep_clk_tmr2 : 1;
		uint16_t sleep_clk_tmr3 : 1;
		uint16_t sleep_clk_uart0 : 1;
		uint16_t sleep_clk_uart1 : 1;
		uint16_t sleep_clk_uart2 : 1;
		uint16_t sleep_clk_uart3 : 1;

		uint16_t sleep_clk_spi0 : 1;
		uint16_t _rsvd1 : 1;
		uint16_t sleep_clk_pwm : 1;
		uint16_t _rsvd2 : 1;
		uint16_t sleep_clk_usb : 1;
		uint16_t _rsvd3 : 2;
		uint16_t sleep_clk_ble : 1;
	};
	uint16_t val;
};

enum sleep_clk_mask {
	SLEEP_CLK_TMR0 = (1 << 0),
	SLEEP_CLK_TMR1 = (1 << 1),
	SLEEP_CLK_TMR2 = (1 << 2),
	SLEEP_CLK_TMR3 = (1 << 3),
	SLEEP_CLK_UART0 = (1 << 4),
	SLEEP_CLK_UART1 = (1 << 5),
	SLEEP_CLK_UART2 = (1 << 6),
	SLEEP_CLK_UART3 = (1 << 7),
	SLEEP_CLK_SPI0 = (1 << 8),
	SLEEP_CLK_PWM = (1 << 10),
	SLEEP_CLK_USB = (1 << 12),
	SLEEP_CLK_BLE = (1 << 15),
};


/* write protected register in safe mode */
extern int syscon_write_prot_reg(const struct device *dev, uint16_t reg, uint32_t val, size_t len);

#endif /* DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_WCH_H_ */
