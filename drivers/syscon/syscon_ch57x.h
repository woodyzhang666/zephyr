
#ifndef DRIVERS_SYSCON_SYSCON_CH57X_H_
#define DRIVERS_SYSCON_SYSCON_CH57X_H_

/* RWA */
#define R16_CLK_SYS_CFG					0x08
#define RB_CLK_PLL_DIV_POS				0
#define RB_CLK_PLL_DIV_MSK				0x1F
#define RB_CLK_SYS_MOD_POS				6
#define RB_CLK_SYS_MOD_MSK				0x3
#define RB_CLK_SYS_MOD_HSE				0x0
#define RB_CLK_SYS_MOD_PLL				0x1
#define RB_CLK_SYS_MOD_32K				0x3

/* RWA */
#define R8_HFCK_PWR_CTRL				0x0A
#define RB_CLK_XT32M_PON_POS			2
#define RB_CLK_XT32M_PON_MSK			0x1
#define RB_CLK_PLL_ON_POS				4
#define RB_CLK_PLL_ON_MSK				0x1


/* write two sig to enable security access in next 16 sys clocks. Write 0 to
 * exit right now. */
#define SAFE_ACCESS_SIG_OFF				0x40
#define SAFE_ACCESS_SIG_1				0x57
#define SAFE_ACCESS_SIG_2				0xA8
#define SAFE_ACCESS_EXIT				0x00

#define CHIP_ID_OFF						0x41
#define SAFE_ACCESS_ID_OFF				0x42
#define WDOG_COUNT_OFF					0x43
#define RESET_STATUS_FLASH_CFG			0x44
#define GLOB_CFG_INFO					0x45
#define WDOG_RESET_CTRL					0x46
#define GLOB_RESET_KEEP					0x47

#endif /* DRIVERS_SYSCON_SYSCON_CH57X_H_ */
