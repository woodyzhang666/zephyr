
#define DT_DRV_COMPAT wch_ch57x_syscon

#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>

#include "syscon_ch57x.h"

struct syscon_ch57x_config {
	DEVICE_MMIO_ROM;
};

int syscon_write_prot_reg(const struct device *dev, uint16_t reg, uint32_t val, size_t len)
{
	uintptr_t base_address;
	int ret = 0;

	base_address = DEVICE_MMIO_GET(dev);

	sys_write8(SAFE_ACCESS_SIG_1, base_address + SAFE_ACCESS_SIG_OFF);
	sys_write8(SAFE_ACCESS_SIG_2, base_address + SAFE_ACCESS_SIG_OFF);

	switch (len) {
		case 1:
			sys_write8(val, base_address + reg);
			break;
		case 2:
			sys_write16(val, base_address + reg);
			break;
		case 4:
			sys_write32(val, base_address + reg);
			break;
		default:
			ret = -EINVAL;
	}

	sys_write8(SAFE_ACCESS_EXIT, base_address + SAFE_ACCESS_SIG_OFF);

	return ret;
}

/* pll cfg is propritory */
static int syscon_ch57x_init(const struct device *dev)
{
#if 0
	/* enable pll power, power of external osc is enabled by default */
	uint8_t pwr_ctl = (RB_CLK_PLL_ON_MSK << RB_CLK_PLL_ON_POS) | \
					  (RB_CLK_XT32M_PON_MSK << RB_CLK_XT32M_PON_POS);
	syscon_write_prot_reg(dev, R8_HFCK_PWR_CTRL, pwr_ctl, sizeof(pwr_ctl));

	/* need delay ? */

	/* 480 / 24 = 20M */
	uint16_t clk_sel = (24 << RB_CLK_PLL_DIV_POS) | \
					(RB_CLK_SYS_MOD_PLL << RB_CLK_SYS_MOD_POS);
	syscon_write_prot_reg(dev, R16_CLK_SYS_CFG, clk_sel, sizeof(clk_sel));
#endif

#if 1
	/* rx: pa8, tx: pa9 */
	volatile uint32_t *addr;
	addr= (volatile uint32_t *)0x400010a8;	/* out 1 first */
	*addr |= (1U << 9);
	addr= (volatile uint32_t *)0x400010a0;	/* dir */
	*addr |= (1U << 9);
	addr= (volatile uint32_t *)0x400010b0;	/* pu */
	*addr |= (1U << 8);

	addr= (volatile uint32_t *)0x400010a8;	/* out 1 first */
	*addr |= (1U << 5);
	addr= (volatile uint32_t *)0x400010a0;	/* dir */
	*addr |= (1U << 5);
	addr= (volatile uint32_t *)0x400010b0;	/* pu */
	*addr |= (1U << 4);

	addr= (volatile uint32_t *)0x400010a8;	/* out 1 first */
	*addr |= (1U << 6);
	addr= (volatile uint32_t *)0x400010a0;	/* dir */
	*addr |= (1U << 6);

	addr= (volatile uint32_t *)0x400010a8;	/* out 1 first */
	*addr |= (1U << 7);
	addr= (volatile uint32_t *)0x400010a0;	/* dir */
	*addr |= (1U << 7);

	/* map gpiob int to pb22, pb23 instead of pb8, pb9 */
	*(volatile uint16_t *)0x40001018 |= BIT(13);

    /* enable usb pins */
    *(volatile uint16_t *)0x4000101a |= BIT(7);
#endif


#if 0
	/* set uart0 pin (tx: pb7, rx: pb4) dir, rx pull up */
	addr= (volatile uint32_t *)0x400010c0;
	*addr |= (1U << 7);
	addr = (volatile uint32_t *)0x400010d0;
	*addr |= (1U << 4);
#endif

	return 0;
}

#define SYSCON_INIT(inst) \
	static const struct syscon_ch57x_config syscon_ch57x_config_##inst = {                 \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, syscon_ch57x_init, NULL, NULL,        \
			      &syscon_ch57x_config_##inst, PRE_KERNEL_1,                         \
			      CONFIG_SYSCON_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SYSCON_INIT);
