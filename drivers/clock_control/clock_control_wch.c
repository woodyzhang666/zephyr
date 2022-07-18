
#define DT_DRV_COMPAT wch_ch57x_rcc

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys_clock.h>

#include "clock_control_wch.h"

#define WCH_CLOCK_REG_BASE \
	DT_REG_ADDR_BY_IDX(DT_PARENT(DT_NODELABEL(rcc)), 0)

struct wch_clock_config {
	uint32_t reg_base;
	const struct device *syscon;
};

static void	wait_clk_stable(void)
{
	volatile int loop = 1024*10;
	while (loop--)
		;
}


static int clock_control_wch_on(const struct device *dev, clock_control_subsys_t sys)
{
	int err = 0;
	const struct wch_clock_config *config = dev->config;
	const struct device *syscon = config->syscon;
	enum sleep_clk_mask sleep_clk_mask = (uint32_t)sys;

	union sleep_clk_off sleep_clk_off = *(union sleep_clk_off *)(WCH_CLOCK_REG_BASE + R16_SLP_CLK_OFF);
	sleep_clk_off.val |= (uint16_t)sleep_clk_mask;
	err = syscon_write_prot_reg(syscon, R16_SLP_CLK_OFF, sleep_clk_off.val, sizeof(sleep_clk_off));
	if (err)
		return err;
	wait_clk_stable();
	return err;
}

static int clock_control_wch_off(const struct device *dev, clock_control_subsys_t sys)
{
	int err = 0;
	const struct wch_clock_config *config = dev->config;
	const struct device *syscon = config->syscon;
	enum sleep_clk_mask sleep_clk_mask = (uint32_t)sys;

	union sleep_clk_off sleep_clk_off = *(union sleep_clk_off *)(WCH_CLOCK_REG_BASE + R16_SLP_CLK_OFF);
	sleep_clk_off.val &= ~(uint16_t)sleep_clk_mask;
	err = syscon_write_prot_reg(syscon, R16_SLP_CLK_OFF, sleep_clk_off.val, sizeof(sleep_clk_off));
	return err;
}

static int clock_control_wch_async_on(const struct device *dev,
					clock_control_subsys_t sys,
					clock_control_cb_t cb,
					void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	return -ENOTSUP;
}

static enum clock_control_status clock_control_wch_get_status(const struct device *dev,
								clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	enum sleep_clk_mask sleep_clk_mask = (uint32_t)sys;

	union sleep_clk_off sleep_clk_off = *(union sleep_clk_off *)(WCH_CLOCK_REG_BASE + R16_SLP_CLK_OFF);
	if (sleep_clk_off.val & sleep_clk_mask)
		return CLOCK_CONTROL_STATUS_OFF;
	else
		return CLOCK_CONTROL_STATUS_ON;
}

static int clock_control_wch_get_rate(const struct device *dev,
					clock_control_subsys_t sub_system,
					uint32_t *rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sub_system);

	*rate = SYSCLK_FREQ;

	return 0;
}

static const struct clock_control_driver_api clock_control_wch_api = {
	.on = clock_control_wch_on,
	.off = clock_control_wch_off,
	.async_on = clock_control_wch_async_on,
	.get_rate = clock_control_wch_get_rate,
	.get_status = clock_control_wch_get_status,
};

static int switch_to_lsi(const struct device *dev)
{
	int err = 0;
	const struct wch_clock_config *config = dev->config;
	const struct device *syscon = config->syscon;

	union ck32k_config ck32k_config = *(union ck32k_config *)(WCH_CLOCK_REG_BASE + R8_CK32K_CONFIG);
	if (!ck32k_config.int32k_pon) {
		ck32k_config.int32k_pon = True;
		err = syscon_write_prot_reg(syscon, R8_CK32K_CONFIG, ck32k_config.val, sizeof(ck32k_config));
		if (err)
			return err;
		wait_clk_stable();
	}
	ck32k_config.ck32k_src = CK32K_SRC_LSI;
	err = syscon_write_prot_reg(syscon, R8_CK32K_CONFIG, ck32k_config.val, sizeof(ck32k_config));
	if (err)
		return err;

	union clk_sys_cfg syscfg = *(union clk_sys_cfg *)(WCH_CLOCK_REG_BASE + R16_CLK_SYS_CFG);
	syscfg.clk_sys_src = SYSCLK_SRC_CK32K;
	err = syscon_write_prot_reg(syscon, R16_CLK_SYS_CFG, syscfg.val, sizeof(syscfg));
	return err;
}

static int config_clock_sources(const struct device *dev)
{
	int err = 0;
	const struct wch_clock_config *config = dev->config;
	const struct device *syscon = config->syscon;

	union ck32k_config ck32k_config = *(union ck32k_config *)(WCH_CLOCK_REG_BASE + R8_CK32K_CONFIG);
#if WCH_LSE_ENABLED
	if (!ck32k_config.xt32k_pon) {
		ck32k_config.xt32k_pon = True;
		err = syscon_write_prot_reg(syscon, R8_CK32K_CONFIG, ck32k_config.val, sizeof(ck32k_config));
		if (err)
			return err;
		wait_clk_stable();
	}
#if WCH_CK32K_SRC_LSE
	ck32k_config.ck32k_src = CK32K_SRC_LSE;
	err = syscon_write_prot_reg(syscon, R8_CK32K_CONFIG, ck32k_config.val, sizeof(ck32k_config));
	if (err)
		return err;
#endif
#else
	ck32k_config.xt32k_pon = False;
	err = syscon_write_prot_reg(syscon, R8_CK32K_CONFIG, ck32k_config.val, sizeof(ck32k_config));
	if (err)
		return err;
#endif

	union hfck_pwr_ctrl hfck_pwr_ctrl = *(union hfck_pwr_ctrl *)(WCH_CLOCK_REG_BASE + R8_HFCK_PWR_CTRL);
#if WCH_HSE_ENABLED
	if (!hfck_pwr_ctrl.clk_xt32m_pon) {
		hfck_pwr_ctrl.clk_xt32m_pon = True;
		err = syscon_write_prot_reg(syscon, R8_HFCK_PWR_CTRL, hfck_pwr_ctrl.val, sizeof(hfck_pwr_ctrl));
		if (err)
			return err;
		wait_clk_stable();
	}
#else
	hfck_pwr_ctrl.clk_xt32m_pon = False;
	err = syscon_write_prot_reg(syscon, R8_HFCK_PWR_CTRL, hfck_pwr_ctrl.val, sizeof(hfck_pwr_ctrl));
	if (err)
		return err;
#endif

	/* mul is immutable as no vendor specification */
	hfck_pwr_ctrl = *(union hfck_pwr_ctrl *)(WCH_CLOCK_REG_BASE + R8_HFCK_PWR_CTRL);
#if WCH_PLL_ENABLED
	if (!hfck_pwr_ctrl.clk_pll_pon) {
		hfck_pwr_ctrl.clk_pll_pon = True;
		err = syscon_write_prot_reg(syscon, R8_HFCK_PWR_CTRL, hfck_pwr_ctrl.val, sizeof(hfck_pwr_ctrl));
		if (err)
			return err;
		wait_clk_stable();
	}
#else
	hfck_pwr_ctrl.clk_pll_pon = False;
	err = syscon_write_prot_reg(syscon, R8_HFCK_PWR_CTRL, hfck_pwr_ctrl.val, sizeof(hfck_pwr_ctrl));
	if (err)
		return err;
#endif

	return err;
}

static int config_sys_clock(const struct device *dev)
{
	int err = 0;
	const struct wch_clock_config *config = dev->config;
	const struct device *syscon = config->syscon;

#if WCH_SYSCLK_SRC_CK32K
	/* We are already using ck32k now */
#elif WCH_SYSCLK_SRC_HSE
	union clk_sys_cfg syscfg = *(union clk_sys_cfg *)(WCH_CLOCK_REG_BASE + R16_CLK_SYS_CFG);
	syscfg.clk_sys_src = SYSCLK_SRC_HSE;
	BUILD_ASSERT((WCH_HSE_FREQ % SYSCLK_FREQ == 0), "SYSCLK can't be divided by HSE");
	BUILD_ASSERT((WCH_HSE_FREQ / SYSCLK_FREQ <= 32 && WCH_HSE_FREQ / SYSCLK_FREQ >= 2), "Invalid divisor");
	uint16_t divisor = WCH_HSE_FREQ / SYSCLK_FREQ;
	if (divisor == 32)
		divisor = 0;
	syscfg.clk_div = divisor;
	err = syscon_write_prot_reg(syscon, R16_CLK_SYS_CFG, syscfg.val, sizeof(syscfg));
#elif WCH_SYSCLK_SRC_PLL
	union clk_sys_cfg syscfg = *(union clk_sys_cfg *)(WCH_CLOCK_REG_BASE + R16_CLK_SYS_CFG);
	syscfg.clk_sys_src = SYSCLK_SRC_PLL;
	BUILD_ASSERT((WCH_HSE_FREQ * WCH_PLL_MUL % SYSCLK_FREQ == 0), "SYSCLK freq can't be divided by PLL");
	BUILD_ASSERT((WCH_HSE_FREQ * WCH_PLL_MUL / SYSCLK_FREQ <= 32 && 
			WCH_HSE_FREQ * WCH_PLL_MUL / SYSCLK_FREQ >= 0), "Invalid divisor");
	uint16_t divisor = WCH_HSE_FREQ * WCH_PLL_MUL / SYSCLK_FREQ;
	if (divisor == 32)
		divisor = 0;
	syscfg.clk_div = divisor;
	err = syscon_write_prot_reg(syscon, R16_CLK_SYS_CFG, syscfg.val, sizeof(syscfg));
#endif

	return err;
}

/* 
 * vendor bootloader has touched clock settings. We should rewind back to a
 * defined state.
 */
static int clock_control_wch_init(const struct device *dev)
{
	int err = 0;
	const struct wch_clock_config *config = dev->config;
	const struct device *syscon = config->syscon;
	
	if (!z_device_is_ready(syscon))
		while(1) {;}

#if 0	/* can't switch to lsi, otherwise swd will fail to connect */
	err = switch_to_lsi(dev);
	if (err)
		return err;
#endif

	err = config_clock_sources(dev);
	if (err)
		return err;

	err = config_sys_clock(dev);

	return err;
}

static const struct wch_clock_config wch_clock_config = {
	.reg_base = WCH_CLOCK_REG_BASE,
	.syscon = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(rcc))),
};

DEVICE_DT_DEFINE(DT_NODELABEL(rcc),
		 &clock_control_wch_init,
		 NULL,
		 NULL,
		 &wch_clock_config,
		 PRE_KERNEL_1,
		 CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		 &clock_control_wch_api);

BUILD_ASSERT(((CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) == SYSCLK_FREQ),
		    "SYS_CLOCK_HW_CYCLES_PER_SEC Value must be equal to SYSCLK freq");
