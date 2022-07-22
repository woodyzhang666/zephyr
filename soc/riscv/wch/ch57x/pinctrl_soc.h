
#ifndef SOC_RISCV_WCH_CH57X_PINCTRL_SOC_H_
#define SOC_RISCV_WCH_CH57X_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

typedef struct pinctrl_soc_pin {
	/** Pinmux settings (port, pin and function). */
	uint32_t pinmux;
	/** Pin configuration (bias, drive and slew rate). */
	uint32_t pincfg;
} pinctrl_soc_pin_t;


#endif /* SOC_RISCV_WCH_CH57X_PINCTRL_SOC_H_ */
