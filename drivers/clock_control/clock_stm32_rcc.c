#include "clock_stm32_rcc.h"

int stm32_clock_control_init(const struct device *dev)
{
	if (IS_ENABLED(CH32_HSE_ENABLED)) {
		
	}

}

/**
 * @brief RCC device, note that priority is intentionally set to 1 so
 * that the device init runs just after SOC init
 */
DEVICE_DT_DEFINE(DT_NODELABEL(rcc),
		    &stm32_clock_control_init,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &stm32_clock_control_api);
