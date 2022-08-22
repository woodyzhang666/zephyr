
#include <zephyr/kernel.h>

#if 0
void soc_interrupt_init(void)
{
	EVENT_UNIT->INTPTPENDCLEAR = 0xFFFFFFFF;
	(void)(EVENT_UNIT->INTPTPENDCLEAR); /* Ensures write has finished. */
	EVENT_UNIT->EVTPENDCLEAR   = 0xFFFFFFFF;
	(void)(EVENT_UNIT->EVTPENDCLEAR); /* Ensures write has finished. */

	if (IS_ENABLED(CONFIG_MULTI_LEVEL_INTERRUPTS)) {
		dev_intmux = DEVICE_DT_GET(DT_INST(0, openisa_rv32m1_intmux));
	}
}

#endif

uint32_t __irq_count[36];
