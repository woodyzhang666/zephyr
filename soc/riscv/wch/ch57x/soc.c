
#include <zephyr/kernel.h>

extern void wch_pfic_irq_enable(uint32_t irq);
extern void wch_pfic_irq_priority_set(uint32_t irq, uint32_t prio);
extern void wch_pfic_irq_set_pending(uint32_t irq);

#if 1
void soc_interrupt_init(void)
{
	wch_pfic_irq_priority_set(WCH_SW_IRQ_NUM, 0x8F);	/* lowest pending and preempt */
	wch_pfic_irq_enable(WCH_SW_IRQ_NUM);
}

#endif

void pend_sw_int(void)
{
	wch_pfic_irq_set_pending(WCH_SW_IRQ_NUM);
}

uint32_t __irq_count[36];
