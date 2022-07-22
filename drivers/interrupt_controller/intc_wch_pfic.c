
#define DT_DRV_COMPAT wch_wch_pfic

#include "intc_wch_pfic.h"

#define REG_BASE	DT_INST_REG_ADDR(0)

static struct pfic *const pfic = (struct pfic *)REG_BASE;


void wch_pfic_irq_enable(uint32_t irq)
{
	pfic->ienr[irq >> 5] = BIT(irq & 0x1f);
}

void wch_pfic_irq_disable(uint32_t irq)
{
	uint32_t thresd = pfic->ithresdr;
	pfic->ithresdr = 0x10;
	pfic->irer[irq >> 5] = BIT(irq & 0x1f);
	pfic->ithresdr = thresd;
	arch_nop();
	arch_nop();
}

int wch_pfic_irq_is_enabled(uint32_t irq)
{
	uint32_t val = pfic->isr[irq >> 5];
	return !!(val & BIT(irq & 0x1f));
}

void arch_irq_enable(unsigned int irq)
{
	wch_pfic_irq_enable(irq);
}

void arch_irq_disable(unsigned int irq)
{
	wch_pfic_irq_disable(irq);
}

int arch_irq_is_enabled(unsigned int irq)
{
	return wch_pfic_irq_is_enabled(irq);
}

void wch_pfic_irq_priority_set(uint32_t irq, uint32_t prio)
{
	prio &= 0xff;

	uint32_t tmp = pfic->iprior[irq / 4];
	pfic->iprior[irq >> 2] = (tmp & ~(0xff << (irq & 0x3 * 8))) | (prio << (irq & 0x3 * 8));
}
