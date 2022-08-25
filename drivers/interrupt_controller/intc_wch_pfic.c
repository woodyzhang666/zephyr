
#define DT_DRV_COMPAT wch_wch_pfic

#include "intc_wch_pfic.h"

#define REG_BASE	DT_INST_REG_ADDR(0)

static struct pfic *const pfic = (struct pfic *)REG_BASE;


void wch_pfic_irq_enable(uint32_t irq)
{
	pfic->ienr[irq >> 5] = BIT(irq & 0x1F);
}

void wch_pfic_irq_disable(uint32_t irq)
{
	uint32_t thresd = pfic->ithresdr;
	pfic->ithresdr = 0x10;
	pfic->irer[irq >> 5] = BIT(irq & 0x1F);
	pfic->ithresdr = thresd;
	arch_nop();
	arch_nop();
}

int wch_pfic_irq_is_enabled(uint32_t irq)
{
	uint32_t val = pfic->isr[irq >> 5];
	return !!(val & BIT(irq & 0x1F));
}

void arch_irq_enable(uint32_t  irq)
{
	wch_pfic_irq_enable(irq);
}

void arch_irq_disable(uint32_t irq)
{
	wch_pfic_irq_disable(irq);
}

int arch_irq_is_enabled(uint32_t irq)
{
	return wch_pfic_irq_is_enabled(irq);
}

void wch_pfic_irq_priority_set(uint32_t irq, uint32_t prio)
{
	uint8_t _prio = prio & 0xff;

	pfic->iprior[irq] = _prio;
}

void wch_pfic_irq_set_pending(uint32_t irq)
{
	pfic->ipsr[irq >> 5] = BIT(irq & 0x1F);
}

void wch_pfic_irq__clear_pending(uint32_t irq)
{
	pfic->iprr[irq >> 5] = BIT(irq & 0x1F);
}
