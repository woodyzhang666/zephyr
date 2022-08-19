
#include <offsets.h>
#include <zephyr/toolchain.h>

/* Exports */
GTEXT(__soc_is_irq)
GTEXT(__soc_handle_irq)
GTEXT(nmi_handler)
GTEXT(hardfault_handler)
GTEXT(systick_handler)
GTEXT(sw_handler)
GTEXT(__usb_irq_wrapper)
GDATA(_sw_isr_table)

SECTION_FUNC(exception.other, __soc_is_irq)
	csrr a0, mcause
	srli a0, a0, 31
	ret

SECTION_FUNC(exception.other, __soc_handle_irq)
	ret

//TODO
SECTION_FUNC(exception.other, nmi_handler)
1:  j 1b

SECTION_FUNC(exception.other, hardfault_handler)
1:  j 1b

SECTION_FUNC(exception.other, sw_handler)
1:  j 1b

SECTION_FUNC(ramfunc, __usb_irq_wrapper)
	la t0, _sw_isr_table
	li a0, 22 << (2 + 1)
	add t0, t0, a0

	/* Load argument in a0 register */
	lw a0, 0(t0)

	/* Load ISR function address in register t1 */
	lw t1, 4(t0)

	jalr ra, t1, 0
	mret
