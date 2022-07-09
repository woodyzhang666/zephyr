
#include <offsets.h>
#include <zephyr/toolchain.h>

/* Exports */
GTEXT(__soc_is_irq)
GTEXT(__soc_handle_irq)
GTEXT(nmi_handler)
GTEXT(hardfault_handler)
GTEXT(systick_handler)
GTEXT(sw_handler)

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

