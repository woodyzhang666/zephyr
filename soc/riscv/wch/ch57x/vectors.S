
#include <zephyr/toolchain.h>
#include <soc.h>

/* Imports */
GTEXT(__irq_wrapper)
GTEXT(__initialize)

/* Exports */
GTEXT(__start)
GTEXT(vector_base)

.section .vectors.start, "ax"
.balign 0x4
.option norvc
__start:
    li t0, 0x1888	/* stay machine level after mret */
    csrs mstatus, t0

	/* defconfig for pipeline, branch prediction */
    li t0, 0x1f
    csrw CORECFGR, t0

    li t0, HWSTKEN | INESTEN /* only this two bits are supported */
    csrw INTSYSCR, t0

    la t0, vector_base
    ori t0, t0, 3		/* vectored, ISR addresses */
    csrw mtvec, t0
	la t0, __initialize
	csrw mepc, t0
	mret

.balign   0x10
vector_base:
	.word	0
	.word	0
	.word	nmi_handler
    .word   hardfault_handler			/* Hard Fault Handler */
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
	.word	__wch_irq_wrapper		/* systick */
    .word   0
	.word	__sw_handler /* software triggerred interrupt */
    .word   0

    /* External Interrupts */
	.rept (20)
	.word	__wch_irq_wrapper		/* 20 identical entries, all pointing to the interrupt handler */
	.endr

