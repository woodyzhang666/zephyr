
#include <zephyr/toolchain.h>

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
    la t0, vector_base
    ori t0, t0, 1
    csrw mtvec, t0
	la t0, __initialize
	csrw mepc, t0
	mret		/* pfic will pop stack, sp should be checked */

.balign   0x10
vector_base:
	.word	0
	.word	0
		j	nmi_handler
        j   hardfault_handler			/* Hard Fault Handler */
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0

    /* External Interrupts */
	.rept (24)
		j	__irq_wrapper		/* 20 identical entries, all pointing to the interrupt handler */
	.endr
