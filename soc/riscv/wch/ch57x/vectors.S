
#include <zephyr/toolchain.h>

/* Imports */
GTEXT(__irq_wrapper)
GTEXT(__initialize)


.global  __start
.section .vectors.start
.balign   0x100
.type     __start, @function
 
__start:
.option push
.option norvc
.option norelax
    li t0, 0x88
    csrs mstatus, t0
    la t0, vector_base
    ori t0, t0, 1
    csrw mtvec, t0
	tail   __initialize

.global  vector_base
.section .vectors.exceptions
.balign   0x10
.type     vector_base, @function
 
vector_base:
.option push
.option norvc
.option norelax
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
        j   systick_isr				/* SysTick Handler */
    .word   0
        j   sw_handler					/* SW Handler */
    .word   0
    /* External Interrupts */
	.rept (20)
		j	__irq_wrapper		/* 20 identical entries, all pointing to the interrupt handler */
	.endr
