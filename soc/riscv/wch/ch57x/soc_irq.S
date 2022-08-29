
#include <offsets.h>
#include <zephyr/toolchain.h>
#include <soc.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/syscall.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/arch/riscv/syscall.h>

#define WCH_DEBUG_GPIO

/* Exports */
GTEXT(nmi_handler)
GTEXT(hardfault_handler)
GTEXT(__sw_handler)
GTEXT(__wch_irq_wrapper)
GTEXT(z_riscv_thread_start)

/* Imports */
GDATA(_sw_isr_table)
GDATA(__irq_count)

#define DO_CALLER_SAVED_T0T1(op) \
	op t0, __z_arch_esf_t_t0_OFFSET(sp)		;\
	op t1, __z_arch_esf_t_t1_OFFSET(sp)

#define DO_CALLER_SAVED_REST(op) \
	op t2, __z_arch_esf_t_t2_OFFSET(sp)		;\
	op t3, __z_arch_esf_t_t3_OFFSET(sp)		;\
	op t4, __z_arch_esf_t_t4_OFFSET(sp)		;\
	op t5, __z_arch_esf_t_t5_OFFSET(sp)		;\
	op t6, __z_arch_esf_t_t6_OFFSET(sp)		;\
	op a0, __z_arch_esf_t_a0_OFFSET(sp)		;\
	op a1, __z_arch_esf_t_a1_OFFSET(sp)		;\
	op a2, __z_arch_esf_t_a2_OFFSET(sp)		;\
	op a3, __z_arch_esf_t_a3_OFFSET(sp)		;\
	op a4, __z_arch_esf_t_a4_OFFSET(sp)		;\
	op a5, __z_arch_esf_t_a5_OFFSET(sp)		;\
	op a6, __z_arch_esf_t_a6_OFFSET(sp)		;\
	op a7, __z_arch_esf_t_a7_OFFSET(sp)		;\
	op tp, __z_arch_esf_t_tp_OFFSET(sp)             ;\
	op ra, __z_arch_esf_t_ra_OFFSET(sp)

#define DO_CALLEE_SAVED(op, reg) \
	op ra, _thread_offset_to_ra(reg)	;\
	op tp, _thread_offset_to_tp(reg)	;\
	op s0, _thread_offset_to_s0(reg)	;\
	op s1, _thread_offset_to_s1(reg)	;\
	op s2, _thread_offset_to_s2(reg)	;\
	op s3, _thread_offset_to_s3(reg)	;\
	op s4, _thread_offset_to_s4(reg)	;\
	op s5, _thread_offset_to_s5(reg)	;\
	op s6, _thread_offset_to_s6(reg)	;\
	op s7, _thread_offset_to_s7(reg)	;\
	op s8, _thread_offset_to_s8(reg)	;\
	op s9, _thread_offset_to_s9(reg)	;\
	op s10, _thread_offset_to_s10(reg)	;\
	op s11, _thread_offset_to_s11(reg)

#define GET_CURRENT_CPU(dst, tmp) \
	la dst, _kernel + ___kernel_t_cpus_OFFSET

SECTION_FUNC(ramfunc, nmi_handler)
1:  j 1b

SECTION_FUNC(ramfunc, hardfault_handler)
1:  j 1b

SECTION_FUNC(ramfunc, __sw_handler_switch)
	/* Get pointer to current thread on this CPU */
	lw a1, ___cpu_t_current_OFFSET(s0)

	/*
	 * Get next thread to schedule with z_get_next_switch_handle().
	 * We pass it a NULL as we didn't save the whole thread context yet.
	 * If no scheduling is necessary then NULL will be returned.
	 */
	addi sp, sp, -16
	sw a1, 0(sp)
	mv a0, zero
	call z_get_next_switch_handle
	lw a1, 0(sp)
	addi sp, sp, 16
	beqz a0, no_switch

	/*
	 * Perform context switch:
	 * a0 = new thread
	 * a1 = old thread
	 */
	call z_riscv_switch


z_riscv_thread_start:
no_switch:

	/* Restore MEPC register */
	lw t0, __z_arch_esf_t_mepc_OFFSET(sp)
	csrw mepc, t0

	/* Restore MSTATUS register */
	lw t2, __z_arch_esf_t_mstatus_OFFSET(sp)
	csrrw t0, mstatus, t2

	/* Restore s0 (it is no longer ours) */
	lw s0, __z_arch_esf_t_s0_OFFSET(sp)

	/* Restore caller-saved registers from thread stack */
	DO_CALLER_SAVED_T0T1(lw)
	DO_CALLER_SAVED_REST(lw)

	addi sp, sp, __z_arch_esf_t_SIZEOF

	mret	/* no hw stack to pop */

SECTION_FUNC(ramfunc, __sw_handler)

	/* Save caller-saved registers on current thread stack. */
	addi sp, sp, -__z_arch_esf_t_SIZEOF
	DO_CALLER_SAVED_T0T1(sw)		;
	DO_CALLER_SAVED_REST(sw)		;

	/* Save s0 in the esf and load it with &_current_cpu. */
	sw s0, __z_arch_esf_t_s0_OFFSET(sp)
	GET_CURRENT_CPU(s0, t0)

	/* Save MEPC register */
	csrr t0, mepc
	sw t0, __z_arch_esf_t_mepc_OFFSET(sp)

	/* Save MSTATUS register */
	csrr t2, mstatus
	sw t2, __z_arch_esf_t_mstatus_OFFSET(sp)

    /* Load mret with the address of the next instruction in the task to run next. */
    la t0, __sw_handler_switch
    csrw mepc, t0

	csrrc a2, mstatus, 0x8
	nop
	nop
	nop

	/* hw stack is poped out */
	mret


/*
 * Handler called upon each exception/interrupt/fault
 *
 * Hw save caller saved regs and mepc, mstatus (?) to internal memory and
 * restore them on mret
 */
SECTION_FUNC(ramfunc, __wch_irq_wrapper)
#ifdef WCH_DEBUG_GPIO
	csrrc a1, mstatus, 0x8	/* irq lock */
	nop
	nop
	nop

	/* pull down gpio */
	li t0, 0x400010a8
	lw t1, 0(t0)
	li t2, 0x20
	and t2, t2, t1
	beqz t2, pull_down_gpio6
	li t2, 0xFFFFFFDF
	j 1f
pull_down_gpio6:
	li t2, 0xFFFFFFBF
1:
	and t1, t1, t2
	sw t1, 0(t0)
	csrw mstatus, a1		/* irq unlock */
#endif

	/* Save caller-saved registers on current thread stack. */
	addi sp, sp, -__z_arch_esf_t_SIZEOF

	/* Save s0 in the esf and load it with &_current_cpu. */
	sw s0, __z_arch_esf_t_s0_OFFSET(sp)
	GET_CURRENT_CPU(s0, t0)

	csrrci a1, mstatus, 0x8	/* irq lock */
	nop
	nop
	nop

	/* Increment _current_cpu->nested */
	lw t1, ___cpu_t_nested_OFFSET(s0)
	addi t2, t1, 1
	sw t2, ___cpu_t_nested_OFFSET(s0)
	bnez t1, on_irq_stack

	/* Switch to interrupt stack */
	mv t0, sp
	lw sp, ___cpu_t_irq_stack_OFFSET(s0)

	/*
	 * Save thread stack pointer on interrupt stack
	 * In RISC-V, stack pointer needs to be 16-byte aligned
	 */
	addi sp, sp, -16
	sw t0, 0(sp)

on_irq_stack:
	csrw mstatus, a1		/* irq unlock */

#ifdef CONFIG_TRACING_ISR
	call sys_trace_isr_enter
#endif

	/* Get IRQ number */
	csrr a0, mcause
	li t0, SOC_MCAUSE_EXP_MASK
	and a0, a0, t0

#if 1
	/* irq accounting */
	la t0, __irq_count
	slli a1, a0, 2
	add t0, t0, a1
	lw t1, 0(t0)
	addi t1, t1, 1
	sw t1, 0(t0)
#endif

	/*
	 * Call corresponding registered function in _sw_isr_table.
	 * (table is 2-word wide, we should shift index accordingly)
	 */
	la t0, _sw_isr_table
	slli a0, a0, 3
	add t0, t0, a0

	/* Load argument in a0 register */
	lw a0, 0(t0)
	/* Load ISR function address in register t1 */
	lw t1, 4(t0)

	/* Call ISR function */
	jalr ra, t1, 0

#ifdef CONFIG_TRACING_ISR
    call sys_trace_isr_exit
#endif

irq_done:
	/* Decrement _current_cpu->nested */
	csrrci a1, mstatus, 0x8	/* irq lock */
	nop
	nop
	nop
	lw t2, ___cpu_t_nested_OFFSET(s0)
	addi t2, t2, -1
	sw t2, ___cpu_t_nested_OFFSET(s0)
	beqz t2, switch_to_thread_stack

	csrw mstatus, a1		/* irq unlock */
	j no_reschedule

switch_to_thread_stack:
	/* nested count is back to 0: Return to thread stack */
	lw sp, 0(sp)
	csrw mstatus, a1		/* irq unlock */

#ifdef CONFIG_STACK_SENTINEL
	call z_check_stack_sentinel
#endif

reschedule:

	/* pend sw interrupt which has lowest prio */
	call pend_sw_int

no_reschedule:

	lw s0, __z_arch_esf_t_s0_OFFSET(sp)

	/* remove esf from the stack */
	addi sp, sp, __z_arch_esf_t_SIZEOF

#ifdef WCH_DEBUG_GPIO
	csrrci a1, mstatus, 0x8	/* irq lock */
	nop
	nop
	nop

	/* pull up gpio */
	li t0, 0x400010a8
	lw t1, 0(t0)
	li t2, 0x40
	and t2, t2, t1
	beqz t2, pull_up_gpio6
	ori t1, t1, 0x20
	j 1f
pull_up_gpio6:
	ori t1, t1, 0x40
1:
	sw t1, 0(t0)

	csrw mstatus, a1		/* irq unlock */
#endif

	mret

