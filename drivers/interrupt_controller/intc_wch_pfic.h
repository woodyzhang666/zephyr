
#ifndef DRIVERS_INTERRUPT_CONTROLLER_INTC_WCH_PFIC_H_
#define DRIVERS_INTERRUPT_CONTROLLER_INTC_WCH_PFIC_H_

#include <zephyr/kernel.h>
#include <zephyr/irq.h>

#define PFIC_VECTOR_NUM		CONFIG_NUM_IRQS
#define VECTOR_ARRAY_LEN	((PFIC_VECTOR_NUM + 31) / 32)
#define PRIORITY_REG_COUNT	((PFIC_VECTOR_NUM + 3) / 4)

struct pfic {
	volatile uint32_t isr[VECTOR_ARRAY_LEN];		/* 0x00, bit12-31 for #12-31 */
	volatile uint32_t _rsvd1[8-VECTOR_ARRAY_LEN];
	volatile uint32_t ipr[VECTOR_ARRAY_LEN];		/* 0x20 */
	volatile uint32_t _rsvd2[8-VECTOR_ARRAY_LEN];
	volatile uint32_t ithresdr;		/* 0x40,  */
	volatile uint32_t fibaddrr;		/* 0x44, Fast Interrupt Base Address */
	volatile uint32_t cfgr;			/* 0x48 */
	volatile uint32_t gisr;			/* 0x4C */
	volatile uint32_t _rsvd3[4];
	volatile uint32_t fiofaddrr0;	/* 0x60, fast irq 0 offset */
	volatile uint32_t fiofaddrr1;	/* 0x64, fast irq 1 offset */
	volatile uint32_t fiofaddrr2;	/* 0x68, fast irq 2 offset */
	volatile uint32_t fiofaddrr3;	/* 0x6C, fast irq 3 offset */
	volatile uint32_t _rsvd4[36];
	volatile uint32_t ienr[VECTOR_ARRAY_LEN];	/* 0x100, Interrupt Enable Reg */
	volatile uint32_t _rsvd5[32-VECTOR_ARRAY_LEN];
	volatile uint32_t irer[VECTOR_ARRAY_LEN];		/* 0x180, Interrupt Remove enable Reg */
	volatile uint32_t _rsvd6[32-VECTOR_ARRAY_LEN];
	volatile uint32_t ipsr[VECTOR_ARRAY_LEN];		/* 0x200, Interrupt Pending Set Reg */
	volatile uint32_t _rsvd7[32-VECTOR_ARRAY_LEN];
	volatile uint32_t iprr[VECTOR_ARRAY_LEN];		/* 0x280, Interrupt Pending Remove Reg */
	volatile uint32_t _rsvd8[32-VECTOR_ARRAY_LEN];
	volatile uint32_t iactr[VECTOR_ARRAY_LEN];		/* 0x300, Interrupt Activate Reg */
	volatile uint32_t _rsvd9[64-VECTOR_ARRAY_LEN];
	volatile uint8_t  iprior[PFIC_VECTOR_NUM];	/* 0x400, Interrupt Priority Reg, 8bit for one interrupt */
	volatile uint32_t _rsvd10[580-PRIORITY_REG_COUNT];
	volatile uint32_t sctlr;					/* 0xd10, System Control Reg */
	volatile uint32_t vtctlr;				/* 0xd14, Vector Table Control Reg */
} __attribute__ ((packed));

BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->isr[0]) == 0x00);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->ipr[0]) == 0x20);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->ithresdr) == 0x40);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->fiofaddrr0) == 0x60);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->ienr[0]) == 0x100);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->irer[0]) == 0x180);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->ipsr[0]) == 0x200);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->iprr) == 0x280);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->iactr[0]) == 0x300);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->iprior[0]) == 0x400);
BUILD_ASSERT((uint32_t)(&((struct pfic *)0)->sctlr) == 0xd10);

#endif /* DRIVERS_INTERRUPT_CONTROLLER_INTC_WCH_PFIC_H_ */
