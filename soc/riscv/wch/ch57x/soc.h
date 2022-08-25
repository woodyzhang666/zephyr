
#ifndef __SOC_H__
#define __SOC_H__

#define SOC_MCAUSE_USER_ECALL_EXP 8 /* User ECALL instruction */
#define SOC_MCAUSE_ECALL_EXP 11 /* Machine ECALL instruction */

#define WCH_SW_IRQ_NUM		14

#define SOC_MCAUSE_EXP_MASK 0x7FFFFFFF

#define SOC_ERET mret

/* QingKe-V4 Vendor CSRs */
#define GINTENR			0x800
#define HWSTKEN				0x00000001
#define INESTEN				0x00000002
#define MASK_PMTCFG			0x0000000C
#define POS_PMTCFG			2
#define PMTCFG_1_LEVEL		0x0
#define PMTCFG_2_LEVEL		0x1
#define PMTCFG_4_LEVEL		0x2
#define PMTCFG_8_LEVEL		0x3
#define HWSTKOVEN			0x00000010
#define GIHWSTKNEN			0x00000020
#define MASK_PMTSTA			0x0000FF00
#define POS_PMTSTA			16
#define INTSYSCR		0x804
#define CORECFGR		0xBC0

#ifndef __ASSEMBLER__
extern void soc_interrupt_init(void);

#endif

#endif /* __SOC_H__ */
