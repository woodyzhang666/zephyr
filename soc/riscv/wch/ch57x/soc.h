
#ifndef __SOC_H__
#define __SOC_H__

#define SOC_MCAUSE_USER_ECALL_EXP 8 /* User ECALL instruction */
#define SOC_MCAUSE_ECALL_EXP 11 /* Machine ECALL instruction */

#define SOC_MCAUSE_EXP_MASK 0x7FFFFFFF

#define SOC_ERET mret

#endif /* __SOC_H__ */
