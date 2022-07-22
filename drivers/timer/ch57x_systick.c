
#define DT_DRV_COMPAT wch_ch57x_systick

#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <soc.h>

#define CYC_PER_TICK ((uint32_t)((uint64_t)sys_clock_hw_cycles_per_sec()	\
			      / (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define MAX_CYCLES 0xffffffffu
#define MAX_TICKS ((MAX_CYCLES - CYC_PER_TICK) / CYC_PER_TICK)

#define TIMER_STOPPED 0xffffffffu

#define MIN_DELAY MAX(1024, (CYC_PER_TICK/16))

static struct k_spinlock lock;

static uint32_t last_load;

static uint32_t cycle_count;

static uint32_t announced_cycles;

static volatile uint32_t overflow_cyc;

union stk_ctlr {
	struct {
		uint32_t ste : 1;
		uint32_t stie : 1;
		uint32_t stclk : 1;
		uint32_t _rsvd1 : 5;
		uint32_t streload : 1;
		uint32_t _rsvd2 : 22;
	};
	uint32_t val;
} __attribute__ ((packed));

enum stk_clk {
	HCLK_DIV_8 = 0,
	HCLK,
};

union stk_cnt {
	struct {
		uint32_t cntl;
		uint32_t cnth;
	};
	uint64_t val;
} __attribute__ ((packed));

union stk_reload {
	struct {
		uint32_t reload_low;
		uint32_t reload_high;
	};
	uint64_t val;
} __attribute__ ((packed));

union stk_flag {
	struct {
		uint32_t swie : 1;
		uint32_t cntif : 1;
		uint32_t _rsvd : 30;
	};
	uint32_t val;
} __attribute__ ((packed));

struct systick {
	union stk_ctlr ctrl;
	union stk_cnt cnt;
	union stk_reload reload;
	union stk_flag flag;
} __attribute__ ((packed));

static struct systick *systick;

struct systick_config {
	void *reg_base;
	int divided_by_8;	/* use hclk/8 as clock base ? */
};

static uint32_t elapsed(void)
{
	uint32_t val1 = systick->cnt.cntl;	/* A */
	union stk_flag flag = systick->flag;	/* B */
	uint32_t val2 = systick->cnt.cntl;	/* C */

	/* SysTick behavior: The counter wraps at zero automatically,
	 * setting the COUNTFLAG field of the CTRL register when it
	 * does.  Reading the control register automatically clears
	 * that field.
	 *
	 * If the count wrapped...
	 * 1) Before A then COUNTFLAG will be set and val1 >= val2
	 * 2) Between A and B then COUNTFLAG will be set and val1 < val2
	 * 3) Between B and C then COUNTFLAG will be clear and val1 < val2
	 * 4) After C we'll see it next time
	 *
	 * So the count in val2 is post-wrap and last_load needs to be
	 * added if and only if COUNTFLAG is set or val1 < val2.
	 */
	if ((flag.cntif) || (val1 < val2)) {
		overflow_cyc += last_load;

		/* We know there was a wrap, but we might not have
		 * seen it in flag, so clear it. */
		systick->flag.cntif = 0;
	}

	return (last_load - val2) + overflow_cyc;
}

void systick_isr(struct device *dev)
{
	ARG_UNUSED(dev);
	uint32_t dticks;

	elapsed();
	cycle_count += overflow_cyc;
	overflow_cyc = 0;

	if (IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		dticks = (cycle_count - announced_cycles) / CYC_PER_TICK;
		announced_cycles += dticks * CYC_PER_TICK;
		sys_clock_announce(dticks);
	} else {
		sys_clock_announce(1);
	}
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	if (IS_ENABLED(CONFIG_TICKLESS_KERNEL) && idle && ticks == K_TICKS_FOREVER) {
		systick->ctrl.ste = 0;
		last_load = TIMER_STOPPED;
		return;
	}

#if defined(CONFIG_TICKLESS_KERNEL)
	uint32_t delay;
	uint32_t val1, val2;
	uint32_t last_load_ = last_load;

	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t pending = elapsed();

	val1 = systick->cnt.cntl;

	cycle_count += pending;
	overflow_cyc = 0U;

	uint32_t unannounced = cycle_count - announced_cycles;

	if ((int32_t)unannounced < 0) {
		/* We haven't announced for more than half the 32-bit
		 * wrap duration, because new timeouts keep being set
		 * before the existing one fires.  Force an announce
		 * to avoid loss of a wrap event, making sure the
		 * delay is at least the minimum delay possible.
		 */
		last_load = MIN_DELAY;
	} else {
		/* Desired delay in the future */
		delay = ticks * CYC_PER_TICK;

		/* Round delay up to next tick boundary */
		delay += unannounced;
		delay = ((delay + CYC_PER_TICK - 1) / CYC_PER_TICK) * CYC_PER_TICK;
		delay -= unannounced;
		delay = MAX(delay, MIN_DELAY);
		if (delay > MAX_CYCLES) {
			last_load = MAX_CYCLES;
		} else {
			last_load = delay;
		}
	}

	val2 = systick->cnt.cntl;

	systick->reload.reload_low = last_load - 1;
	systick->ctrl.streload = 1; /* resets timer to last_load */

	/*
	 * Add elapsed cycles while computing the new load to cycle_count.
	 *
	 * Note that comparing val1 and val2 is normaly not good enough to
	 * guess if the counter wrapped during this interval. Indeed if val1 is
	 * close to LOAD, then there are little chances to catch val2 between
	 * val1 and LOAD after a wrap. COUNTFLAG should be checked in addition.
	 * But since the load computation is faster than MIN_DELAY, then we
	 * don't need to worry about this case.
	 */
	if (val1 < val2) {
		cycle_count += (val1 + (last_load_ - val2));
	} else {
		cycle_count += (val1 - val2);
	}
	k_spin_unlock(&lock, key);

#endif
}

/* ticks elapsed since last annocement */
uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t cyc = elapsed() + cycle_count - announced_cycles;

	k_spin_unlock(&lock, key);
	return cyc / CYC_PER_TICK;
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = elapsed() + cycle_count;

	k_spin_unlock(&lock, key);
	return ret;
}

#if 0
uint64_t sys_clock_cycle_get_64(void)
{
	return 0;
}
#endif

void sys_clock_idle_exit(void)
{
	if (last_load == TIMER_STOPPED) {
		systick->ctrl.ste = 1;
	}
}

void sys_clock_disable(void)
{
	systick->ctrl.ste = 0;
}

static int systick_init(const struct device *dev)
{
	const struct systick_config *config = dev->config;

	systick = (struct systick *)(config->reg_base);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), \
		systick_isr, DEVICE_DT_INST_GET(0),	0);
	irq_enable(DT_INST_IRQN(0));

	last_load = MAX_CYCLES;
	systick->reload.val = last_load - 1;
	systick->ctrl.stclk = HCLK;
	systick->ctrl.streload = 1;
	systick->ctrl.stie = 1;
	systick->ctrl.ste = 1;

	return 0;
}

static const struct systick_config systick_config = {
	.reg_base = (void *)DT_INST_REG_ADDR(0),
	.divided_by_8 = 0,	/* TODO get from dts */
};

DEVICE_DT_INST_DEFINE(0,	\
		systick_init,		\
		NULL,				\
		NULL,				\
		&systick_config,	\
		PRE_KERNEL_2,		\
		CONFIG_SYSTEM_CLOCK_INIT_PRIORITY,	\
		NULL);
