
#define DT_DRV_COMPAT wch_ch58x_systick

#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <soc.h>

#define CYC_PER_TICK ((uint32_t)((uint64_t)sys_clock_hw_cycles_per_sec()	\
			      / (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define MAX_CYCLES 0xFFFFFFFFFFFFULL
#define MAX_TICKS (uint32_t)((MAX_CYCLES - CYC_PER_TICK) / CYC_PER_TICK)

#define TIMER_STOPPED 0xFFFFFFFFFFFFFFFFULL

#define MIN_DELAY MAX(1024, (CYC_PER_TICK/16))

#define INITIAL_CYCLES	0xFFFFFFFFULL

static struct k_spinlock lock;

static uint64_t last_load;

static uint64_t cycle_count;

static uint64_t announced_cycles;

static volatile uint64_t overflow_cyc;

union stk_ctrl {
	struct {
		volatile uint32_t ste : 1;		/* systick enable */
		volatile uint32_t stie : 1;		/* systick interrupt enable */
		volatile uint32_t stclk : 1;		/* use hclk or hclk/8 as clock base */
		volatile uint32_t stre : 1;		/* reload enable */
		volatile uint32_t mode : 1;		/* counting downwards or upwards */
		volatile uint32_t init : 1;		/* force load initial value */
		volatile uint32_t _rsvd : 25;
		volatile uint32_t swie : 1;		/* trigger software interrupt */
	};
	volatile uint32_t val;
};

union stk_status {
	struct {
		volatile uint32_t cntif : 1;
		volatile uint32_t _rsvd : 31;
	};
	volatile uint32_t val;
};

union stk_cnt {
	struct {
		volatile uint32_t low;
		volatile uint32_t high;
	};
	volatile uint64_t val;
};

union stk_cmp {
	struct {
		volatile uint32_t low;
		volatile uint32_t high;
	};
	volatile uint64_t val;
};

struct systick {
	union stk_ctrl ctrl;
	union stk_status status;
	union stk_cnt cnt;
	union stk_cmp cmp;
} __attribute__ ((packed));

BUILD_ASSERT((uint32_t)(&((struct systick *)0)->ctrl) == 0x00);
BUILD_ASSERT((uint32_t)(&((struct systick *)0)->status) == 0x04);
BUILD_ASSERT((uint32_t)(&((struct systick *)0)->cnt.low) == 0x08);
BUILD_ASSERT((uint32_t)(&((struct systick *)0)->cnt.high) == 0x0C);
BUILD_ASSERT((uint32_t)(&((struct systick *)0)->cmp.low) == 0x10);
BUILD_ASSERT((uint32_t)(&((struct systick *)0)->cmp.high) == 0x14);
BUILD_ASSERT(sizeof(struct systick) == 0x18);

static struct systick *systick = (void *)DT_INST_REG_ADDR(0);

enum stk_clk {
	HCLK_DIV_8 = 0,
	HCLK,
};

enum stk_mode {
	COUNTING_UP	= 0,
	COUNTING_DOWN = 1,
};

struct systick_config {
	int divided_by_8;	/* use hclk/8 as clock base ? */
	int mode;
};

static uint64_t elapsed(void)
{
	uint64_t val1 = systick->cnt.val;	/* A */
	union stk_status status = systick->status;	/* B */
	uint64_t val2 = systick->cnt.val;	/* C */

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
	if ((status.cntif) || (val1 > val2)) {
		overflow_cyc += last_load;

		/* We know there was a wrap, but we might not have
		 * seen it in flag, so clear it. */
		systick->status.cntif = 0;
	}

	return val2 + overflow_cyc;
}

static void systick_isr(struct device *dev)
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
	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;

	k_spinlock_key_t key = k_spin_lock(&lock);

	last_load = (uint64_t)ticks * CYC_PER_TICK - 1;
	systick->cmp.val = last_load;

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
	uint64_t cyc = elapsed() + cycle_count - announced_cycles;

	k_spin_unlock(&lock, key);
	return cyc / CYC_PER_TICK;
}

#if 1
uint64_t sys_clock_cycle_get_64(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t ret = elapsed() + cycle_count;

	k_spin_unlock(&lock, key);
	return ret;
}
#endif

uint32_t sys_clock_cycle_get_32(void)
{
	return (uint32_t)sys_clock_cycle_get_64();
}

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
	ARG_UNUSED(dev);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), \
		systick_isr, DEVICE_DT_INST_GET(0),	0);
	irq_enable(DT_INST_IRQN(0));

	last_load = MAX_CYCLES;
	systick->cmp.val = last_load - 1;
	systick->ctrl.mode = COUNTING_UP;
	systick->ctrl.stclk = HCLK;
	systick->ctrl.stre = 1;
	systick->ctrl.stie = 1;
	systick->ctrl.ste = 1;

	return 0;
}

static const struct systick_config systick_config = {
	.divided_by_8 = 0,	/* TODO get from dts */
	.mode = 1,	/* TODO get from dts */
};

DEVICE_DT_INST_DEFINE(0,	\
		systick_init,		\
		NULL,				\
		NULL,				\
		&systick_config,	\
		PRE_KERNEL_2,		\
		CONFIG_SYSTEM_CLOCK_INIT_PRIORITY,	\
		NULL);
