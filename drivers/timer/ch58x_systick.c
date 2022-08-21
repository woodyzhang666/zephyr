
#define DT_DRV_COMPAT wch_ch58x_systick

#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <soc.h>

#define CYC_PER_TICK ((uint32_t)((uint64_t)sys_clock_hw_cycles_per_sec()	\
			      / (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define MAX_SKIP_CYCLES 0xFFFFFFFFULL
#define MAX_SKIP_TICKS (uint32_t)((MAX_SKIP_CYCLES - CYC_PER_TICK) / CYC_PER_TICK)


static struct k_spinlock lock;

static uint64_t announced_cycles;

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

__ramfunc static void systick_isr(struct device *dev)
{
	ARG_UNUSED(dev);
	uint32_t dticks;

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint64_t cyc = systick->cnt.val;
	systick->status.cntif = 0;

	k_spin_unlock(&lock, key);

	if (IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		dticks = (cyc - announced_cycles) / CYC_PER_TICK;
		announced_cycles += dticks * CYC_PER_TICK;
		sys_clock_announce(dticks);
	} else {
		sys_clock_announce(1);
	}
}

/* timeout in ticks after last announce */
__ramfunc void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	if (IS_ENABLED(CONFIG_TICKLESS_KERNEL) && idle && ticks == K_TICKS_FOREVER) {
		systick->ctrl.ste = 0;
		return;
	}

#if defined(CONFIG_TICKLESS_KERNEL)
	ticks = (ticks == K_TICKS_FOREVER) ? MAX_SKIP_TICKS : ticks;
	uint64_t target_cycle = ticks * CYC_PER_TICK + announced_cycles;

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint64_t cyc = systick->cnt.val;
	if (cyc + CYC_PER_TICK / 2 > target_cycle) {
		systick->cmp.val = cyc + CYC_PER_TICK;
	} else {
		systick->cmp.val = target_cycle;
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
	uint64_t cyc = systick->cnt.val - announced_cycles;

	k_spin_unlock(&lock, key);
	return cyc / CYC_PER_TICK;
}

#if 1
uint64_t sys_clock_cycle_get_64(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t ret = systick->cnt.val;

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
	systick->ctrl.ste = 1;
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

	systick->cmp.val = ULLONG_MAX;
	systick->ctrl.mode = COUNTING_UP;
	systick->ctrl.stclk = HCLK;
	systick->ctrl.stre = 0;		/* never reach cmp value */
	systick->ctrl.stie = 1;
	systick->status.cntif = 0;
	systick->ctrl.ste = 1;
	systick->ctrl.init = 1;

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
