#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <soc.h>

void systick_isr(void *arg)
{
	ARG_UNUSED(arg);

}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(ticks);
	ARG_UNUSED(idle);

}

/* ticks elapsed since last annocement */
uint32_t sys_clock_elapsed(void)
{
	return 0;

}

uint32_t sys_clock_cycle_get_32(void)
{
	return 0;

}

uint64_t sys_clock_cycle_get_64(void)
{
	return 0;
}

void sys_clock_idle_exit(void)
{

}

void sys_clock_disable(void)
{

}

static int systick_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

SYS_INIT(systick_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
