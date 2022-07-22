
#define DT_DRV_COMPAT wch_wch_gpio

#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/spinlock.h>

#include "gpio_utils.h"
#include "gpio_wch.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_wch, CONFIG_LOG_DEFAULT_LEVEL);

/* avoid disrupting read modify and update sequence */
static struct k_spinlock gpio_spinlock;

typedef void (*gpio_irq_config_func_t)(const struct device *dev);

struct gpio_wch_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config drv_cfg;
	//void *const reg_base;
	const int gpio_port;
	gpio_irq_config_func_t irq_config_func;
};

struct gpio_wch_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t cb;
};


static int gpio_wch_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_wch_config *config = dev->config;
	int port = config->gpio_port;

	*value = *(uint32_t *)R32_PX_PIN(port);

	return 0;
}

static int gpio_wch_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_wch_config *config = dev->config;
	int port = config->gpio_port;
	uint32_t port_value;

	k_spinlock_key_t key = k_spin_lock(&gpio_spinlock);
	port_value = *(uint32_t *)R32_PX_OUT(port);
	*(uint32_t *)R32_PX_OUT(port) = (port_value & ~mask) | (mask & value);
	k_spin_unlock(&gpio_spinlock, key);

	return 0;
}


static int gpio_wch_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_wch_config *config = dev->config;
	int port = config->gpio_port;

	k_spinlock_key_t key = k_spin_lock(&gpio_spinlock);
	*(uint32_t *)R32_PX_OUT(port) |= pins;
	k_spin_unlock(&gpio_spinlock, key);

	return 0;
}

static int gpio_wch_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t pins)
{
	const struct gpio_wch_config *config = dev->config;
	int port = config->gpio_port;

	k_spinlock_key_t key = k_spin_lock(&gpio_spinlock);
	*(uint32_t *)R32_PX_CLR(port) = pins;
	k_spin_unlock(&gpio_spinlock, key);

	return 0;
}

static int gpio_wch_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t pins)
{
	const struct gpio_wch_config *config = dev->config;
	int port = config->gpio_port;

	k_spinlock_key_t key = k_spin_lock(&gpio_spinlock);
	*(uint32_t *)R32_PX_OUT(port) ^= pins;
	k_spin_unlock(&gpio_spinlock, key);

	return 0;
}


static int gpio_wch_configure(const struct device *dev, gpio_pin_t pin,
			     gpio_flags_t flags)
{
	const struct gpio_wch_config *config = dev->config;
	int port = config->gpio_port;

	k_spinlock_key_t key = k_spin_lock(&gpio_spinlock);
	if (flags & GPIO_PULL_UP) {
		*(uint32_t *)R32_PX_PU(port) |= BIT(pin);
	} else {
		*(uint32_t *)R32_PX_PU(port) &= ~BIT(pin);
	}

	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			*(uint32_t *)R32_PX_OUT(port) |= BIT(pin);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			*(uint32_t *)R32_PX_OUT(port) &= ~BIT(pin);
		}
		*(uint32_t *)R32_PX_DIR(port) |= BIT(pin);
	} else {
		*(uint32_t *)R32_PX_DIR(port) &= ~BIT(pin);
	}

	if (flags & GPIO_PULL_DOWN && !(flags & GPIO_OUTPUT)) {
		*(uint32_t *)R32_PX_PD_DRV(port) |= BIT(pin);
	} else {
		*(uint32_t *)R32_PX_PD_DRV(port) &= ~BIT(pin);
	}
	k_spin_unlock(&gpio_spinlock, key);

	return 0;
}

static int gpio_wch_pin_interrupt_configure(const struct device *dev,
			gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_wch_config *config = dev->config;
	int port = config->gpio_port;
	uint32_t mask = WCH_GPIO_INT_BIT(pin);
	int ret = 0;

	k_spinlock_key_t key = k_spin_lock(&gpio_spinlock);

	if (mode == GPIO_INT_MODE_DISABLED) {
		*(uint16_t *)R16_PX_INT_EN(port) &= ~mask;
		/* TODO disable int in pfic */
		goto exit;
	}

	if (mode == GPIO_INT_MODE_LEVEL) {
		*(uint16_t *)R16_PX_INT_MODE(port) &= ~mask;
	} else if (mode == GPIO_INT_MODE_EDGE) {
		*(uint16_t *)R16_PX_INT_MODE(port) |= mask;
	} else {
		// ??
	}

	switch (trig) {
		case GPIO_INT_TRIG_LOW:
			*(uint32_t *)R32_PX_OUT(port) &= ~BIT(pin);
			break;
		case GPIO_INT_TRIG_HIGH:
			*(uint32_t *)R32_PX_OUT(port) |= BIT(pin);
			break;
		default:
			ret = -ENOTSUP;
	}
	*(uint16_t *)R16_PX_INT_EN(port) |= mask;
	/* TODO enable int in pfic */

exit:
	k_spin_unlock(&gpio_spinlock, key);

	return ret;
}

static int gpio_wch_manage_callback(const struct device *dev,
				      struct gpio_callback *callback,
				      bool set)
{
	struct gpio_wch_data *data = dev->data;

	return gpio_manage_callback(&data->cb, callback, set);
}

static uint32_t gpio_wch_get_pending_int(const struct device *dev)
{
	const struct gpio_wch_config *config = dev->config;
	int port = config->gpio_port;

	uint16_t mask = *(uint16_t *)R16_PX_INT_IF(port);
	uint32_t interrupted_pins;
	if (port == 1) {
		interrupted_pins = (uint32_t)(mask & 0xfcff);
		if (mask & BIT(8))
			interrupted_pins |= BIT(22);
		if (mask & BIT(9))
			interrupted_pins |= BIT(23);
	} else {
		interrupted_pins = (uint32_t)mask;
	}
		
	return interrupted_pins;
}


static void gpio_wch_isr(void *param)
{
	const struct device *dev = param;
	const struct gpio_wch_config *const config = dev->config;
	int port = config->gpio_port;
	struct gpio_wch_data *data = dev->data;

	uint16_t mask = *(uint16_t *)R16_PX_INT_IF(port);
	uint32_t interrupted_pins;
	if (port == 1) {
		interrupted_pins = (uint32_t)(mask & 0xfcff);
		if (mask & BIT(8))
			interrupted_pins |= BIT(22);
		if (mask & BIT(9))
			interrupted_pins |= BIT(23);
	} else {
		interrupted_pins = (uint32_t)mask;
	}
		
	if (interrupted_pins != 0) {
		gpio_fire_callbacks(&data->cb, dev, interrupted_pins);
	}
}

static int gpio_wch_init(const struct device *dev)
{
	const struct gpio_wch_config *config = dev->config;

	config->irq_config_func(dev);

	return 0;
}

static const struct gpio_driver_api gpio_wch_driver_api = {
	.pin_configure = gpio_wch_configure,
	.port_get_raw = gpio_wch_port_get_raw,
	.port_set_masked_raw = gpio_wch_port_set_masked_raw,
	.port_set_bits_raw = gpio_wch_port_set_bits_raw,
	.port_clear_bits_raw = gpio_wch_port_clear_bits_raw,
	.port_toggle_bits = gpio_wch_port_toggle_bits,
	.pin_interrupt_configure = gpio_wch_pin_interrupt_configure,
	.manage_callback = gpio_wch_manage_callback,
	.get_pending_int = gpio_wch_get_pending_int
};

#define WCH_GPIO_IRQ_HANDLER_DECL(index)				\
	static void gpio_wch_irq_config_func_##index(const struct device *dev);
#define WCH_GPIO_IRQ_HANDLER(index)					\
static void gpio_wch_irq_config_func_##index(const struct device *dev)	\
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),				\
		DT_INST_IRQ(index, priority),				\
		gpio_wch_isr, DEVICE_DT_INST_GET(index),		\
		0);							\
	irq_enable(DT_INST_IRQN(index));				\
}

#define WCH_GPIO_IRQ_HANDLER_FUNC(index)				\
	.irq_config_func = gpio_wch_irq_config_func_##index,


#define WCH_SOC_GPIO_INIT(_id)	\
								\
WCH_GPIO_IRQ_HANDLER_DECL(_id);	\
								\
WCH_GPIO_IRQ_HANDLER(_id);		\
	static struct gpio_wch_data gpio_data_##_id;	\
	static struct gpio_wch_config gpio_config_##_id = {			\
		.drv_cfg = {							\
			.port_pin_mask = DT_PROP(DT_NODELABEL(gpio##_id), pin_mask),	\
		},								\
		.gpio_port = _id,	\
		WCH_GPIO_IRQ_HANDLER_FUNC(_id)		\
	};									\
	DEVICE_DT_DEFINE(DT_NODELABEL(gpio##_id),				\
			&gpio_wch_init,					\
			NULL,							\
			&gpio_data_##_id,					\
			&gpio_config_##_id,					\
			PRE_KERNEL_1,						\
			CONFIG_GPIO_INIT_PRIORITY,				\
			&gpio_wch_driver_api);

DT_INST_FOREACH_STATUS_OKAY(WCH_SOC_GPIO_INIT);


