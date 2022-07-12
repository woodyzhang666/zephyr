
#define DT_DRV_COMPAT wch_wch_uart

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>

#include "uart_wch.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_wch, CONFIG_UART_LOG_LEVEL);


static inline int uart_wch_set_baudrate(const struct device *dev, uint32_t baudrate)
{
	const struct uart_wch_config *config = dev->config;
	struct wch_uart *uart = (struct wch_uart *)(config->base);
	struct uart_wch_data *data = dev->data;

	uint32_t fsys = 20 * 1024 *1024;	/* TODO get from clock source */
	uint8_t div = 1;
	uart->div.divisor1 = div;
	uart->dl.divisor2 = (uint16_t)((fsys * 2 / div / 16 + data->baudrate / 2) / data->baudrate);
	return 0;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) ||  defined(CONFIG_UART_ASYNC_API)
static void uart_wch_isr(const struct device *dev)
{

}
#endif

static int uart_wch_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_wch_config *config = dev->config;
	struct wch_uart *uart = (struct wch_uart *)(config->base);

	/* err flags are RZ */
	struct uart_lsr lsr = uart->lsr;
	if (lsr.data_rdy) {
		*c = uart->rbr.val;
		return 0;
	} else {
		return -1;
	}
}

static void uart_wch_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_wch_config *config = dev->config;
	struct wch_uart *uart = (struct wch_uart *)(config->base);
	int key;

	while (1) {
		if (uart->lsr.tx_fifo_empty) {
			key = irq_lock();
			if (uart->lsr.tx_fifo_empty)
				break;
			irq_unlock(key);
		}
	}

	uart->thr.val = (uint8_t)c;
	irq_unlock(key);
}

static int uart_wch_err_check(const struct device *dev)
{
	const struct uart_wch_config *config = dev->config;
	struct wch_uart *uart = (struct wch_uart *)(config->base);
	int err = 0;

	struct uart_lsr lsr = uart->lsr;
	if (lsr.rx_overflow)
		err |= UART_ERROR_OVERRUN;

	if (lsr.parity_err)
		err |= UART_ERROR_PARITY;

	if (lsr.frame_err)
		err |= UART_ERROR_FRAMING;

	if (lsr.break_err)
		err |= UART_BREAK;

	return err;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_wch_configure(const struct device *dev,
				const struct uart_config *cfg)
{
	const struct uart_wch_config *config = dev->config;
	struct wch_uart *uart = (struct wch_uart *)(config->base);
	struct uart_wch_data *data = dev->data;

	struct uart_lcr lcr = uart->lcr;

	switch (cfg->parity) {
		case UART_CFG_PARITY_NONE:
			lcr.parity_en = False;
			break;
		case UART_CFG_PARITY_ODD:
			lcr.parity_en = True;
			lcr.parity_mode = UART_LCR_PARITY_MODE_ODD;
			break;
		case UART_CFG_PARITY_EVEN:
			lcr.parity_en = True;
			lcr.parity_mode = UART_LCR_PARITY_MODE_EVEN;
			break;
		case UART_CFG_PARITY_MARK:
			lcr.parity_en = True;
			lcr.parity_mode = UART_LCR_PARITY_MODE_MARK;
			break;
		case UART_CFG_PARITY_SPACE:
			lcr.parity_en = True;
			lcr.parity_mode = UART_LCR_PARITY_MODE_SPACE;
			break;
		default:
			return -EINVAL;
	}

	switch (cfg->data_bits) {
		case UART_CFG_DATA_BITS_5:
			lcr.word_sz = UART_LCR_WORD_SZ_5_BIT;
			break;
		case UART_CFG_DATA_BITS_6:
			lcr.word_sz = UART_LCR_WORD_SZ_6_BIT;
			break;
		case UART_CFG_DATA_BITS_7:
			lcr.word_sz = UART_LCR_WORD_SZ_7_BIT;
			break;
		case UART_CFG_DATA_BITS_8:
			lcr.word_sz = UART_LCR_WORD_SZ_8_BIT;
			break;
		case UART_CFG_DATA_BITS_9:
			return -ENOTSUP;
		default:
			return -EINVAL;
	}

	switch (cfg->stop_bits) {
		case UART_CFG_STOP_BITS_0_5:
		case UART_CFG_STOP_BITS_1_5:
			return -ENOTSUP;
		case UART_CFG_STOP_BITS_1:
			lcr.stop_bits = UART_LCR_STOP_BIT_1_BIT;
			break;
		case UART_CFG_STOP_BITS_2:
			lcr.stop_bits = UART_LCR_STOP_BIT_2_BIT;
			break;
		default:
			return -EINVAL;
	}

	/* TODO Add flow control for uart0 */
	if (cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE)
		return -ENOTSUP;

	/* how to disable uart without reset */

	uart->lcr = lcr;

	if (cfg->baudrate != data->baudrate) {
		uart_wch_set_baudrate(dev, cfg->baudrate);
		data->baudrate = cfg->baudrate;
	}

	struct uart_fcr fcr = uart->fcr;
	fcr.rx_fifo_clr = True;
	fcr.tx_fifo_clr = True;
	uart->fcr = fcr;

	/* read to clear */
	struct uart_lsr lsr __unused = uart->lsr;

	return 0;
}

static int uart_wch_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	const struct uart_wch_config *config = dev->config;
	struct wch_uart *uart = (struct wch_uart *)(config->base);
	struct uart_wch_data *data = dev->data;

	cfg->baudrate = data->baudrate;

	struct uart_lcr lcr = uart->lcr;
	switch (lcr.word_sz) {
		case UART_LCR_WORD_SZ_5_BIT:
			cfg->data_bits = UART_CFG_DATA_BITS_5;
			break;
		case UART_LCR_WORD_SZ_6_BIT:
			cfg->data_bits = UART_CFG_DATA_BITS_6;
			break;
		case UART_LCR_WORD_SZ_7_BIT:
			cfg->data_bits = UART_CFG_DATA_BITS_7;
			break;
		case UART_LCR_WORD_SZ_8_BIT:
			cfg->data_bits = UART_CFG_DATA_BITS_8;
			break;
	}

	switch (lcr.stop_bits) {
		case UART_LCR_STOP_BIT_1_BIT:
			cfg->stop_bits = UART_CFG_STOP_BITS_1;
			break;
		case UART_LCR_STOP_BIT_2_BIT:
			cfg->stop_bits = UART_CFG_STOP_BITS_2;
			break;
	}

	if (!lcr.parity_en)
		cfg->parity = UART_CFG_PARITY_NONE;
	else {
		switch (lcr.parity_mode) {
			case UART_LCR_PARITY_MODE_ODD:
				cfg->parity = UART_CFG_PARITY_ODD;
				break;
			case UART_LCR_PARITY_MODE_EVEN:
				cfg->parity = UART_CFG_PARITY_EVEN;
				break;
			case UART_LCR_PARITY_MODE_MARK:
				cfg->parity = UART_CFG_PARITY_MARK;
				break;
			case UART_LCR_PARITY_MODE_SPACE:
				cfg->parity = UART_CFG_PARITY_SPACE;
				break;
		}
	}

	cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	return 0;
}

#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static const struct uart_driver_api uart_wch_driver_api = {
	.poll_in = uart_wch_poll_in,
	.poll_out = uart_wch_poll_out,
	.err_check = uart_wch_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_wch_configure,
	.config_get = uart_wch_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_stm32_fifo_fill,
	.fifo_read = uart_stm32_fifo_read,
	.irq_tx_enable = uart_stm32_irq_tx_enable,
	.irq_tx_disable = uart_stm32_irq_tx_disable,
	.irq_tx_ready = uart_stm32_irq_tx_ready,
	.irq_tx_complete = uart_stm32_irq_tx_complete,
	.irq_rx_enable = uart_stm32_irq_rx_enable,
	.irq_rx_disable = uart_stm32_irq_rx_disable,
	.irq_rx_ready = uart_stm32_irq_rx_ready,
	.irq_err_enable = uart_stm32_irq_err_enable,
	.irq_err_disable = uart_stm32_irq_err_disable,
	.irq_is_pending = uart_stm32_irq_is_pending,
	.irq_update = uart_stm32_irq_update,
	.irq_callback_set = uart_stm32_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_stm32_async_callback_set,
	.tx = uart_stm32_async_tx,
	.tx_abort = uart_stm32_async_tx_abort,
	.rx_enable = uart_stm32_async_rx_enable,
	.rx_disable = uart_stm32_async_rx_disable,
	.rx_buf_rsp = uart_stm32_async_rx_buf_rsp,
#endif  /* CONFIG_UART_ASYNC_API */
};

static int uart_wch_init(const struct device *dev)
{
	const struct uart_wch_config *config = dev->config;
	const struct uart_wch_data *data = dev->data;
	int err = 0;
	struct wch_uart *uart = (struct wch_uart *)(config->base);

	/* enabled power and clock */

	/* pinctrl */

	uart->ier.uart_reset = True;

	struct uart_lcr lcr = {0};
	if (config->parity == 2) {
		/* 8 data bits, 1 parity bit, parity even */
		lcr.parity_en = True;
		lcr.parity_mode = UART_LCR_PARITY_MODE_EVEN;
	} else if (config->parity == 1) {
		/* 8 data bits, 1 parity bit, parity odd */
		lcr.parity_en = True;
		lcr.parity_mode = UART_LCR_PARITY_MODE_ODD;
	} else {
		lcr.parity_en = False;
	}
	lcr.word_sz = UART_LCR_WORD_SZ_8_BIT;
	lcr.stop_bits = UART_LCR_STOP_BIT_1_BIT;

	uart->lcr = lcr;

	struct uart_mcr mcr = {0};
	if (config->hw_flow_control) {
		mcr.hw_flow_ctrl = True;
	}
	uart->mcr = mcr;

	struct uart_fcr fcr = {
		.fifo_en = True,
		.fifo_trig = UART_FIFO_TRIG_4_BYTES,
		.tx_fifo_clr = True,
		.rx_fifo_clr = True,
	};
	uart->fcr = fcr;

	err = uart_wch_set_baudrate(dev, data->baudrate);
	if (err)
		return err;

	uart->ier.txd_output_en = True;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	config->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
	return uart_wch_async_init(dev);
#else
	return err;
#endif
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define WCH_UART_IRQ_HANDLER_DECL(index)				\
	static void uart_wch_irq_config_func_##index(const struct device *dev);
#define WCH_UART_IRQ_HANDLER(index)					\
static void uart_wch_irq_config_func_##index(const struct device *dev)	\
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),				\
		DT_INST_IRQ(index, priority),				\
		uart_wch_isr, DEVICE_DT_INST_GET(index),		\
		0);							\
	irq_enable(DT_INST_IRQN(index));				\
}

#define WCH_UART_IRQ_HANDLER_FUNC(index)				\
	.irq_config_func = uart_wch_irq_config_func_##index,
#else
#define WCH_UART_IRQ_HANDLER_DECL(index) /* Not used */
#define WCH_UART_IRQ_HANDLER(index) /* Not used */
#define WCH_UART_IRQ_HANDLER_FUNC(index) /* Not used */
#endif


#define WCH_UART_INIT(index)	\
WCH_UART_IRQ_HANDLER_DECL(index)					\
									\
static const struct uart_wch_config uart_wch_cfg_##index = {	\
	.base = (void *)DT_INST_REG_ADDR(index),		\
	.hw_flow_control = DT_INST_PROP(index, hw_flow_control),	\
	.parity = DT_INST_ENUM_IDX_OR(index, parity, UART_CFG_PARITY_NONE),	\
	.single_wire = DT_INST_PROP_OR(index, single_wire, false),	\
	WCH_UART_IRQ_HANDLER_FUNC(index)				\
};													\
													\
static struct uart_wch_data uart_wch_data_##index = {		\
	.baudrate = DT_INST_PROP(index, current_speed),		\
};									\
									\
DEVICE_DT_INST_DEFINE(index,						\
		    &uart_wch_init,					\
		    NULL,						\
		    &uart_wch_data_##index, &uart_wch_cfg_##index,	\
		    PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,		\
		    &uart_wch_driver_api);				\
									\
WCH_UART_IRQ_HANDLER(index)
	
DT_INST_FOREACH_STATUS_OKAY(WCH_UART_INIT)
