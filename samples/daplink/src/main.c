
#include <string.h>
#include <zephyr/zephyr.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#if 1

#define CONN_LED_NODE	DT_NODELABEL(conn_led)
#define RUN_LED_NODE	DT_NODELABEL(run_led)

#define SWCLK_TCK_NODE	DT_NODELABEL(swclk_tck)
#define SWDIO_TMS_IN_NODE	DT_NODELABEL(swdio_tms_in)
#define SWDIO_TMS_OUT_NODE	DT_NODELABEL(swdio_tms_out)
#define TDI_NODE	DT_NODELABEL(tdi)
#define TDO_NODE	DT_NODELABEL(tdo)
#define NRESET_NODE		DT_NODELABEL(nreset)

#define TARGET_PWR_NODE		DT_NODELABEL(pwr_sw)
#define PWR_BUTTON_NODE		DT_NODELABEL(pwr_button)

#if !DT_NODE_HAS_STATUS(CONN_LED_NODE, okay) || \
	!DT_NODE_HAS_STATUS(RUN_LED_NODE, okay) || \
	!DT_NODE_HAS_STATUS(SWCLK_TCK_NODE, okay) || \
	!DT_NODE_HAS_STATUS(SWDIO_TMS_IN_NODE, okay) || \
	!DT_NODE_HAS_STATUS(SWDIO_TMS_OUT_NODE, okay) || \
	!DT_NODE_HAS_STATUS(TDI_NODE, okay) || \
	!DT_NODE_HAS_STATUS(TDO_NODE, okay) || \
	!DT_NODE_HAS_STATUS(NRESET_NODE, okay) || \
	!DT_NODE_HAS_STATUS(TARGET_PWR_NODE, okay) || \
	!DT_NODE_HAS_STATUS(PWR_BUTTON_NODE, okay)
#error "Unsupported board: gpio labels are not correctly defined"
#endif

#define LED_OFF		0
#define LED_ON		1
#define GPIO_ASSERT		1
#define GPIO_DEASSERT	0
#define GPIO_HIHG	1
#define GPIO_LOW	0

static const struct gpio_dt_spec conn_led_spec = GPIO_DT_SPEC_GET(CONN_LED_NODE, gpios);
static const struct gpio_dt_spec run_led_spec = GPIO_DT_SPEC_GET(RUN_LED_NODE, gpios);
static const struct gpio_dt_spec nreset_spec = GPIO_DT_SPEC_GET(NRESET_NODE, gpios);
static const struct gpio_dt_spec swclk_tck_spec = GPIO_DT_SPEC_GET(SWCLK_TCK_NODE, gpios);
static const struct gpio_dt_spec swdio_tms_in_spec = GPIO_DT_SPEC_GET(SWDIO_TMS_IN_NODE, gpios);
static const struct gpio_dt_spec swdio_tms_out_spec = GPIO_DT_SPEC_GET(SWDIO_TMS_OUT_NODE, gpios);
static const struct gpio_dt_spec tdi_spec = GPIO_DT_SPEC_GET(TDI_NODE, gpios);
static const struct gpio_dt_spec tdo_spec = GPIO_DT_SPEC_GET(TDO_NODE, gpios);
static const struct gpio_dt_spec target_pwr_spec = GPIO_DT_SPEC_GET(TARGET_PWR_NODE, gpios);
static const struct gpio_dt_spec pwr_button_spec = GPIO_DT_SPEC_GET(PWR_BUTTON_NODE, gpios);

static struct gpio_callback pwr_button_cb_data;

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("gpio interrupt\n");
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	k_busy_wait(1000 * 1000 * 10);
	printk("gpio int end\n");
}


int gpio_init(void)
{
	int ret = 0;

	if (!device_is_ready(conn_led_spec.port) || 
			!device_is_ready(run_led_spec.port) ||
			!device_is_ready(nreset_spec.port) ||
			!device_is_ready(swclk_tck_spec.port) ||
			!device_is_ready(swdio_tms_in_spec.port) ||
			!device_is_ready(swdio_tms_out_spec.port) ||
			!device_is_ready(tdi_spec.port) ||
			!device_is_ready(tdo_spec.port) ||
			!device_is_ready(pwr_button_spec.port) ||
			!device_is_ready(target_pwr_spec.port)) {
		printk("Not all gpio ports are ready!\n");
		return -1;
	}

	ret = gpio_pin_configure_dt(&conn_led_spec, GPIO_OUTPUT);
	ret += gpio_pin_configure_dt(&run_led_spec, GPIO_OUTPUT);
	ret += gpio_pin_configure_dt(&nreset_spec, GPIO_OUTPUT);
	ret += gpio_pin_configure_dt(&swclk_tck_spec, GPIO_OUTPUT);
	ret += gpio_pin_configure_dt(&swdio_tms_in_spec, GPIO_INPUT);
	ret += gpio_pin_configure_dt(&swdio_tms_out_spec, GPIO_OUTPUT);
	ret += gpio_pin_configure_dt(&tdi_spec, GPIO_INPUT);
	ret += gpio_pin_configure_dt(&tdo_spec, GPIO_OUTPUT);
	ret += gpio_pin_configure_dt(&target_pwr_spec, GPIO_OUTPUT);
	ret += gpio_pin_configure_dt(&pwr_button_spec, GPIO_INPUT);
	if (ret) {
		printk("fail to configu gpio pins!\n");
		return ret;
	}

	gpio_pin_set_dt(&target_pwr_spec, GPIO_DEASSERT);	/* keep target power off */
	gpio_pin_set_dt(&conn_led_spec, LED_OFF);	/* logic off */
	gpio_pin_set_dt(&run_led_spec, LED_OFF);
	gpio_pin_set_dt(&nreset_spec, GPIO_DEASSERT);
	gpio_pin_set_dt(&swclk_tck_spec, GPIO_HIHG);
	gpio_pin_set_dt(&swdio_tms_out_spec, GPIO_HIHG);
	gpio_pin_set_dt(&tdo_spec, GPIO_HIHG);

	ret = gpio_pin_interrupt_configure_dt(&pwr_button_spec, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, pwr_button_spec.port->name, pwr_button_spec.pin);
		return ret;
	}

	gpio_init_callback(&pwr_button_cb_data, button_pressed, BIT(pwr_button_spec.pin));
	gpio_add_callback(pwr_button_spec.port, &pwr_button_cb_data);
	printk("Set up button at %s pin %d\n", pwr_button_spec.port->name, pwr_button_spec.pin);

	gpio_pin_set_dt(&conn_led_spec, LED_ON);
	gpio_pin_set_dt(&run_led_spec, LED_OFF);

	return ret;
}

#endif

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

void main(void)
{
	const struct device *dev;
	uint32_t baudrate, dtr = 0U;
	int ret;

	gpio_init();

	dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(dev)) {
		LOG_ERR("CDC ACM device not ready");
		return;
	}

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	LOG_INF("Wait for DTR");

	while (true) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}

	LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate detected: %d", baudrate);
	}

	uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);
}
