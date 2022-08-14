
#include <zephyr/zephyr.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#define CONN_LED_NODE	DT_NODELABEL(conn_led)
#define RUN_LED_NODE	DT_NODELABEL(run_led)

#define SWCLK_TCK_NODE	DT_NODELABEL(swclk_tck)
#define SWDIO_TMS_IN_NODE	DT_NODELABEL(swdio_tms_in)
#define SWDIO_TMS_OUT_NODE	DT_NODELABEL(swdio_tms_out)
#define TDI_NODE	DT_NODELABEL(tdi)
#define TDO_NODE	DT_NODELABEL(tdo)
#define NRESET_NODE		DT_NODELABEL(nreset)

#define TARGET_PWR_NODE		DT_NODELABEL(pwr_sw)

#if !DT_NODE_HAS_STATUS(CONN_LED_NODE, okay) || \
	!DT_NODE_HAS_STATUS(RUN_LED_NODE, okay) || \
	!DT_NODE_HAS_STATUS(SWCLK_TCK_NODE, okay) || \
	!DT_NODE_HAS_STATUS(SWDIO_TMS_IN_NODE, okay) || \
	!DT_NODE_HAS_STATUS(SWDIO_TMS_OUT_NODE, okay) || \
	!DT_NODE_HAS_STATUS(TDI_NODE, okay) || \
	!DT_NODE_HAS_STATUS(TDO_NODE, okay) || \
	!DT_NODE_HAS_STATUS(NRESET_NODE, okay) || \
	!DT_NODE_HAS_STATUS(TARGET_PWR_NODE, okay)
#error "Unsupported board: gpio labels are not correctly defined"
#endif

#define LED_OFF		0
#define LED_ON		1
#define GPIO_ASSERT		1
#define GPIO_DEASSERT	0
#define GPIO_HIHG	1
#define GPIO_LOW	0

static struct gpio_dt_spec conn_led_spec = GPIO_DT_SPEC_GET(CONN_LED_NODE, gpios);
static struct gpio_dt_spec run_led_spec = GPIO_DT_SPEC_GET(RUN_LED_NODE, gpios);
static struct gpio_dt_spec nreset_spec = GPIO_DT_SPEC_GET(NRESET_NODE, gpios);
static struct gpio_dt_spec swclk_tck_spec = GPIO_DT_SPEC_GET(SWCLK_TCK_NODE, gpios);
static struct gpio_dt_spec swdio_tms_in_spec = GPIO_DT_SPEC_GET(SWDIO_TMS_IN_NODE, gpios);
static struct gpio_dt_spec swdio_tms_out_spec = GPIO_DT_SPEC_GET(SWDIO_TMS_OUT_NODE, gpios);
static struct gpio_dt_spec tdi_spec = GPIO_DT_SPEC_GET(TDI_NODE, gpios);
static struct gpio_dt_spec tdo_spec = GPIO_DT_SPEC_GET(TDO_NODE, gpios);
static struct gpio_dt_spec target_pwr_spec = GPIO_DT_SPEC_GET(TARGET_PWR_NODE, gpios);

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

}

void main(void)
{
	int ret;

	ret = gpio_init();
	if (ret) {
		LOG_ERR("Failed to init GPIO");
		return;
	}

	ret = usb_enable(NULL);
	if (ret) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	return;
}
