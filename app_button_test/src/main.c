#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>

#define SW0_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;
static volatile int interrupt_count = 0;

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	interrupt_count++;
	printk("IRQ! count=%d pins=0x%x\n", interrupt_count, pins);
}

int main(void)
{
	int ret;

	printk("=== IRQ 30 Test ===\n");
	printk("Testing IRQ 30 (46 - 16 = 30 for GPIOF)\n");

	if (!gpio_is_ready_dt(&button)) {
		printk("ERROR: Button device not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	printk("Button configure: %d\n", ret);

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
	printk("Interrupt configure: %d\n", ret);

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	ret = gpio_add_callback_dt(&button, &button_cb_data);
	printk("Add callback: %d\n", ret);

	printk("Calling irq_enable(30)...\n");
	irq_enable(30);

	printk("Now press the button...\n");

	while (1) {
		k_sleep(K_MSEC(1000));
	}

	return 0;
}
