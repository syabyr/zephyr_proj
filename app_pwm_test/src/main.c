#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/printk.h>

#define LED0_NODE	DT_ALIAS(pwm_led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#else
#error "No pwm-led0 alias found"
#endif

static const struct device *led_dev = DEVICE_DT_GET(DT_PARENT(LED0_NODE));

int main(void)
{
	int ret;
	int brightness;
	int step = 5;
	int max_brightness = 100;

	printk("=== PWM LED Test (pwm-leds) ===\n");
	printk("LED device: %s\n", led_dev->name);

	if (!device_is_ready(led_dev)) {
		printk("ERROR: LED device not ready\n");
		return 0;
	}

	ret = led_set_brightness(led_dev, 0, 0);
	printk("led_set_brightness(0): %d\n", ret);

	printk("Breathing LED...\n");

	while (1) {
		/* Fade in */
		for (brightness = 0; brightness <= max_brightness; brightness += step) {
			led_set_brightness(led_dev, 0, brightness);
			k_sleep(K_MSEC(30));
		}

		/* Fade out */
		for (brightness = max_brightness; brightness > 0; brightness -= step) {
			led_set_brightness(led_dev, 0, brightness);
			k_sleep(K_MSEC(30));
		}
	}

	return 0;
}
