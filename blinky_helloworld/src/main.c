/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTOS demo: two worker threads synchronized through a message queue,
 * with main() acting only as the boot thread and then returning.
 *
 *   blink_thread (prio 5, producer)  toggles LED every second and puts
 *                                     {tick, state} on a k_msgq
 *   hello_thread (prio 6, consumer)  blocks on the k_msgq and prints
 *                                     "Hello World" for each LED event
 *
 * main() configures hardware, dumps the thread list, starts the workers
 * (defined with a delayed start) and RETURNS - the scheduler keeps
 * running the two threads off the idle thread forever after.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define BLINK_INTERVAL_MS 1000
#define BLINK_THREAD_PRIO 5  /* lower number == higher priority */
#define HELLO_THREAD_PRIO 6
#define BLINK_STACK_SIZE 512
#define HELLO_STACK_SIZE 1024

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* Message passed from the blink thread to the hello thread. */
struct led_event {
	uint32_t tick;
	bool on;
};

/* Producer/consumer queue: blink_thread -> hello_thread. */
K_MSGQ_DEFINE(led_events, sizeof(struct led_event), 4, 4);

static void blink_thread(void)
{
	struct led_event ev = {
		.tick = 0,
		.on = false,
	};

	for (;;) {
		gpio_pin_toggle_dt(&led);
		ev.on = !ev.on;
		ev.tick++;

		/* K_FOREVER: block if the queue is full - no busy-waiting. */
		k_msgq_put(&led_events, &ev, K_FOREVER);

		/* Thread blocks here; the scheduler runs hello_thread meanwhile. */
		k_msleep(BLINK_INTERVAL_MS);
	}
}

static void hello_thread(void)
{
	struct led_event ev;

	for (;;) {
		/* Sleep until the blink thread reports a toggle. */
		k_msgq_get(&led_events, &ev, K_FOREVER);

		printk("Hello World! tick=%u LED=%s uptime=%u ms (thread \"%s\", prio %d)\n",
		       ev.tick, ev.on ? "ON" : "OFF", k_uptime_get_32(),
		       k_thread_name_get(k_current_get()),
		       k_thread_priority_get(k_current_get()));
	}
}

/* Delayed start: created at boot, held until main() calls k_thread_start().
 * (Delay is given in ms; SYS_FOREVER_MS = never auto-start.) */
K_THREAD_DEFINE(blink_tid, BLINK_STACK_SIZE, blink_thread, NULL, NULL, NULL,
		BLINK_THREAD_PRIO, 0, SYS_FOREVER_MS);
K_THREAD_DEFINE(hello_tid, HELLO_STACK_SIZE, hello_thread, NULL, NULL, NULL,
		HELLO_THREAD_PRIO, 0, SYS_FOREVER_MS);

static void dump_thread(const struct k_thread *t, void *user_data)
{
	const char *name = k_thread_name_get((k_tid_t)t);

	ARG_UNUSED(user_data);

#if defined(CONFIG_THREAD_STACK_INFO)
	printk("  %-10s prio %3d   stack %u B\n", name ? name : "(unnamed)",
	       k_thread_priority_get((k_tid_t)t), t->stack_info.size);
#else
	printk("  %-10s prio %3d\n", name ? name : "(unnamed)",
	       k_thread_priority_get((k_tid_t)t));
#endif
}

int main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		printk("Error: LED device not ready\n");
		return 0;
	}
	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
		printk("Error: failed to configure LED\n");
		return 0;
	}

	k_thread_name_set(blink_tid, "blink");
	k_thread_name_set(hello_tid, "hello");

	printk("\nThreads scheduled at boot:\n");
	k_thread_foreach(dump_thread, NULL);

	k_thread_start(blink_tid);
	k_thread_start(hello_tid);

	printk("main() is just the boot thread - returning now, "
	       "the scheduler keeps %d threads alive.\n\n", 2);
	return 0;
}
