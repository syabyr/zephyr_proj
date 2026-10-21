/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Power management test for the EFR32MG1P. The kernel tick runs on the
 * RTCC through the Silabs sleeptimer, so the uptime this prints must track
 * the wall clock of whatever reads the console, including across the EM2
 * the idle thread enters with CONFIG_PM.
 *
 * The Gecko power manager reports every energy mode transition it makes, so
 * the counters show which modes are really entered rather than which ones
 * the policy selected.
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/printk.h>

#include <sl_power_manager.h>

#define REPORT_PERIOD_MS	2000

static volatile uint32_t em1_entries;
static volatile uint32_t em2_entries;

static void em_transition_handler(sl_power_manager_em_t from, sl_power_manager_em_t to)
{
	ARG_UNUSED(from);

	if (to == SL_POWER_MANAGER_EM1) {
		em1_entries++;
	} else if (to == SL_POWER_MANAGER_EM2) {
		em2_entries++;
	}
}

static const sl_power_manager_em_transition_event_info_t em_transition_info = {
	.event_mask = SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM1 |
		      SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM2,
	.on_event = em_transition_handler,
};

static sl_power_manager_em_transition_event_handle_t em_transition_handle;

int main(void)
{
	uint32_t em1_last = 0;
	uint32_t em2_last = 0;

	printk("EFR32MG1P power management test\n");

	/* Nothing reports transitions when CONFIG_PM is off, and the Gecko
	 * power manager is not linked in that build.
	 */
	if (IS_ENABLED(CONFIG_PM)) {
		sl_power_manager_subscribe_em_transition_event(&em_transition_handle,
							       &em_transition_info);
	}

	while (true) {
		int64_t before = k_uptime_get();
		int64_t slept;

		k_msleep(REPORT_PERIOD_MS);
		slept = k_uptime_get() - before;

		/* The overshoot is measured around the sleep only: printing the
		 * line itself blocks for about 7 ms at 115200 baud.
		 */
		printk("uptime %lld ms, slept %lld ms for %d ms, overshoot %lld ms, "
		       "EM1 %u (d%u), EM2 %u (d%u)\n",
		       k_uptime_get(), slept, REPORT_PERIOD_MS, slept - REPORT_PERIOD_MS,
		       em1_entries, em1_entries - em1_last,
		       em2_entries, em2_entries - em2_last);

		em1_last = em1_entries;
		em2_last = em2_entries;
	}

	return 0;
}