#include <zephyr/kernel.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/printk.h>

static const struct device *rtc = DEVICE_DT_GET(DT_NODELABEL(hib));

int main(void)
{
	struct rtc_time tm;
	int ret;
	int count = 0;

	printk("=== RTC Test ===\n");

	if (!device_is_ready(rtc)) {
		printk("ERROR: RTC device not ready\n");
		return 0;
	}

	printk("RTC device: %s\n", rtc->name);

	/* Set initial time: 2026-10-11 12:00:00 (Sunday) */
	tm.tm_sec = 0;
	tm.tm_min = 0;
	tm.tm_hour = 12;
	tm.tm_mday = 11;
	tm.tm_mon = 9;		/* 0-based: 9 = October */
	tm.tm_year = 126;	/* 1900 + 126 = 2026 */
	tm.tm_wday = 0;		/* Sunday = 0 */
	tm.tm_yday = 0;
	tm.tm_isdst = 0;
	tm.tm_nsec = 0;

	ret = rtc_set_time(rtc, &tm);
	printk("rtc_set_time: %d\n", ret);

	if (ret != 0) {
		printk("ERROR: Failed to set time\n");
		return 0;
	}

	printk("Time set. Reading every second...\n");

	while (1) {
		ret = rtc_get_time(rtc, &tm);
		if (ret == 0) {
			printk("%04d-%02d-%02d %02d:%02d:%02d  (read #%d)\n",
			       tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			       tm.tm_hour, tm.tm_min, tm.tm_sec,
			       count++);
		} else {
			printk("rtc_get_time failed: %d\n", ret);
		}

		k_sleep(K_SECONDS(1));
	}

	return 0;
}
