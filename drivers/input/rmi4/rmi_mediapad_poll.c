// SPDX-License-Identifier: GPL-2.0
/*
 * Temporary Huawei MediaPad 10 FHD RMI polling fallback.
 *
 * The Synaptics TM2263-002 enumerates correctly over the GPIO-backed I2C bus,
 * but touch reports are not reaching userspace through the PL061 ATTN IRQ.
 * Poll the normal RMI interrupt-status path at 100 Hz so we can separate an
 * IRQ-controller/wiring issue from RMI/F11 data reporting and keep bring-up
 * moving.  This is deliberately scoped to the MediaPad machine only.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/rmi.h>
#include <linux/workqueue.h>

#include "rmi_bus.h"

#define MEDIAPAD_RMI_POLL_MS 10
#define MEDIAPAD_RMI_RETRY_MS 100

static struct delayed_work mediapad_rmi_poll_work;
static struct rmi_device *mediapad_rmi_dev;
static unsigned int mediapad_poll_count;

static int mediapad_match_physical_rmi(struct device *dev, void *data)
{
	return rmi_is_physical_device(dev);
}

static void mediapad_rmi_poll(struct work_struct *work)
{
	struct device *dev;
	int ret;

	if (!mediapad_rmi_dev) {
		dev = bus_find_device(&rmi_bus_type, NULL, NULL,
				      mediapad_match_physical_rmi);
		if (!dev) {
			schedule_delayed_work(&mediapad_rmi_poll_work,
				msecs_to_jiffies(MEDIAPAD_RMI_RETRY_MS));
			return;
		}

		mediapad_rmi_dev = to_rmi_device(dev);
		pr_info("HI3620-RMI-POLL: attached to %s, interval=%ums\n",
			dev_name(dev), MEDIAPAD_RMI_POLL_MS);
	}

	ret = rmi_process_interrupt_requests(mediapad_rmi_dev);
	if (ret)
		pr_warn_ratelimited("HI3620-RMI-POLL: process failed: %d\n", ret);
	else if (mediapad_poll_count++ == 0)
		pr_info("HI3620-RMI-POLL: first status poll completed\n");

	schedule_delayed_work(&mediapad_rmi_poll_work,
		msecs_to_jiffies(MEDIAPAD_RMI_POLL_MS));
}

static int __init mediapad_rmi_poll_init(void)
{
	if (!of_machine_is_compatible("huawei,s10-101x"))
		return 0;

	INIT_DELAYED_WORK(&mediapad_rmi_poll_work, mediapad_rmi_poll);
	schedule_delayed_work(&mediapad_rmi_poll_work,
		msecs_to_jiffies(MEDIAPAD_RMI_RETRY_MS));
	pr_info("HI3620-RMI-POLL: MediaPad fallback enabled\n");
	return 0;
}
late_initcall(mediapad_rmi_poll_init);
