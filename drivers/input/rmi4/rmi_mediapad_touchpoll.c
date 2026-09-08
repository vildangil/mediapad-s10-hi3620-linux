// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD RMI4 F11 polling fallback.
 *
 * The TM2263-002 controller probes correctly, but GPIO157/ATTN does not
 * currently deliver usable RMI IRQs on the mainline PL061 path.  Poll F11
 * directly at display-friendly cadence so the tablet remains usable while
 * the GPIO interrupt controller path is investigated.  Unlike the old F54
 * diagnostic helper this does not recalibrate, rezero or reconfigure the
 * sensor and never touches flash/configuration functions.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/rmi.h>
#include <linux/workqueue.h>

#include "rmi_bus.h"
#include "rmi_driver.h"

#define MEDIAPAD_TOUCH_POLL_MS 16
#define MEDIAPAD_TOUCH_RETRY_MS 100

static struct delayed_work mediapad_touch_poll_work;
static struct rmi_device *mediapad_touch_rmi;
static bool mediapad_touch_announced;

static int mediapad_match_physical_rmi(struct device *dev, void *data)
{
	return rmi_is_physical_device(dev);
}

static int mediapad_poll_f11(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function_handler *handler;
	struct rmi_function *fn;
	int count = 0;
	int ret;

	if (!drvdata)
		return -EAGAIN;

	list_for_each_entry(fn, &drvdata->function_list, node) {
		if (fn->fd.function_number != 0x11 || !fn->dev.driver)
			continue;

		handler = to_rmi_function_handler(fn->dev.driver);
		if (!handler->attention)
			continue;

		/* Force only F11's normal attention handler; it performs the same
		 * data reads/reporting as the IRQ-driven RMI core path. */
		ret = handler->attention(fn, fn->irq_mask);
		if (ret < 0)
			return ret;
		count++;
	}

	if (!count)
		return -ENODEV;

	if (drvdata->input)
		input_sync(drvdata->input);

	return 0;
}

static void mediapad_touch_poll(struct work_struct *work)
{
	struct device *dev;
	int ret;

	if (!mediapad_touch_rmi) {
		dev = bus_find_device(&rmi_bus_type, NULL, NULL,
				      mediapad_match_physical_rmi);
		if (!dev) {
			schedule_delayed_work(&mediapad_touch_poll_work,
				msecs_to_jiffies(MEDIAPAD_TOUCH_RETRY_MS));
			return;
		}

		/* Keep the reference returned by bus_find_device for the lifetime
		 * of this built-in diagnostic fallback. */
		mediapad_touch_rmi = to_rmi_device(dev);
		pr_info("HI3620-TOUCH-POLL: attached to %s, F11 every %ums\n",
			dev_name(dev), MEDIAPAD_TOUCH_POLL_MS);
	}

	ret = mediapad_poll_f11(mediapad_touch_rmi);
	if (!ret && !mediapad_touch_announced) {
		mediapad_touch_announced = true;
		pr_info("HI3620-TOUCH-POLL: F11 polling fallback active\n");
	} else if (ret && ret != -EAGAIN && ret != -ENODEV) {
		pr_warn_ratelimited("HI3620-TOUCH-POLL: F11 attention failed: %d\n",
				    ret);
	}

	schedule_delayed_work(&mediapad_touch_poll_work,
		msecs_to_jiffies(MEDIAPAD_TOUCH_POLL_MS));
}

static int __init mediapad_touch_poll_init(void)
{
	if (!of_machine_is_compatible("huawei,s10-101x"))
		return 0;

	INIT_DELAYED_WORK(&mediapad_touch_poll_work, mediapad_touch_poll);
	schedule_delayed_work(&mediapad_touch_poll_work,
		msecs_to_jiffies(MEDIAPAD_TOUCH_RETRY_MS));
	pr_info("HI3620-TOUCH-POLL: safe F11 polling fallback enabled\n");
	return 0;
}
late_initcall(mediapad_touch_poll_init);
