// SPDX-License-Identifier: GPL-2.0
/*
 * Temporary Huawei MediaPad 10 FHD RMI polling/diagnostic fallback.
 *
 * The Synaptics TM2263-002 enumerates correctly over the GPIO-backed I2C bus,
 * but touch reports are not reaching userspace.  The first fallback only
 * polled F01 interrupt status; that still depends on the controller asserting
 * the F11/F12 interrupt-status bits.  For this diagnostic build we also invoke
 * the 2-D function attention callback directly at 100 Hz and trace the first
 * bytes of the F11 data window when they change.
 *
 * This is deliberately scoped to the MediaPad machine only and is not meant as
 * a final upstream solution.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/rmi.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "rmi_bus.h"
#include "rmi_driver.h"

#define MEDIAPAD_RMI_POLL_MS 10
#define MEDIAPAD_RMI_RETRY_MS 100
#define MEDIAPAD_RMI_RAW_BYTES 16

static struct delayed_work mediapad_rmi_poll_work;
static struct rmi_device *mediapad_rmi_dev;
static unsigned int mediapad_poll_count;
static u8 mediapad_last_raw[MEDIAPAD_RMI_RAW_BYTES];
static bool mediapad_last_raw_valid;

static int mediapad_match_physical_rmi(struct device *dev, void *data)
{
	return rmi_is_physical_device(dev);
}

static void mediapad_dump_functions(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *fn;

	if (!drvdata)
		return;

	list_for_each_entry(fn, &drvdata->function_list, node) {
		pr_info("HI3620-RMI-FN: F%02x q=%04x c=%04x d=%04x cmd=%04x irq_pos=%u irq0=%lx bound=%d\n",
			fn->fd.function_number,
			fn->fd.query_base_addr,
			fn->fd.control_base_addr,
			fn->fd.data_base_addr,
			fn->fd.command_base_addr,
			fn->irq_pos,
			fn->irq_mask[0],
			!!fn->dev.driver);
	}
}

static int mediapad_trace_f11_raw(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *fn;
	u8 raw[MEDIAPAD_RMI_RAW_BYTES];
	int ret;

	if (!drvdata)
		return -EAGAIN;

	list_for_each_entry(fn, &drvdata->function_list, node) {
		if (fn->fd.function_number != 0x11)
			continue;

		ret = rmi_read_block(rmi_dev, fn->fd.data_base_addr,
				     raw, sizeof(raw));
		if (ret < 0)
			return ret;

		if (!mediapad_last_raw_valid ||
		    memcmp(raw, mediapad_last_raw, sizeof(raw))) {
			pr_info("HI3620-RMI-F11-RAW: d=%04x bytes=%*phN\n",
				fn->fd.data_base_addr, (int)sizeof(raw), raw);
			memcpy(mediapad_last_raw, raw, sizeof(raw));
			mediapad_last_raw_valid = true;
		}
		return 0;
	}

	return -ENODEV;
}

static int mediapad_force_2d_attention(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function_handler *handler;
	struct rmi_function *fn;
	int ret = 0;
	int count = 0;

	if (!drvdata)
		return -EAGAIN;

	/* Prefer F11 on this panel.  If it is present and bound, drive its normal
	 * attention callback directly, independent of F01 interrupt status. */
	list_for_each_entry(fn, &drvdata->function_list, node) {
		if (fn->fd.function_number != 0x11 || !fn->dev.driver)
			continue;

		handler = to_rmi_function_handler(fn->dev.driver);
		if (!handler->attention)
			continue;

		ret = handler->attention(fn, fn->irq_mask);
		if (ret < 0)
			return ret;
		count++;
	}

	/* Some RMI4 panels expose F12 instead of F11.  Only fall back to F12 when
	 * there was no bound F11, avoiding duplicate reports. */
	if (!count) {
		list_for_each_entry(fn, &drvdata->function_list, node) {
			if (fn->fd.function_number != 0x12 || !fn->dev.driver)
				continue;

			handler = to_rmi_function_handler(fn->dev.driver);
			if (!handler->attention)
				continue;

			ret = handler->attention(fn, fn->irq_mask);
			if (ret < 0)
				return ret;
			count++;
		}
	}

	if (!count)
		return -ENODEV;

	if (drvdata->input)
		input_sync(drvdata->input);

	return 0;
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
		pr_info("HI3620-RMI-POLL2: attached to %s, interval=%ums\n",
			dev_name(dev), MEDIAPAD_RMI_POLL_MS);
		mediapad_dump_functions(mediapad_rmi_dev);
	}

	/* Keep the normal path active so reset/status handling still happens. */
	ret = rmi_process_interrupt_requests(mediapad_rmi_dev);
	if (ret)
		pr_warn_ratelimited("HI3620-RMI-POLL2: status process failed: %d\n", ret);

	ret = mediapad_trace_f11_raw(mediapad_rmi_dev);
	if (ret && ret != -ENODEV)
		pr_warn_ratelimited("HI3620-RMI-POLL2: F11 raw read failed: %d\n", ret);

	ret = mediapad_force_2d_attention(mediapad_rmi_dev);
	if (ret)
		pr_warn_ratelimited("HI3620-RMI-POLL2: forced 2D attention failed: %d\n", ret);
	else if (mediapad_poll_count++ == 0)
		pr_info("HI3620-RMI-POLL2: forced 2D polling active\n");

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
	pr_info("HI3620-RMI-POLL2: MediaPad forced-data fallback enabled\n");
	return 0;
}
late_initcall(mediapad_rmi_poll_init);
