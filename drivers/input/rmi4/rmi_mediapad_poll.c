// SPDX-License-Identifier: GPL-2.0
/*
 * Temporary Huawei MediaPad 10 FHD RMI polling/diagnostic fallback.
 *
 * TM2263-002 enumerates correctly over the GPIO-backed I2C bus, but its F11
 * data window has stayed all-zero even while the panel is being touched.  The
 * vendor Huawei RMI stack used this controller in normal/full-power mode and
 * also exposed the F11 rezero command.  Force those conservative runtime
 * settings once, then keep polling F11 directly so bring-up does not depend on
 * the still-unverified ATTN IRQ path.
 */

#include <linux/delay.h>
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

#define RMI_F01_SLEEP_MODE_MASK 0x03
#define RMI_F01_NOSLEEP_BIT     BIT(2)
#define RMI_F01_CONFIGURED_BIT  BIT(7)
#define RMI_F11_REPORT_MODE_MASK 0x07
#define RMI_F11_REZERO          0x01

static struct delayed_work mediapad_rmi_poll_work;
static struct rmi_device *mediapad_rmi_dev;
static unsigned int mediapad_poll_count;
static u8 mediapad_last_raw[MEDIAPAD_RMI_RAW_BYTES];
static bool mediapad_last_raw_valid;
static bool mediapad_runtime_forced;

static int mediapad_match_physical_rmi(struct device *dev, void *data)
{
	return rmi_is_physical_device(dev);
}

static struct rmi_function *mediapad_find_function(struct rmi_device *rmi_dev,
						   u8 function_number)
{
	struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *fn;

	if (!drvdata)
		return NULL;

	list_for_each_entry(fn, &drvdata->function_list, node)
		if (fn->fd.function_number == function_number)
			return fn;

	return NULL;
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

static int mediapad_force_runtime_mode(struct rmi_device *rmi_dev)
{
	struct rmi_function *f01 = mediapad_find_function(rmi_dev, 0x01);
	struct rmi_function *f11 = mediapad_find_function(rmi_dev, 0x11);
	struct rmi_function *f34 = mediapad_find_function(rmi_dev, 0x34);
	u8 f01_status = 0;
	u8 f01_ctrl[2] = { 0 };
	u8 f11_ctrl[12] = { 0 };
	u8 f11_query[16] = { 0 };
	u8 f34_config[4] = { 0 };
	u8 new_ctrl;
	int ret;

	if (!f01 || !f11 || !f11->dev.driver)
		return -EAGAIN;

	ret = rmi_read(rmi_dev, f01->fd.data_base_addr, &f01_status);
	if (ret < 0)
		return ret;

	ret = rmi_read_block(rmi_dev, f01->fd.control_base_addr,
			     f01_ctrl, sizeof(f01_ctrl));
	if (ret < 0)
		return ret;

	ret = rmi_read_block(rmi_dev, f11->fd.query_base_addr,
			     f11_query, sizeof(f11_query));
	if (ret < 0)
		return ret;

	ret = rmi_read_block(rmi_dev, f11->fd.control_base_addr,
			     f11_ctrl, sizeof(f11_ctrl));
	if (ret < 0)
		return ret;

	if (f34) {
		ret = rmi_read_block(rmi_dev, f34->fd.control_base_addr,
				     f34_config, sizeof(f34_config));
		if (ret < 0)
			memset(f34_config, 0xff, sizeof(f34_config));
	}

	pr_info("HI3620-RMI-STATE[before]: F01stat=%02x F01ctrl=%02x irqen=%02x F11q=%*phN F11ctrl=%*phN F34cfg=%*phN\n",
		f01_status, f01_ctrl[0], f01_ctrl[1],
		(int)sizeof(f11_query), f11_query,
		(int)sizeof(f11_ctrl), f11_ctrl,
		(int)sizeof(f34_config), f34_config);

	/* Keep the whole RMI sensor awake and explicitly mark it configured. */
	new_ctrl = f01_ctrl[0];
	new_ctrl &= ~RMI_F01_SLEEP_MODE_MASK;
	new_ctrl |= RMI_F01_NOSLEEP_BIT | RMI_F01_CONFIGURED_BIT;
	ret = rmi_write(rmi_dev, f01->fd.control_base_addr, new_ctrl);
	if (ret < 0)
		return ret;

	/* Enable the F11 source as the vendor stack did.  Direct polling does not
	 * require the IRQ, but some firmware only advances reports with the source
	 * enabled. */
	new_ctrl = f01_ctrl[1] | (u8)f11->irq_mask[0];
	ret = rmi_write(rmi_dev, f01->fd.control_base_addr + 1, new_ctrl);
	if (ret < 0)
		return ret;

	/* F11 report mode 0 is continuous reporting.  Preserve all filter bits. */
	new_ctrl = f11_ctrl[0] & ~RMI_F11_REPORT_MODE_MASK;
	ret = rmi_write(rmi_dev, f11->fd.control_base_addr, new_ctrl);
	if (ret < 0)
		return ret;

	/* Recalibrate after the rails/reset churn used during early bring-up. */
	ret = rmi_write(rmi_dev, f11->fd.command_base_addr, RMI_F11_REZERO);
	if (ret < 0)
		return ret;
	msleep(50);

	ret = rmi_read_block(rmi_dev, f01->fd.control_base_addr,
			     f01_ctrl, sizeof(f01_ctrl));
	if (ret < 0)
		return ret;
	ret = rmi_read_block(rmi_dev, f11->fd.control_base_addr,
			     f11_ctrl, sizeof(f11_ctrl));
	if (ret < 0)
		return ret;
	ret = rmi_read(rmi_dev, f01->fd.data_base_addr, &f01_status);
	if (ret < 0)
		return ret;

	pr_info("HI3620-RMI-STATE[forced]: F01stat=%02x F01ctrl=%02x irqen=%02x F11ctrl=%*phN\n",
		f01_status, f01_ctrl[0], f01_ctrl[1],
		(int)sizeof(f11_ctrl), f11_ctrl);
	pr_info("HI3620-RMI-SCAN: full-power + continuous + rezero applied\n");

	return 0;
}

static int mediapad_trace_f11_raw(struct rmi_device *rmi_dev)
{
	struct rmi_function *fn = mediapad_find_function(rmi_dev, 0x11);
	u8 raw[MEDIAPAD_RMI_RAW_BYTES];
	int ret;

	if (!fn)
		return -ENODEV;

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

static int mediapad_force_2d_attention(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function_handler *handler;
	struct rmi_function *fn;
	int ret = 0;
	int count = 0;

	if (!drvdata)
		return -EAGAIN;

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
		pr_info("HI3620-RMI-POLL3: attached to %s, interval=%ums\n",
			dev_name(dev), MEDIAPAD_RMI_POLL_MS);
		mediapad_dump_functions(mediapad_rmi_dev);
	}

	if (!mediapad_runtime_forced) {
		ret = mediapad_force_runtime_mode(mediapad_rmi_dev);
		if (!ret)
			mediapad_runtime_forced = true;
		else if (ret != -EAGAIN)
			pr_warn_ratelimited("HI3620-RMI-POLL3: runtime force failed: %d\n", ret);
	}

	ret = rmi_process_interrupt_requests(mediapad_rmi_dev);
	if (ret)
		pr_warn_ratelimited("HI3620-RMI-POLL3: status process failed: %d\n", ret);

	ret = mediapad_trace_f11_raw(mediapad_rmi_dev);
	if (ret && ret != -ENODEV)
		pr_warn_ratelimited("HI3620-RMI-POLL3: F11 raw read failed: %d\n", ret);

	ret = mediapad_force_2d_attention(mediapad_rmi_dev);
	if (ret)
		pr_warn_ratelimited("HI3620-RMI-POLL3: forced 2D attention failed: %d\n", ret);
	else if (mediapad_poll_count++ == 0)
		pr_info("HI3620-RMI-POLL3: forced 2D polling active\n");

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
	pr_info("HI3620-RMI-POLL3: MediaPad full-power scan fallback enabled\n");
	return 0;
}
late_initcall(mediapad_rmi_poll_init);
