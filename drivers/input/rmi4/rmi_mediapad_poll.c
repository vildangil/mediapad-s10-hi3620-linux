// SPDX-License-Identifier: GPL-2.0
/*
 * Temporary Huawei MediaPad 10 FHD RMI polling/diagnostic fallback.
 *
 * TM2263-002 enumerates correctly over the GPIO-backed I2C bus, but its F11
 * data window stays all-zero while the panel is touched.  Keep the conservative
 * full-power/F11 polling fallback, and additionally inspect F54 because Huawei's
 * stock kernel binds F54 while upstream 4.9 leaves it unbound here.
 *
 * F54 control 0 contains the runtime "no scan" bit.  This diagnostic clears
 * that bit when needed, issues FORCE_CAL, and takes a few 16-bit image reports.
 * None of this writes F34 or flash configuration.
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

#define MEDIAPAD_RMI_POLL_MS            10
#define MEDIAPAD_RMI_RETRY_MS           100
#define MEDIAPAD_RMI_RAW_BYTES          16

#define RMI_F01_SLEEP_MODE_MASK         0x03
#define RMI_F01_NOSLEEP_BIT             BIT(2)
#define RMI_F01_CONFIGURED_BIT          BIT(7)

#define RMI_F11_REPORT_MODE_MASK        0x07
#define RMI_F11_REZERO                  0x01

#define RMI_F54_NO_SCAN_BIT             BIT(1)
#define RMI_F54_GET_REPORT              BIT(0)
#define RMI_F54_FORCE_CAL               BIT(1)
#define RMI_F54_REPORT_16BIT_IMAGE      2
#define RMI_F54_REPORT_DATA_OFFSET      3
#define RMI_F54_FIFO_OFFSET             1
#define RMI_F54_QUERY_BYTES             14

/* A 30x46 matrix is 2760 bytes.  Keep enough room for nearby variants. */
#define MEDIAPAD_F54_MAX_REPORT_BYTES   4096
#define MEDIAPAD_F54_SAMPLE_EVERY_POLLS 100 /* roughly once per second */
#define MEDIAPAD_F54_MAX_SAMPLES        20

static struct delayed_work mediapad_rmi_poll_work;
static struct rmi_device *mediapad_rmi_dev;
static unsigned int mediapad_poll_count;
static u8 mediapad_last_raw[MEDIAPAD_RMI_RAW_BYTES];
static bool mediapad_last_raw_valid;
static bool mediapad_runtime_forced;

static u8 mediapad_f54_rx;
static u8 mediapad_f54_tx;
static bool mediapad_f54_image16;
static unsigned int mediapad_f54_sample_count;
/* Static buffers avoid putting several KiB on the kernel stack. */
static u8 mediapad_f54_report[MEDIAPAD_F54_MAX_REPORT_BYTES];
static u8 mediapad_f54_prev[MEDIAPAD_F54_MAX_REPORT_BYTES];
static size_t mediapad_f54_prev_size;
static bool mediapad_f54_prev_valid;

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

static int mediapad_f54_wait_idle(struct rmi_device *rmi_dev,
				  struct rmi_function *f54,
				  unsigned int timeout_ms)
{
	unsigned int waited = 0;
	u8 command;
	int ret;

	while (waited < timeout_ms) {
		ret = rmi_read(rmi_dev, f54->fd.command_base_addr, &command);
		if (ret < 0)
			return ret;
		if (!command)
			return 0;
		msleep(5);
		waited += 5;
	}

	ret = rmi_read(rmi_dev, f54->fd.command_base_addr, &command);
	if (ret < 0)
		return ret;

	pr_warn("HI3620-RMI-F54: command timeout cmd=%02x after %ums\n",
		command, timeout_ms);
	return -ETIMEDOUT;
}

static int mediapad_prepare_f54(struct rmi_device *rmi_dev)
{
	struct rmi_function *f54 = mediapad_find_function(rmi_dev, 0x54);
	u8 query[RMI_F54_QUERY_BYTES] = { 0 };
	u8 ctrl0 = 0;
	u8 command = 0;
	u8 new_ctrl0;
	int ret;

	if (!f54)
		return -ENODEV;

	ret = rmi_read_block(rmi_dev, f54->fd.query_base_addr,
			     query, sizeof(query));
	if (ret < 0)
		return ret;

	ret = rmi_read(rmi_dev, f54->fd.control_base_addr, &ctrl0);
	if (ret < 0)
		return ret;

	ret = rmi_read(rmi_dev, f54->fd.command_base_addr, &command);
	if (ret < 0)
		return ret;

	/* Huawei F54 query 0/1 are RX/TX counts; query2 bit6 is image16. */
	mediapad_f54_rx = query[0];
	mediapad_f54_tx = query[1];
	mediapad_f54_image16 = !!(query[2] & BIT(6));

	pr_info("HI3620-RMI-F54[before]: q=%*phN rx=%u tx=%u image16=%u ctrl0=%02x no_scan=%u cmd=%02x\n",
		(int)sizeof(query), query,
		mediapad_f54_rx, mediapad_f54_tx,
		mediapad_f54_image16, ctrl0,
		!!(ctrl0 & RMI_F54_NO_SCAN_BIT), command);

	/* Volatile F54 runtime control only; F34/flash is deliberately untouched. */
	if (ctrl0 & RMI_F54_NO_SCAN_BIT) {
		new_ctrl0 = ctrl0 & ~RMI_F54_NO_SCAN_BIT;
		ret = rmi_write(rmi_dev, f54->fd.control_base_addr, new_ctrl0);
		if (ret < 0)
			return ret;

		ret = rmi_read(rmi_dev, f54->fd.control_base_addr, &ctrl0);
		if (ret < 0)
			return ret;

		pr_info("HI3620-RMI-F54: cleared no_scan, ctrl0=%02x\n", ctrl0);
	}

	/* Recalibrate the analog front-end after the early hard power/reset. */
	ret = rmi_write(rmi_dev, f54->fd.command_base_addr, RMI_F54_FORCE_CAL);
	if (ret < 0)
		return ret;

	ret = mediapad_f54_wait_idle(rmi_dev, f54, 1000);
	if (ret < 0)
		return ret;

	ret = rmi_read(rmi_dev, f54->fd.control_base_addr, &ctrl0);
	if (ret < 0)
		return ret;

	pr_info("HI3620-RMI-F54[ready]: ctrl0=%02x no_scan=%u force_cal=done\n",
		ctrl0, !!(ctrl0 & RMI_F54_NO_SCAN_BIT));
	return 0;
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
	int f54_ret;

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

	new_ctrl = f01_ctrl[0];
	new_ctrl &= ~RMI_F01_SLEEP_MODE_MASK;
	new_ctrl |= RMI_F01_NOSLEEP_BIT | RMI_F01_CONFIGURED_BIT;
	ret = rmi_write(rmi_dev, f01->fd.control_base_addr, new_ctrl);
	if (ret < 0)
		return ret;

	new_ctrl = f01_ctrl[1] | (u8)f11->irq_mask[0];
	ret = rmi_write(rmi_dev, f01->fd.control_base_addr + 1, new_ctrl);
	if (ret < 0)
		return ret;

	/* F11 report mode zero is continuous reporting; preserve the other bits. */
	new_ctrl = f11_ctrl[0] & ~RMI_F11_REPORT_MODE_MASK;
	ret = rmi_write(rmi_dev, f11->fd.control_base_addr, new_ctrl);
	if (ret < 0)
		return ret;

	f54_ret = mediapad_prepare_f54(rmi_dev);
	if (f54_ret && f54_ret != -ENODEV)
		pr_warn("HI3620-RMI-F54: preparation failed: %d\n", f54_ret);

	/* Rezero F11 after F54 scan state/calibration has been handled. */
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
	pr_info("HI3620-RMI-SCAN4: full-power + F54 scan/cal + F11 continuous/rezero applied\n");

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

static int mediapad_sample_f54_image(struct rmi_device *rmi_dev)
{
	struct rmi_function *f54 = mediapad_find_function(rmi_dev, 0x54);
	u8 fifo[2] = { 0, 0 };
	size_t cells;
	size_t bytes;
	size_t i;
	u32 hash = 2166136261U;
	u32 changed = 0;
	u32 diff_sum = 0;
	u8 diff_max = 0;
	int ret;

	if (!f54 || !mediapad_f54_image16 ||
	    !mediapad_f54_rx || !mediapad_f54_tx)
		return -ENODEV;

	cells = (size_t)mediapad_f54_rx * mediapad_f54_tx;
	bytes = cells * 2;
	if (!cells || bytes > sizeof(mediapad_f54_report)) {
		pr_warn_once("HI3620-RMI-F54: image too large rx=%u tx=%u bytes=%zu\n",
			     mediapad_f54_rx, mediapad_f54_tx, bytes);
		return -E2BIG;
	}

	ret = rmi_write(rmi_dev, f54->fd.data_base_addr,
			RMI_F54_REPORT_16BIT_IMAGE);
	if (ret < 0)
		return ret;

	ret = rmi_write_block(rmi_dev,
			      f54->fd.data_base_addr + RMI_F54_FIFO_OFFSET,
			      fifo, sizeof(fifo));
	if (ret < 0)
		return ret;

	ret = rmi_write(rmi_dev, f54->fd.command_base_addr, RMI_F54_GET_REPORT);
	if (ret < 0)
		return ret;

	ret = mediapad_f54_wait_idle(rmi_dev, f54, 1000);
	if (ret < 0)
		return ret;

	ret = rmi_read_block(rmi_dev,
			     f54->fd.data_base_addr + RMI_F54_REPORT_DATA_OFFSET,
			     mediapad_f54_report, bytes);
	if (ret < 0)
		return ret;

	for (i = 0; i < bytes; i++) {
		u8 v = mediapad_f54_report[i];

		hash ^= v;
		hash *= 16777619U;

		if (mediapad_f54_prev_valid && mediapad_f54_prev_size == bytes) {
			u8 old = mediapad_f54_prev[i];
			u8 diff = v >= old ? v - old : old - v;

			if (diff) {
				changed++;
				diff_sum += diff;
				if (diff > diff_max)
					diff_max = diff;
			}
		}
	}

	pr_info("HI3620-RMI-F54-IMAGE[%u]: rx=%u tx=%u bytes=%zu hash=%08x changed=%u diff_sum=%u diff_max=%u head=%*phN\n",
		mediapad_f54_sample_count,
		mediapad_f54_rx, mediapad_f54_tx, bytes,
		hash, changed, diff_sum, diff_max,
		(int)min_t(size_t, bytes, 16), mediapad_f54_report);

	memcpy(mediapad_f54_prev, mediapad_f54_report, bytes);
	mediapad_f54_prev_size = bytes;
	mediapad_f54_prev_valid = true;
	mediapad_f54_sample_count++;
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
		pr_info("HI3620-RMI-POLL4: attached to %s, interval=%ums\n",
			dev_name(dev), MEDIAPAD_RMI_POLL_MS);
		mediapad_dump_functions(mediapad_rmi_dev);
	}

	if (!mediapad_runtime_forced) {
		ret = mediapad_force_runtime_mode(mediapad_rmi_dev);
		if (!ret)
			mediapad_runtime_forced = true;
		else if (ret != -EAGAIN)
			pr_warn_ratelimited("HI3620-RMI-POLL4: runtime force failed: %d\n",
					    ret);
	}

	ret = rmi_process_interrupt_requests(mediapad_rmi_dev);
	if (ret)
		pr_warn_ratelimited("HI3620-RMI-POLL4: status process failed: %d\n", ret);

	ret = mediapad_trace_f11_raw(mediapad_rmi_dev);
	if (ret && ret != -ENODEV)
		pr_warn_ratelimited("HI3620-RMI-POLL4: F11 raw read failed: %d\n", ret);

	ret = mediapad_force_2d_attention(mediapad_rmi_dev);
	if (ret)
		pr_warn_ratelimited("HI3620-RMI-POLL4: forced 2D attention failed: %d\n", ret);
	else if (mediapad_poll_count == 0)
		pr_info("HI3620-RMI-POLL4: forced 2D polling active\n");

	/* Full matrix transfer is slow on bit-banged I2C, so sample sparingly. */
	if (mediapad_runtime_forced &&
	    mediapad_f54_sample_count < MEDIAPAD_F54_MAX_SAMPLES &&
	    !(mediapad_poll_count % MEDIAPAD_F54_SAMPLE_EVERY_POLLS)) {
		ret = mediapad_sample_f54_image(mediapad_rmi_dev);
		if (ret && ret != -ENODEV)
			pr_warn("HI3620-RMI-POLL4: F54 image failed: %d\n", ret);
	}

	mediapad_poll_count++;
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
	pr_info("HI3620-RMI-POLL4: MediaPad F54 scan/image diagnostic enabled\n");
	return 0;
}
late_initcall(mediapad_rmi_poll_init);
