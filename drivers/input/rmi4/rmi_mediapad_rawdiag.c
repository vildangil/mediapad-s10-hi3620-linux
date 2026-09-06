// SPDX-License-Identifier: GPL-2.0
/*
 * Temporary Huawei MediaPad 10 FHD Synaptics TM2263 diagnostic.
 *
 * The controller enumerates and exposes F11, but F11 finger data remains zero.
 * F54 TRUE_BASELINE is populated while RAW_16BIT_IMAGE is suspiciously flat.
 * Sample the Huawei factory-test FULL_RAW_CAP reports (19/20) and, in parallel,
 * watch F01 status/control/IRQ state and F11 data at a much higher rate.
 *
 * This is deliberately volatile: F34 flash/config and F55 assignments are
 * read-only here and are never modified.
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

#define MEDIAPAD_RETRY_MS                  100
#define MEDIAPAD_MONITOR_MS                 50
#define MEDIAPAD_REPORT_EVERY_TICKS         30 /* about 1.5 s */
#define MEDIAPAD_MAX_MONITOR_TICKS         700 /* at least 35 s */
#define MEDIAPAD_MAX_REPORTS                20
#define MEDIAPAD_F11_BYTES                  16
#define MEDIAPAD_F54_QUERY_BYTES            14
#define MEDIAPAD_F54_MAX_REPORT_BYTES     4096

#define RMI_F01_SLEEP_MODE_MASK           0x03
#define RMI_F01_NOSLEEP_BIT               BIT(2)
#define RMI_F01_CONFIGURED_BIT            BIT(7)

#define RMI_F11_REPORT_MODE_MASK          0x07
#define RMI_F11_REZERO                    0x01

#define RMI_F54_GET_REPORT                1
#define RMI_F54_FORCE_CAL                 2
#define RMI_F54_FULL_RAW_CAP             19
#define RMI_F54_FULL_RAW_CAP_RX_REMOVED  20
#define RMI_F54_FIFO_OFFSET               1
#define RMI_F54_REPORT_DATA_OFFSET        3
#define RMI_F54_NO_SCAN_BIT               BIT(1)

static struct delayed_work mediapad_diag_work;
static struct rmi_device *mediapad_rmi_dev;
static unsigned int mediapad_tick;
static unsigned int mediapad_report_count;
static u8 mediapad_report[MEDIAPAD_F54_MAX_REPORT_BYTES];
static u8 mediapad_prev19[MEDIAPAD_F54_MAX_REPORT_BYTES];
static u8 mediapad_prev20[MEDIAPAD_F54_MAX_REPORT_BYTES];
static size_t mediapad_prev19_size;
static size_t mediapad_prev20_size;
static bool mediapad_prev19_valid;
static bool mediapad_prev20_valid;
static u8 mediapad_last_f11[MEDIAPAD_F11_BYTES];
static bool mediapad_last_f11_valid;
static u8 mediapad_last_status;
static u8 mediapad_last_ctrl;
static u8 mediapad_last_irq;
static bool mediapad_last_live_valid;
static bool mediapad_prepared;

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

	list_for_each_entry(fn, &drvdata->function_list, node)
		pr_info("HI3620-RMI-LIVE-FN: F%02x q=%04x c=%04x d=%04x cmd=%04x irq=%lx bound=%d\n",
			fn->fd.function_number, fn->fd.query_base_addr,
			fn->fd.control_base_addr, fn->fd.data_base_addr,
			fn->fd.command_base_addr, fn->irq_mask[0],
			!!fn->dev.driver);
}

static int mediapad_f54_wait_idle(struct rmi_device *rmi_dev,
				  struct rmi_function *f54,
				  unsigned int timeout_ms)
{
	unsigned int waited = 0;
	u8 command = 0;
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

	pr_warn("HI3620-RMI-LIVE: F54 command timeout cmd=%02x\n", command);
	return -ETIMEDOUT;
}

static void mediapad_dump_readonly_blocks(struct rmi_device *rmi_dev,
					 struct rmi_function *f54)
{
	struct rmi_function *f55 = mediapad_find_function(rmi_dev, 0x55);
	u8 buf[64];
	int ret;

	ret = rmi_read_block(rmi_dev, f54->fd.control_base_addr, buf, sizeof(buf));
	if (!ret)
		pr_info("HI3620-RMI-F54-CTRL64: base=%04x bytes=%*phN\n",
			f54->fd.control_base_addr, (int)sizeof(buf), buf);
	else
		pr_warn("HI3620-RMI-LIVE: F54 control dump failed: %d\n", ret);

	if (!f55)
		return;

	ret = rmi_read_block(rmi_dev, f55->fd.query_base_addr, buf, 16);
	if (!ret)
		pr_info("HI3620-RMI-F55-Q16: base=%04x bytes=%*phN\n",
			f55->fd.query_base_addr, 16, buf);

	ret = rmi_read_block(rmi_dev, f55->fd.control_base_addr, buf, sizeof(buf));
	if (!ret)
		pr_info("HI3620-RMI-F55-C64: base=%04x bytes=%*phN\n",
			f55->fd.control_base_addr, (int)sizeof(buf), buf);
}

static int mediapad_read_core_state(struct rmi_device *rmi_dev,
				    struct rmi_function *f01,
				    struct rmi_function *f11,
				    const char *tag)
{
	u8 ctrl[2] = { 0 };
	u8 status = 0;
	u8 f11ctrl[12] = { 0 };
	int ret;

	ret = rmi_read_block(rmi_dev, f01->fd.control_base_addr,
			     ctrl, sizeof(ctrl));
	if (ret < 0)
		return ret;
	ret = rmi_read(rmi_dev, f01->fd.data_base_addr, &status);
	if (ret < 0)
		return ret;
	ret = rmi_read_block(rmi_dev, f11->fd.control_base_addr,
			     f11ctrl, sizeof(f11ctrl));
	if (ret < 0)
		return ret;

	pr_info("HI3620-RMI-CORE[%s]: F01stat=%02x F01ctrl=%02x irqen=%02x F11ctrl=%*phN\n",
		tag, status, ctrl[0], ctrl[1],
		(int)sizeof(f11ctrl), f11ctrl);
	return 0;
}

static int mediapad_prepare(struct rmi_device *rmi_dev)
{
	struct rmi_function *f01 = mediapad_find_function(rmi_dev, 0x01);
	struct rmi_function *f11 = mediapad_find_function(rmi_dev, 0x11);
	struct rmi_function *f34 = mediapad_find_function(rmi_dev, 0x34);
	struct rmi_function *f54 = mediapad_find_function(rmi_dev, 0x54);
	u8 f54q[MEDIAPAD_F54_QUERY_BYTES] = { 0 };
	u8 f34cfg[4] = { 0xff, 0xff, 0xff, 0xff };
	u8 f01ctrl[2] = { 0 };
	u8 f11ctrl[12] = { 0 };
	u8 f01status = 0;
	u8 f54ctrl0 = 0;
	u8 newval;
	int ret;

	if (!f01 || !f11 || !f54 || !f11->dev.driver)
		return -EAGAIN;

	ret = rmi_read_block(rmi_dev, f54->fd.query_base_addr,
			     f54q, sizeof(f54q));
	if (ret < 0)
		return ret;
	ret = rmi_read_block(rmi_dev, f01->fd.control_base_addr,
			     f01ctrl, sizeof(f01ctrl));
	if (ret < 0)
		return ret;
	ret = rmi_read(rmi_dev, f01->fd.data_base_addr, &f01status);
	if (ret < 0)
		return ret;
	ret = rmi_read_block(rmi_dev, f11->fd.control_base_addr,
			     f11ctrl, sizeof(f11ctrl));
	if (ret < 0)
		return ret;
	ret = rmi_read(rmi_dev, f54->fd.control_base_addr, &f54ctrl0);
	if (ret < 0)
		return ret;
	if (f34)
		rmi_read_block(rmi_dev, f34->fd.control_base_addr,
			       f34cfg, sizeof(f34cfg));

	pr_info("HI3620-RMI-LIVE-STATE[before]: F01stat=%02x F01ctrl=%02x irqen=%02x F11ctrl=%*phN F54q=%*phN F54ctrl0=%02x F34cfg=%*phN\n",
		f01status, f01ctrl[0], f01ctrl[1],
		(int)sizeof(f11ctrl), f11ctrl,
		(int)sizeof(f54q), f54q, f54ctrl0,
		(int)sizeof(f34cfg), f34cfg);

	/* Full power and configured host state. */
	newval = f01ctrl[0] & ~RMI_F01_SLEEP_MODE_MASK;
	newval |= RMI_F01_NOSLEEP_BIT | RMI_F01_CONFIGURED_BIT;
	ret = rmi_write(rmi_dev, f01->fd.control_base_addr, newval);
	if (ret < 0)
		return ret;

	/* Ensure the F11 source is enabled even if an earlier reset lost the mask. */
	newval = f01ctrl[1] | (u8)f11->irq_mask[0];
	ret = rmi_write(rmi_dev, f01->fd.control_base_addr + 1, newval);
	if (ret < 0)
		return ret;

	/* Continuous F11 reporting, preserving every non-mode control bit. */
	newval = f11ctrl[0] & ~RMI_F11_REPORT_MODE_MASK;
	ret = rmi_write(rmi_dev, f11->fd.control_base_addr, newval);
	if (ret < 0)
		return ret;

	if (f54ctrl0 & RMI_F54_NO_SCAN_BIT) {
		newval = f54ctrl0 & ~RMI_F54_NO_SCAN_BIT;
		ret = rmi_write(rmi_dev, f54->fd.control_base_addr, newval);
		if (ret < 0)
			return ret;
	}

	ret = rmi_write(rmi_dev, f54->fd.command_base_addr, RMI_F54_FORCE_CAL);
	if (ret < 0)
		return ret;
	ret = mediapad_f54_wait_idle(rmi_dev, f54, 1000);
	if (ret < 0)
		return ret;

	ret = rmi_write(rmi_dev, f11->fd.command_base_addr, RMI_F11_REZERO);
	if (ret < 0)
		return ret;
	msleep(50);

	/* Reassert configured after the diagnostic calibration/rezero sequence. */
	ret = rmi_read(rmi_dev, f01->fd.control_base_addr, &newval);
	if (ret < 0)
		return ret;
	newval &= ~RMI_F01_SLEEP_MODE_MASK;
	newval |= RMI_F01_NOSLEEP_BIT | RMI_F01_CONFIGURED_BIT;
	ret = rmi_write(rmi_dev, f01->fd.control_base_addr, newval);
	if (ret < 0)
		return ret;

	mediapad_dump_readonly_blocks(rmi_dev, f54);
	mediapad_read_core_state(rmi_dev, f01, f11, "after");

	pr_info("HI3620-RMI-LIVE: prepared rx=%u tx=%u caps=%02x baseline=%u image16=%u; F34/F55 untouched\n",
		f54q[0], f54q[1], f54q[2], !!(f54q[2] & BIT(2)),
		!!(f54q[2] & BIT(6)));
	return 0;
}

static int mediapad_read_f54_report(struct rmi_device *rmi_dev,
				    struct rmi_function *f54,
				    u8 report_type,
				    size_t bytes)
{
	u8 fifo[2] = { 0, 0 };
	int ret;

	/* Match the standard/vendor F54 request order. */
	ret = rmi_write(rmi_dev, f54->fd.data_base_addr, report_type);
	if (ret < 0)
		return ret;
	usleep_range(2000, 3000);

	ret = rmi_write(rmi_dev, f54->fd.command_base_addr, RMI_F54_GET_REPORT);
	if (ret < 0)
		return ret;
	ret = mediapad_f54_wait_idle(rmi_dev, f54, 1000);
	if (ret < 0)
		return ret;

	ret = rmi_write_block(rmi_dev,
			      f54->fd.data_base_addr + RMI_F54_FIFO_OFFSET,
			      fifo, sizeof(fifo));
	if (ret < 0)
		return ret;

	return rmi_read_block(rmi_dev,
			      f54->fd.data_base_addr + RMI_F54_REPORT_DATA_OFFSET,
			      mediapad_report, bytes);
}

static void mediapad_log_report(u8 type, unsigned int sample,
				const u8 *buf, size_t bytes)
{
	u8 *prev = NULL;
	size_t *prev_size = NULL;
	bool *prev_valid = NULL;
	size_t cells = bytes / 2;
	size_t i;
	u32 hash = 2166136261U;
	u32 nonzero = 0;
	u32 changed = 0;
	u64 sum = 0;
	u64 diff_sum = 0;
	u16 minv = 0xffff;
	u16 maxv = 0;
	u16 diff_max = 0;

	if (type == RMI_F54_FULL_RAW_CAP) {
		prev = mediapad_prev19;
		prev_size = &mediapad_prev19_size;
		prev_valid = &mediapad_prev19_valid;
	} else if (type == RMI_F54_FULL_RAW_CAP_RX_REMOVED) {
		prev = mediapad_prev20;
		prev_size = &mediapad_prev20_size;
		prev_valid = &mediapad_prev20_valid;
	}

	for (i = 0; i < bytes; i++) {
		hash ^= buf[i];
		hash *= 16777619U;
	}

	for (i = 0; i < cells; i++) {
		u16 v = buf[i * 2] | ((u16)buf[i * 2 + 1] << 8);

		if (v)
			nonzero++;
		if (v < minv)
			minv = v;
		if (v > maxv)
			maxv = v;
		sum += v;

		if (prev && prev_valid && *prev_valid &&
		    prev_size && *prev_size == bytes) {
			u16 old = prev[i * 2] | ((u16)prev[i * 2 + 1] << 8);
			u16 diff = v >= old ? v - old : old - v;

			if (diff) {
				changed++;
				diff_sum += diff;
				if (diff > diff_max)
					diff_max = diff;
			}
		}
	}

	pr_info("HI3620-RMI-F54-FULL[%u]: type=%u cells=%zu nonzero=%u min=%u max=%u sum=%llu hash=%08x changed=%u diff_sum=%llu diff_max=%u head=%*phN\n",
		sample, type, cells, nonzero, minv, maxv,
		(unsigned long long)sum, hash, changed,
		(unsigned long long)diff_sum, diff_max,
		(int)min_t(size_t, bytes, 24), buf);

	if (prev && prev_size && prev_valid) {
		memcpy(prev, buf, bytes);
		*prev_size = bytes;
		*prev_valid = true;
	}
}

static int mediapad_dispatch_f11(struct rmi_device *rmi_dev,
				 struct rmi_function *f11)
{
	struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function_handler *handler;
	int ret;

	if (!f11 || !f11->dev.driver)
		return -ENODEV;

	handler = to_rmi_function_handler(f11->dev.driver);
	if (!handler->attention)
		return -ENODEV;

	ret = handler->attention(f11, f11->irq_mask);
	if (!ret && drvdata && drvdata->input)
		input_sync(drvdata->input);
	return ret;
}

static void mediapad_trace_live(struct rmi_device *rmi_dev)
{
	struct rmi_function *f01 = mediapad_find_function(rmi_dev, 0x01);
	struct rmi_function *f11 = mediapad_find_function(rmi_dev, 0x11);
	u8 raw[MEDIAPAD_F11_BYTES];
	u8 status = 0;
	u8 ctrl = 0;
	u8 irq = 0;
	int ret;

	if (!f01 || !f11)
		return;

	ret = rmi_read(rmi_dev, f01->fd.data_base_addr, &status);
	if (ret < 0)
		return;
	ret = rmi_read(rmi_dev, f01->fd.control_base_addr, &ctrl);
	if (ret < 0)
		return;

	/*
	 * Reading F01 interrupt status can consume the latched status.  When the
	 * F11 bit is observed below, dispatch F11 attention ourselves so this
	 * diagnostic does not steal a real touch report from the normal driver.
	 */
	ret = rmi_read(rmi_dev, f01->fd.data_base_addr + 1, &irq);
	if (ret < 0)
		return;

	if (!mediapad_last_live_valid || status != mediapad_last_status ||
	    ctrl != mediapad_last_ctrl || irq != mediapad_last_irq || irq) {
		pr_info("HI3620-RMI-LIVE[%u]: F01stat=%02x ctrl=%02x irq=%02x f11mask=%02lx configured=%u nosleep=%u\n",
			mediapad_tick, status, ctrl, irq, f11->irq_mask[0],
			!!(ctrl & RMI_F01_CONFIGURED_BIT),
			!!(ctrl & RMI_F01_NOSLEEP_BIT));
		mediapad_last_status = status;
		mediapad_last_ctrl = ctrl;
		mediapad_last_irq = irq;
		mediapad_last_live_valid = true;
	}

	ret = rmi_read_block(rmi_dev, f11->fd.data_base_addr, raw, sizeof(raw));
	if (!ret && (!mediapad_last_f11_valid ||
		    memcmp(raw, mediapad_last_f11, sizeof(raw)))) {
		pr_info("HI3620-RMI-F11-LIVE[%u]: bytes=%*phN\n",
			mediapad_tick, (int)sizeof(raw), raw);
		memcpy(mediapad_last_f11, raw, sizeof(raw));
		mediapad_last_f11_valid = true;
	}

	if (irq & (u8)f11->irq_mask[0]) {
		ret = mediapad_dispatch_f11(rmi_dev, f11);
		pr_info("HI3620-RMI-F11-IRQ[%u]: irq=%02x dispatch=%d\n",
			mediapad_tick, irq, ret);
	}
}

static void mediapad_diag(struct work_struct *work)
{
	struct rmi_function *f54;
	struct device *dev;
	u8 query[MEDIAPAD_F54_QUERY_BYTES];
	u8 type;
	size_t bytes;
	int ret;

	if (!mediapad_rmi_dev) {
		dev = bus_find_device(&rmi_bus_type, NULL, NULL,
				      mediapad_match_physical_rmi);
		if (!dev)
			goto retry;
		mediapad_rmi_dev = to_rmi_device(dev);
		pr_info("HI3620-RMI-LIVE: attached to %s\n", dev_name(dev));
		mediapad_dump_functions(mediapad_rmi_dev);
	}

	if (!mediapad_prepared) {
		ret = mediapad_prepare(mediapad_rmi_dev);
		if (ret == -EAGAIN)
			goto retry;
		if (ret < 0) {
			pr_warn("HI3620-RMI-LIVE: prepare failed: %d\n", ret);
			goto retry;
		}
		mediapad_prepared = true;
	}

	mediapad_trace_live(mediapad_rmi_dev);

	if (mediapad_report_count < MEDIAPAD_MAX_REPORTS &&
	    !(mediapad_tick % MEDIAPAD_REPORT_EVERY_TICKS)) {
		f54 = mediapad_find_function(mediapad_rmi_dev, 0x54);
		if (!f54)
			goto retry;

		ret = rmi_read_block(mediapad_rmi_dev, f54->fd.query_base_addr,
				     query, sizeof(query));
		if (ret < 0)
			goto retry;

		bytes = (size_t)query[0] * query[1] * 2;
		if (!bytes || bytes > sizeof(mediapad_report)) {
			pr_warn("HI3620-RMI-LIVE: invalid matrix %ux%u bytes=%zu\n",
				query[0], query[1], bytes);
			return;
		}

		/* Huawei's factory raw-capacitance path uses report type 19. */
		type = (mediapad_report_count & 1) ?
			RMI_F54_FULL_RAW_CAP_RX_REMOVED : RMI_F54_FULL_RAW_CAP;
		ret = mediapad_read_f54_report(mediapad_rmi_dev, f54, type, bytes);
		if (ret < 0) {
			pr_warn("HI3620-RMI-LIVE: report type %u failed: %d\n",
				type, ret);
		} else {
			mediapad_log_report(type, mediapad_report_count,
					    mediapad_report, bytes);
			mediapad_report_count++;
		}
	}

	mediapad_tick++;
	if (mediapad_tick >= MEDIAPAD_MAX_MONITOR_TICKS &&
	    mediapad_report_count >= MEDIAPAD_MAX_REPORTS) {
		pr_info("HI3620-RMI-LIVE: diagnostic complete ticks=%u reports=%u\n",
			mediapad_tick, mediapad_report_count);
		return;
	}

	schedule_delayed_work(&mediapad_diag_work,
		msecs_to_jiffies(MEDIAPAD_MONITOR_MS));
	return;

retry:
	schedule_delayed_work(&mediapad_diag_work,
		msecs_to_jiffies(MEDIAPAD_RETRY_MS));
}

static int __init mediapad_rawdiag_init(void)
{
	if (!of_machine_is_compatible("huawei,s10-101x"))
		return 0;

	INIT_DELAYED_WORK(&mediapad_diag_work, mediapad_diag);
	schedule_delayed_work(&mediapad_diag_work,
		msecs_to_jiffies(MEDIAPAD_RETRY_MS));
	pr_info("HI3620-RMI-LIVE: full-raw + F01/F11 IRQ diagnostic enabled\n");
	return 0;
}
late_initcall(mediapad_rawdiag_init);
