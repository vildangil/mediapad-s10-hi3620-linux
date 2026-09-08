// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD RMI4 F11 polling/trace fallback.
 *
 * TM2263-002 enumerates correctly but does not yet report contacts.  Keep a
 * low-overhead poller so the normal F11 attention handler is exercised even
 * if the GPIO157 IRQ path is imperfect, and log the actual F11 data bytes.
 * Do not rewrite F01/F11 runtime controls here: the stock Huawei driver leaves
 * the sensor's firmware-selected F11 report mode intact, and the previous
 * forced mode change did not produce contact data.
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

#define MEDIAPAD_TOUCH_POLL_MS          16
#define MEDIAPAD_TOUCH_RETRY_MS         100
#define MEDIAPAD_TOUCH_RAW_BYTES        16
#define MEDIAPAD_TOUCH_STATE_EVERY      250

static struct delayed_work mediapad_touch_poll_work;
static struct rmi_device *mediapad_touch_rmi;
static bool mediapad_touch_announced;
static bool mediapad_touch_described;
static unsigned int mediapad_touch_ticks;
static u8 mediapad_touch_last_raw[MEDIAPAD_TOUCH_RAW_BYTES];
static bool mediapad_touch_last_raw_valid;

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

static int mediapad_touch_describe(struct rmi_device *rmi_dev)
{
        struct rmi_function *f01 = mediapad_find_function(rmi_dev, 0x01);
        struct rmi_function *f11 = mediapad_find_function(rmi_dev, 0x11);
        u8 f01_ctrl[2];
        u8 f01_status;
        u8 f11_ctrl;
        u8 f11_query[16];
        int ret;

        if (!f01 || !f11 || !f11->dev.driver)
                return -EAGAIN;

        ret = rmi_read_block(rmi_dev, f01->fd.control_base_addr,
                             f01_ctrl, sizeof(f01_ctrl));
        if (ret < 0)
                return ret;
        ret = rmi_read(rmi_dev, f01->fd.data_base_addr, &f01_status);
        if (ret < 0)
                return ret;
        ret = rmi_read(rmi_dev, f11->fd.control_base_addr, &f11_ctrl);
        if (ret < 0)
                return ret;
        ret = rmi_read_block(rmi_dev, f11->fd.query_base_addr,
                             f11_query, sizeof(f11_query));
        if (ret < 0)
                return ret;

        pr_info("HI3620-TOUCH-FWSTATE: F01stat=%02x ctrl=%02x irqen=%02x F11ctrl=%02x mask=%02lx q=%*phN d=%04x (controls untouched)\n",
                f01_status, f01_ctrl[0], f01_ctrl[1], f11_ctrl,
                f11->irq_mask[0] & 0xff, (int)sizeof(f11_query), f11_query,
                f11->fd.data_base_addr);
        return 0;
}

static int mediapad_touch_trace_f11(struct rmi_device *rmi_dev)
{
        struct rmi_function *f11 = mediapad_find_function(rmi_dev, 0x11);
        u8 raw[MEDIAPAD_TOUCH_RAW_BYTES];
        int ret;

        if (!f11)
                return -ENODEV;

        ret = rmi_read_block(rmi_dev, f11->fd.data_base_addr,
                             raw, sizeof(raw));
        if (ret < 0)
                return ret;

        if (!mediapad_touch_last_raw_valid ||
            memcmp(raw, mediapad_touch_last_raw, sizeof(raw))) {
                pr_info("HI3620-TOUCH-DATA[%u]: d=%04x raw=%*phN\n",
                        mediapad_touch_ticks, f11->fd.data_base_addr,
                        (int)sizeof(raw), raw);
                memcpy(mediapad_touch_last_raw, raw, sizeof(raw));
                mediapad_touch_last_raw_valid = true;
        }

        return 0;
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

static void mediapad_touch_log_state(struct rmi_device *rmi_dev)
{
        struct rmi_function *f01 = mediapad_find_function(rmi_dev, 0x01);
        struct rmi_function *f11 = mediapad_find_function(rmi_dev, 0x11);
        u8 ctrl[2];
        u8 status;
        u8 f11ctrl;

        if (!f01 || !f11)
                return;
        if (rmi_read_block(rmi_dev, f01->fd.control_base_addr,
                           ctrl, sizeof(ctrl)) < 0)
                return;
        if (rmi_read(rmi_dev, f01->fd.data_base_addr, &status) < 0)
                return;
        if (rmi_read(rmi_dev, f11->fd.control_base_addr, &f11ctrl) < 0)
                return;

        pr_info("HI3620-TOUCH-STATE[%u]: F01stat=%02x ctrl=%02x irqen=%02x F11ctrl=%02x f11mask=%02lx\n",
                mediapad_touch_ticks, status, ctrl[0], ctrl[1], f11ctrl,
                f11->irq_mask[0] & 0xff);
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

                mediapad_touch_rmi = to_rmi_device(dev);
                pr_info("HI3620-TOUCH-POLL: attached to %s, F11 every %ums\n",
                        dev_name(dev), MEDIAPAD_TOUCH_POLL_MS);
        }

        if (!mediapad_touch_described) {
                ret = mediapad_touch_describe(mediapad_touch_rmi);
                if (!ret)
                        mediapad_touch_described = true;
                else if (ret != -EAGAIN)
                        pr_warn_ratelimited("HI3620-TOUCH-POLL: state read failed: %d\n",
                                            ret);
        }

        ret = mediapad_touch_trace_f11(mediapad_touch_rmi);
        if (ret && ret != -ENODEV)
                pr_warn_ratelimited("HI3620-TOUCH-POLL: F11 raw read failed: %d\n",
                                    ret);

        ret = mediapad_poll_f11(mediapad_touch_rmi);
        if (!ret && !mediapad_touch_announced) {
                mediapad_touch_announced = true;
                pr_info("HI3620-TOUCH-POLL: F11 polling fallback active\n");
        } else if (ret && ret != -EAGAIN && ret != -ENODEV) {
                pr_warn_ratelimited("HI3620-TOUCH-POLL: F11 attention failed: %d\n",
                                    ret);
        }

        if (!(mediapad_touch_ticks % MEDIAPAD_TOUCH_STATE_EVERY))
                mediapad_touch_log_state(mediapad_touch_rmi);
        mediapad_touch_ticks++;

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
        pr_info("HI3620-TOUCH-POLL: firmware-state-preserving F11 poller enabled\n");
        return 0;
}
late_initcall(mediapad_touch_poll_init);
