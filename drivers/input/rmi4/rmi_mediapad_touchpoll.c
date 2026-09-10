// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD RMI4 F11 polling fallback.
 *
 * TM2263-002 needs the Huawei-style F01 software reset after the board-level
 * power/reset sequence, followed by the normal RMI reset/config callbacks.
 * Keep the F11 polling fallback that makes touch reliable on this tablet, but
 * avoid periodic/raw tracing now that the controller is known to work: pstore
 * space is more useful for bring-up failures in other subsystems.
 *
 * This never touches F34 flash or writes F54 factory/calibration controls.
 */

#include <linux/delay.h>
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

#define MEDIAPAD_TOUCH_POLL_MS          16
#define MEDIAPAD_TOUCH_RETRY_MS         100
#define MEDIAPAD_TOUCH_RAW_BYTES        16
#define MEDIAPAD_TOUCH_RESET_WAIT_MS    200

#define RMI_F01_DEVICE_RESET            0x01
#define RMI_F01_SLEEP_MODE_MASK         0x03
#define RMI_F01_CONFIGURED_BIT          BIT(7)

static struct delayed_work mediapad_touch_poll_work;
static struct rmi_device *mediapad_touch_rmi;
static bool mediapad_touch_announced;
static bool mediapad_touch_described;
static bool mediapad_touch_reset_done;

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

static int mediapad_reconfigure_bound_functions(struct rmi_device *rmi_dev)
{
        struct rmi_driver_data *drvdata = dev_get_drvdata(&rmi_dev->dev);
        struct rmi_function_handler *handler;
        struct rmi_function *fn;
        int ret;

        if (!drvdata)
                return -EAGAIN;

        /* Match the RMI core's normal reset-handler ordering: reset every
         * bound function first, then restore every volatile configuration. */
        list_for_each_entry(fn, &drvdata->function_list, node) {
                if (!fn->dev.driver)
                        continue;
                handler = to_rmi_function_handler(fn->dev.driver);
                if (!handler->reset)
                        continue;
                ret = handler->reset(fn);
                if (ret < 0)
                        return ret;
        }

        list_for_each_entry(fn, &drvdata->function_list, node) {
                if (!fn->dev.driver)
                        continue;
                handler = to_rmi_function_handler(fn->dev.driver);
                if (!handler->config)
                        continue;
                ret = handler->config(fn);
                if (ret < 0)
                        return ret;
        }

        return 0;
}

static int mediapad_touch_vendor_reset(struct rmi_device *rmi_dev)
{
        struct rmi_function *f01 = mediapad_find_function(rmi_dev, 0x01);
        struct rmi_function *f11 = mediapad_find_function(rmi_dev, 0x11);
        u8 ctrl[2] = { 0 };
        u8 status = 0;
        u8 f11ctrl = 0;
        u8 raw[MEDIAPAD_TOUCH_RAW_BYTES] = { 0 };
        u8 newval;
        int ret;

        if (!f01 || !f11 || !f01->dev.driver || !f11->dev.driver)
                return -EAGAIN;

        ret = rmi_read_block(rmi_dev, f01->fd.control_base_addr,
                             ctrl, sizeof(ctrl));
        if (ret < 0)
                return ret;
        ret = rmi_read(rmi_dev, f01->fd.data_base_addr, &status);
        if (ret < 0)
                return ret;
        ret = rmi_read(rmi_dev, f11->fd.control_base_addr, &f11ctrl);
        if (ret < 0)
                return ret;

        pr_info("HI3620-TOUCH-SWRESET[before]: F01stat=%02x ctrl=%02x irqen=%02x F11ctrl=%02x cmd=%04x\n",
                status, ctrl[0], ctrl[1], f11ctrl,
                f01->fd.command_base_addr);

        /* Huawei K3V2 vendor driver issues exactly this F01 software reset
         * after powering/waking the controller. It is a volatile device reset,
         * not an F34 firmware/flash operation. */
        ret = rmi_write(rmi_dev, f01->fd.command_base_addr,
                        RMI_F01_DEVICE_RESET);
        if (ret < 0)
                return ret;

        msleep(MEDIAPAD_TOUCH_RESET_WAIT_MS);

        ret = mediapad_reconfigure_bound_functions(rmi_dev);
        if (ret < 0)
                return ret;

        /* Be explicit about the two bits the old Huawei driver relies on:
         * normal power mode + host-configured, and F11 IRQ enabled. */
        ret = rmi_read(rmi_dev, f01->fd.control_base_addr, &newval);
        if (ret < 0)
                return ret;
        newval &= ~RMI_F01_SLEEP_MODE_MASK;
        newval |= RMI_F01_CONFIGURED_BIT;
        ret = rmi_write(rmi_dev, f01->fd.control_base_addr, newval);
        if (ret < 0)
                return ret;

        ret = rmi_read(rmi_dev, f01->fd.control_base_addr + 1, &newval);
        if (ret < 0)
                return ret;
        newval |= (u8)f11->irq_mask[0];
        ret = rmi_write(rmi_dev, f01->fd.control_base_addr + 1, newval);
        if (ret < 0)
                return ret;

        ret = rmi_process_interrupt_requests(rmi_dev);
        if (ret < 0)
                return ret;

        ret = rmi_read_block(rmi_dev, f01->fd.control_base_addr,
                             ctrl, sizeof(ctrl));
        if (ret < 0)
                return ret;
        ret = rmi_read(rmi_dev, f01->fd.data_base_addr, &status);
        if (ret < 0)
                return ret;
        ret = rmi_read(rmi_dev, f11->fd.control_base_addr, &f11ctrl);
        if (ret < 0)
                return ret;
        ret = rmi_read_block(rmi_dev, f11->fd.data_base_addr,
                             raw, sizeof(raw));
        if (ret < 0)
                return ret;

        pr_info("HI3620-TOUCH-SWRESET[after]: F01stat=%02x ctrl=%02x irqen=%02x F11ctrl=%02x raw=%*phN\n",
                status, ctrl[0], ctrl[1], f11ctrl,
                (int)sizeof(raw), raw);

        mediapad_touch_described = false;
        return 0;
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

        pr_info("HI3620-TOUCH-FWSTATE: F01stat=%02x ctrl=%02x irqen=%02x F11ctrl=%02x mask=%02lx q=%*phN d=%04x\n",
                f01_status, f01_ctrl[0], f01_ctrl[1], f11_ctrl,
                f11->irq_mask[0] & 0xff, (int)sizeof(f11_query), f11_query,
                f11->fd.data_base_addr);
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

        if (!mediapad_touch_reset_done) {
                ret = mediapad_touch_vendor_reset(mediapad_touch_rmi);
                if (!ret) {
                        mediapad_touch_reset_done = true;
                        pr_info("HI3620-TOUCH-SWRESET: Huawei-style one-shot reset/config completed\n");
                } else if (ret != -EAGAIN) {
                        pr_warn_ratelimited("HI3620-TOUCH-SWRESET: failed: %d\n",
                                            ret);
                }
        }

        if (!mediapad_touch_described) {
                ret = mediapad_touch_describe(mediapad_touch_rmi);
                if (!ret)
                        mediapad_touch_described = true;
                else if (ret != -EAGAIN)
                        pr_warn_ratelimited("HI3620-TOUCH-POLL: state read failed: %d\n",
                                            ret);
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
        pr_info("HI3620-TOUCH-POLL: Huawei-reset F11 poller enabled\n");
        return 0;
}
late_initcall(mediapad_touch_poll_init);
