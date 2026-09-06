// SPDX-License-Identifier: GPL-2.0
/* One-shot late touchscreen state log for MediaPad 10 FHD bring-up. */

#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_MAP_SIZE                0x1000
#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TOUCH_RESET_PIN                4
#define TOUCH_ATTN_PIN                 5

static int __init hi3620_touch_late_log(void)
{
        void __iomem *gpio;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        gpio = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        if (!gpio)
                return -ENOMEM;

        pr_info("HI3620-TOUCH-FINAL: dir=%02x rst=%u attn=%u\n",
                readb(gpio + PL061_GPIODIR),
                !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                !!(readb(gpio + PL061_DATA(TOUCH_ATTN_PIN)) & BIT(TOUCH_ATTN_PIN)));

        iounmap(gpio);
        return 0;
}
late_initcall_sync(hi3620_touch_late_log);
