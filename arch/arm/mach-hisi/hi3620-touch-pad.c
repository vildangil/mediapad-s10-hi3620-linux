// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD touchscreen pad/reset setup.
 *
 * The vendor K3V2 iomux tables put GPIO156 (reset) and GPIO157 (RMI ATTN)
 * in the touchscreen block. In NORMAL mode GPIO156 has no pull, while
 * GPIO157 has an internal pull-up. The stock board code also explicitly
 * makes ATTN an input and pulses reset before registering the RMI device.
 *
 * GPIO156/157 have no IOMG selector, but their IOCG registers are at
 * 0xfc80380c/0xfc803810. Bits 1:0 are pull configuration:
 *   0 = no pull, 1 = pull-up, 2 = pull-down.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_IOCFG_PHYS              0xfc803800
#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_MAP_SIZE                0x1000

#define IOCG_GPIO156                   0x00c
#define IOCG_GPIO157                   0x010
#define IOCG_PULL_MASK                 0x3
#define IOCG_NOPULL                    0x0
#define IOCG_PULLUP                    0x1

#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TOUCH_RESET_PIN                4
#define TOUCH_ATTN_PIN                 5

static int __init hi3620_mediapad_touch_pad_prepare(void)
{
        void __iomem *iocfg;
        void __iomem *gpio;
        u32 cfg156;
        u32 cfg157;
        u8 dir;
        u8 attn;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        if (!iocfg || !gpio) {
                pr_err("HI3620-TOUCH-PAD: failed to map IOCG/GPIO19\n");
                if (gpio)
                        iounmap(gpio);
                if (iocfg)
                        iounmap(iocfg);
                return -ENOMEM;
        }

        cfg156 = readl(iocfg + IOCG_GPIO156);
        cfg157 = readl(iocfg + IOCG_GPIO157);
        dir = readb(gpio + PL061_GPIODIR);

        pr_info("HI3620-TOUCH-PAD: pre iocg156=%08x iocg157=%08x dir=%02x rst=%u attn=%u\n",
                cfg156, cfg157, dir,
                !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                !!(readb(gpio + PL061_DATA(TOUCH_ATTN_PIN)) & BIT(TOUCH_ATTN_PIN)));

        /* Match Huawei ts_es NORMAL: reset=Nopull, ATTN=Pullup. */
        cfg156 &= ~IOCG_PULL_MASK;
        cfg156 |= IOCG_NOPULL;
        writel(cfg156, iocfg + IOCG_GPIO156);

        cfg157 &= ~IOCG_PULL_MASK;
        cfg157 |= IOCG_PULLUP;
        writel(cfg157, iocfg + IOCG_GPIO157);

        /* Match stock gpio setup: reset is output, ATTN must be input. */
        dir |= BIT(TOUCH_RESET_PIN);
        dir &= ~BIT(TOUCH_ATTN_PIN);
        writeb(dir, gpio + PL061_GPIODIR);

        /* Use a conservative 100 ms low pulse, then allow 100 ms startup. */
        writeb(0, gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(100);
        writeb(BIT(TOUCH_RESET_PIN), gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(100);

        attn = readb(gpio + PL061_DATA(TOUCH_ATTN_PIN));
        pr_info("HI3620-TOUCH-PAD: post iocg156=%08x iocg157=%08x dir=%02x rst=%u attn=%u\n",
                readl(iocfg + IOCG_GPIO156), readl(iocfg + IOCG_GPIO157),
                readb(gpio + PL061_GPIODIR),
                !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                !!(attn & BIT(TOUCH_ATTN_PIN)));

        iounmap(gpio);
        iounmap(iocfg);
        return 0;
}
subsys_initcall(hi3620_mediapad_touch_pad_prepare);
