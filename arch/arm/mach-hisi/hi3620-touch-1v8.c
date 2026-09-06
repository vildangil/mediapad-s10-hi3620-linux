// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD touchscreen 1.8 V load-switch enable.
 *
 * Huawei's maintenance manual names GPIO61_TP1V8_EN as the touchscreen
 * 1.8 V power switch.  The vendor Synaptics driver also explicitly requests
 * GPIO_7_5 (GPIO61) and drives it high before resetting/probing the panel.
 * Without this enable, both TP_SCL/TP_SDA remain low even with the Hi3620
 * internal pull-ups enabled, and i2c-gpio cannot generate a valid idle bus.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_IOMUX_PHYS              0xfc803000
#define HI3620_IOCFG_PHYS              0xfc803800
#define HI3620_GPIO7_PHYS              0xfc80d000
#define HI3620_MAP_SIZE                0x1000

#define IOMG24_TP1V8_EN                0x060
#define IOMG_GPIO_FUNC                 0x1
#define IOCG_TP1V8_EN                  0x110
#define IOCG_PULL_MASK                 0x3
#define IOCG_NOPULL                    0x0

#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TP1V8_EN_PIN                   5       /* GPIO61 = GPIO7_5 */

static int __init hi3620_mediapad_touch_1v8_enable(void)
{
        void __iomem *iomux = NULL;
        void __iomem *iocfg = NULL;
        void __iomem *gpio7 = NULL;
        u32 mux_old;
        u32 pad_old;
        u8 dir_old;
        u8 dir_new;
        unsigned int level;

        if (!of_machine_is_compatible("huawei,s10-101x") &&
            !of_machine_is_compatible("hisilicon,hi3620-hi4511"))
                return 0;

        iomux = ioremap(HI3620_IOMUX_PHYS, HI3620_MAP_SIZE);
        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio7 = ioremap(HI3620_GPIO7_PHYS, HI3620_MAP_SIZE);
        if (!iomux || !iocfg || !gpio7) {
                pr_err("HI3620-TOUCH-1V8: failed to map GPIO61 controls\n");
                goto out;
        }

        mux_old = readl(iomux + IOMG24_TP1V8_EN);
        pad_old = readl(iocfg + IOCG_TP1V8_EN);
        dir_old = readb(gpio7 + PL061_GPIODIR);

        /* GPIO61_TP1V8_EN: GPIO mode, no pull, output high. */
        writel(IOMG_GPIO_FUNC, iomux + IOMG24_TP1V8_EN);
        writel((pad_old & ~IOCG_PULL_MASK) | IOCG_NOPULL,
               iocfg + IOCG_TP1V8_EN);

        dir_new = dir_old | BIT(TP1V8_EN_PIN);
        writeb(dir_new, gpio7 + PL061_GPIODIR);
        writeb(BIT(TP1V8_EN_PIN), gpio7 + PL061_DATA(TP1V8_EN_PIN));
        mb();

        /* Vendor code enables GPIO61 before touchscreen reset/probe. */
        msleep(5);
        level = !!(readb(gpio7 + PL061_DATA(TP1V8_EN_PIN)) &
                   BIT(TP1V8_EN_PIN));

        pr_info("HI3620-TOUCH-1V8: gpio61 mux=%08x->%08x pad=%08x->%08x dir=%02x->%02x level=%u\n",
                mux_old, readl(iomux + IOMG24_TP1V8_EN),
                pad_old, readl(iocfg + IOCG_TP1V8_EN),
                dir_old, readb(gpio7 + PL061_GPIODIR), level);

out:
        if (gpio7)
                iounmap(gpio7);
        if (iocfg)
                iounmap(iocfg);
        if (iomux)
                iounmap(iomux);
        return 0;
}

/* Run after the existing postcore touch setup and before arch bus diagnostics. */
postcore_initcall(hi3620_mediapad_touch_1v8_enable);
