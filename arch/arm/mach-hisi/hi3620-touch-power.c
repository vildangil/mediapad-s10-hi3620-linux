// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD late touchscreen recovery helper.
 *
 * Stock K3V2 code powers Synaptics RMI4 from HI6421 LDO5 (ts-vbus, 1.8 V)
 * and LDO13 (ts-vdd), then pulses GPIO156 reset. During mainline bring-up the
 * GPIO direction is later clobbered and the controller is observed in reset.
 * Re-assert the two stock rails and perform one late reset pulse before the
 * diagnostic I2C probe. This is intentionally board-specific.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_PMUSPI_PHYS             0xfcc00000
#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_MAP_SIZE                0x1000

#define HI6421_LDO5_CTRL               (0x25 << 2)
#define HI6421_LDO13_CTRL              (0x2d << 2)
#define HI6421_LDO_ENA                 0x10
/* LDO5/13 voltage table index 1 is 1.8 V; preserve all unrelated bits. */
#define HI6421_LDO_VSEL_MASK           0x07
#define HI6421_LDO_1800MV              0x01

#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TOUCH_RESET_PIN                4
#define TOUCH_ATTN_PIN                 5

static int __init hi3620_mediapad_touch_power_late(void)
{
        void __iomem *pmu;
        void __iomem *gpio;
        u32 ldo5;
        u32 ldo13;
        u8 dir;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_MAP_SIZE);
        gpio = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        if (!pmu || !gpio) {
                pr_err("HI3620-TOUCH-PWR: failed to map PMU/GPIO19\n");
                if (gpio)
                        iounmap(gpio);
                if (pmu)
                        iounmap(pmu);
                return -ENOMEM;
        }

        ldo5 = readl(pmu + HI6421_LDO5_CTRL);
        ldo13 = readl(pmu + HI6421_LDO13_CTRL);

        ldo5 &= ~HI6421_LDO_VSEL_MASK;
        ldo5 |= HI6421_LDO_1800MV | HI6421_LDO_ENA;
        writel(ldo5, pmu + HI6421_LDO5_CTRL);

        /* Stock Synaptics ts-vdd is LDO13. Keep its bootloader voltage select
         * if already programmed; only force the enable bit here. */
        ldo13 |= HI6421_LDO_ENA;
        writel(ldo13, pmu + HI6421_LDO13_CTRL);
        msleep(10);

        dir = readb(gpio + PL061_GPIODIR);
        dir |= BIT(TOUCH_RESET_PIN);
        dir &= ~BIT(TOUCH_ATTN_PIN);
        writeb(dir, gpio + PL061_GPIODIR);

        writeb(0, gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(20);
        writeb(BIT(TOUCH_RESET_PIN), gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(120);

        pr_info("HI3620-TOUCH-PWR: ldo5=%08x ldo13=%08x dir=%02x rst=%u attn=%u\n",
                readl(pmu + HI6421_LDO5_CTRL),
                readl(pmu + HI6421_LDO13_CTRL),
                readb(gpio + PL061_GPIODIR),
                !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                !!(readb(gpio + PL061_DATA(TOUCH_ATTN_PIN)) & BIT(TOUCH_ATTN_PIN)));

        iounmap(gpio);
        iounmap(pmu);
        return 0;
}
late_initcall_sync(hi3620_mediapad_touch_power_late);
