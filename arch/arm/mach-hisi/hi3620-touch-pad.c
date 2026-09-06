// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD touchscreen power/pad/reset setup.
 *
 * The stock K3V2 board powers the I2C2 Synaptics device from HI6421 LDO5
 * ("ts-vbus", 1.8 V) and LDO13 ("ts-vdd", 2.85 V), then configures
 * GPIO156 as reset and GPIO157 as the active-low RMI attention input.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_IOCFG_PHYS              0xfc803800
#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_PMUSPI_PHYS             0xfcc00000
#define HI3620_MAP_SIZE                0x1000

#define IOCG_GPIO156                   0x00c
#define IOCG_GPIO157                   0x010
#define IOCG_PULL_MASK                 0x3
#define IOCG_NOPULL                    0x0
#define IOCG_PULLUP                    0x1

/* HI6421 regulator control registers from the vendor PMIC driver. */
#define PMU_LDO5_CTRL                  (0x25 << 2)
#define PMU_LDO13_CTRL                 (0x2d << 2)
#define PMU_LDO_ENABLE                 0x10
#define PMU_LDO_VSEL_MASK              0x07
#define PMU_LDO_CTRL_MASK              (PMU_LDO_ENABLE | PMU_LDO_VSEL_MASK)
#define PMU_LDO5_1V8                   0x01
#define PMU_LDO13_2V85                 0x06

#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TOUCH_RESET_PIN                4
#define TOUCH_ATTN_PIN                 5

static int __init hi3620_mediapad_touch_pad_prepare(void)
{
        void __iomem *iocfg;
        void __iomem *gpio;
        void __iomem *pmu;
        u32 cfg156;
        u32 cfg157;
        u32 ldo5_old;
        u32 ldo13_old;
        u32 ldo5_new;
        u32 ldo13_new;
        u8 dir;
        u8 attn;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_MAP_SIZE);
        if (!iocfg || !gpio || !pmu) {
                pr_err("HI3620-TOUCH: failed to map IOCG/GPIO19/PMUSPI\n");
                if (pmu)
                        iounmap(pmu);
                if (gpio)
                        iounmap(gpio);
                if (iocfg)
                        iounmap(iocfg);
                return -ENOMEM;
        }

        /*
         * Match the stock touchscreen supplies before releasing reset.
         * LDO5 table index 1 is 1.8 V; LDO13 table index 6 is 2.85 V.
         * The vendor touchscreen helper writes 0x16 to LDO13 directly.
         * Preserve unrelated PMIC bits and only replace enable/vsel fields.
         */
        ldo5_old = readl(pmu + PMU_LDO5_CTRL);
        ldo13_old = readl(pmu + PMU_LDO13_CTRL);
        ldo5_new = (ldo5_old & ~PMU_LDO_CTRL_MASK) |
                   PMU_LDO_ENABLE | PMU_LDO5_1V8;
        ldo13_new = (ldo13_old & ~PMU_LDO_CTRL_MASK) |
                    PMU_LDO_ENABLE | PMU_LDO13_2V85;
        writel(ldo5_new, pmu + PMU_LDO5_CTRL);
        writel(ldo13_new, pmu + PMU_LDO13_CTRL);
        msleep(5);

        pr_info("HI3620-TOUCH-PWR: ldo5=%08x->%08x ldo13=%08x->%08x\n",
                ldo5_old, readl(pmu + PMU_LDO5_CTRL),
                ldo13_old, readl(pmu + PMU_LDO13_CTRL));

        cfg156 = readl(iocfg + IOCG_GPIO156);
        cfg157 = readl(iocfg + IOCG_GPIO157);
        dir = readb(gpio + PL061_GPIODIR);

        /* Match Huawei ts_es NORMAL: reset=Nopull, ATTN=Pullup. */
        cfg156 &= ~IOCG_PULL_MASK;
        cfg156 |= IOCG_NOPULL;
        writel(cfg156, iocfg + IOCG_GPIO156);

        cfg157 &= ~IOCG_PULL_MASK;
        cfg157 |= IOCG_PULLUP;
        writel(cfg157, iocfg + IOCG_GPIO157);

        /* Stock GPIO setup: reset is output, ATTN is input. */
        dir |= BIT(TOUCH_RESET_PIN);
        dir &= ~BIT(TOUCH_ATTN_PIN);
        writeb(dir, gpio + PL061_GPIODIR);

        /* Power is stable now: perform the stock-style reset pulse. */
        writeb(0, gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(100);
        writeb(BIT(TOUCH_RESET_PIN), gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(100);

        attn = readb(gpio + PL061_DATA(TOUCH_ATTN_PIN));
        pr_info("HI3620-TOUCH-PAD: iocg156=%08x iocg157=%08x dir=%02x rst=%u attn=%u\n",
                readl(iocfg + IOCG_GPIO156), readl(iocfg + IOCG_GPIO157),
                readb(gpio + PL061_GPIODIR),
                !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                !!(attn & BIT(TOUCH_ATTN_PIN)));

        iounmap(pmu);
        iounmap(gpio);
        iounmap(iocfg);
        return 0;
}
subsys_initcall(hi3620_mediapad_touch_pad_prepare);
