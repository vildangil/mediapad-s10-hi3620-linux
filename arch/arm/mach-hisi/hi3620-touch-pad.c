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

/*
 * HI6421's in-tree regmap is reg_stride=4, val_bits=8.  The PMIC control
 * registers therefore live four bytes apart but must be accessed as bytes;
 * 32-bit writel() here can touch neighbouring PMIC registers.
 */
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
        u8 ldo5_old;
        u8 ldo13_old;
        u8 ldo5_new;
        u8 ldo13_new;
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

        /* Stock RMI4 rails: LDO5/ts-vbus=1.8 V, LDO13/ts-vdd=2.85 V. */
        ldo5_old = readb(pmu + PMU_LDO5_CTRL);
        ldo13_old = readb(pmu + PMU_LDO13_CTRL);
        ldo5_new = (ldo5_old & ~PMU_LDO_CTRL_MASK) |
                   PMU_LDO_ENABLE | PMU_LDO5_1V8;
        ldo13_new = (ldo13_old & ~PMU_LDO_CTRL_MASK) |
                    PMU_LDO_ENABLE | PMU_LDO13_2V85;
        writeb(ldo5_new, pmu + PMU_LDO5_CTRL);
        writeb(ldo13_new, pmu + PMU_LDO13_CTRL);
        mb();
        msleep(10);

        pr_info("HI3620-TOUCH-PWR: ldo5=%02x->%02x ldo13=%02x->%02x\n",
                ldo5_old, readb(pmu + PMU_LDO5_CTRL),
                ldo13_old, readb(pmu + PMU_LDO13_CTRL));

        cfg156 = readl(iocfg + IOCG_GPIO156);
        cfg157 = readl(iocfg + IOCG_GPIO157);

        /* Huawei touchscreen NORMAL state: reset=no-pull, ATTN=pull-up. */
        cfg156 = (cfg156 & ~IOCG_PULL_MASK) | IOCG_NOPULL;
        cfg157 = (cfg157 & ~IOCG_PULL_MASK) | IOCG_PULLUP;
        writel(cfg156, iocfg + IOCG_GPIO156);
        writel(cfg157, iocfg + IOCG_GPIO157);

        /* Reset is output; active-low ATTN is input. */
        dir = readb(gpio + PL061_GPIODIR);
        dir |= BIT(TOUCH_RESET_PIN);
        dir &= ~BIT(TOUCH_ATTN_PIN);
        writeb(dir, gpio + PL061_GPIODIR);

        /* Match the vendor 100 ms reset/startup delay conservatively. */
        writeb(0, gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(100);
        writeb(BIT(TOUCH_RESET_PIN), gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(100);

        attn = readb(gpio + PL061_DATA(TOUCH_ATTN_PIN));
        pr_info("HI3620-TOUCH-PAD: dir=%02x rst=%u attn=%u pull=%08x/%08x\n",
                readb(gpio + PL061_GPIODIR),
                !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                !!(attn & BIT(TOUCH_ATTN_PIN)),
                readl(iocfg + IOCG_GPIO156), readl(iocfg + IOCG_GPIO157));

        iounmap(pmu);
        iounmap(gpio);
        iounmap(iocfg);
        return 0;
}

/* Must run before the DesignWare I2C platform driver's subsys_initcall. */
postcore_initcall(hi3620_mediapad_touch_pad_prepare);
