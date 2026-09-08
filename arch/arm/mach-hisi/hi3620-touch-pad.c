// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD touchscreen pad/reset setup.
 *
 * The S10 schematic routes TP_SCL/TP_SDA to I2C2 and gives the panel a
 * dedicated 3.3 V rail plus a switched 1.8 V I/O rail controlled by GPIO61.
 * Do not reuse the generic K3V2/P6 LDO13 recipe here: on S10 that regulator
 * is not the documented TP 3.3 V supply.  Rail cycling is handled later by
 * the S10-specific isolation helper; this file only prepares pads/reset and
 * the GPIO-backed I2C transport.
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
#define HI3620_GPIO8_PHYS              0xfc80e000
#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_PMUSPI_PHYS             0xfcc00000
#define HI3620_MAP_SIZE                0x1000

#define IOCG_GPIO156                   0x00c
#define IOCG_GPIO157                   0x010
#define IOCG_I2C2_SCL                  0x118
#define IOCG_I2C2_SDA                  0x11c
#define IOCG_PULL_MASK                 0x3
#define IOCG_NOPULL                    0x0
#define IOCG_PULLUP                    0x1

#define IOMG26_I2C2_SCL                0x068
#define IOMG27_I2C2_SDA                0x06c
#define IOMG_GPIO_FUNC                 0x1

/* Read-only diagnostics: these are deliberately not modified here. */
#define PMU_LDO5_CTRL                  (0x25 << 2)
#define PMU_LDO13_CTRL                 (0x2d << 2)

#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TOUCH_RESET_PIN                4       /* GPIO156 = GPIO19_4 */
#define TOUCH_ATTN_PIN                 5       /* GPIO157 = GPIO19_5 */
#define I2C2_SCL_PIN                   7       /* GPIO63  = GPIO7_7 */
#define I2C2_SDA_PIN                   0       /* GPIO64  = GPIO8_0 */

static void hi3620_mediapad_touch_bitbang_prepare(void)
{
        void __iomem *iomux = NULL;
        void __iomem *iocfg = NULL;
        void __iomem *gpio7 = NULL;
        void __iomem *gpio8 = NULL;
        u32 scl_pad_old;
        u32 sda_pad_old;
        u8 dir7;
        u8 dir8;
        u8 scl;
        u8 sda;

        iomux = ioremap(HI3620_IOMUX_PHYS, HI3620_MAP_SIZE);
        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio7 = ioremap(HI3620_GPIO7_PHYS, HI3620_MAP_SIZE);
        gpio8 = ioremap(HI3620_GPIO8_PHYS, HI3620_MAP_SIZE);
        if (!iomux || !iocfg || !gpio7 || !gpio8) {
                pr_err("HI3620-TOUCH-BITBANG: failed to map IOMUX/IOCFG/GPIO\n");
                goto out;
        }

        /* The fitted S10 resistors route the panel to I2C2.  Keep the pads on
         * PL061 GPIO because the DesignWare I2C2 controller does not start
         * transfers yet on this kernel. */
        writel(IOMG_GPIO_FUNC, iomux + IOMG26_I2C2_SCL);
        writel(IOMG_GPIO_FUNC, iomux + IOMG27_I2C2_SDA);

        scl_pad_old = readl(iocfg + IOCG_I2C2_SCL);
        sda_pad_old = readl(iocfg + IOCG_I2C2_SDA);
        writel((scl_pad_old & ~IOCG_PULL_MASK) | IOCG_PULLUP,
               iocfg + IOCG_I2C2_SCL);
        writel((sda_pad_old & ~IOCG_PULL_MASK) | IOCG_PULLUP,
               iocfg + IOCG_I2C2_SDA);
        mb();

        /* Open-drain release: input/high-Z means high through the pull-ups. */
        dir7 = readb(gpio7 + PL061_GPIODIR) & ~BIT(I2C2_SCL_PIN);
        dir8 = readb(gpio8 + PL061_GPIODIR) & ~BIT(I2C2_SDA_PIN);
        writeb(dir7, gpio7 + PL061_GPIODIR);
        writeb(dir8, gpio8 + PL061_GPIODIR);
        mb();
        udelay(100);

        scl = !!(readb(gpio7 + PL061_DATA(I2C2_SCL_PIN)) & BIT(I2C2_SCL_PIN));
        sda = !!(readb(gpio8 + PL061_DATA(I2C2_SDA_PIN)) & BIT(I2C2_SDA_PIN));

        pr_info("HI3620-TOUCH-BITBANG-PULLUP: mux=%08x/%08x pad=%08x->%08x/%08x->%08x dir=%02x/%02x released scl=%u sda=%u\n",
                readl(iomux + IOMG26_I2C2_SCL),
                readl(iomux + IOMG27_I2C2_SDA),
                scl_pad_old, readl(iocfg + IOCG_I2C2_SCL),
                sda_pad_old, readl(iocfg + IOCG_I2C2_SDA),
                readb(gpio7 + PL061_GPIODIR),
                readb(gpio8 + PL061_GPIODIR), scl, sda);

out:
        if (gpio8)
                iounmap(gpio8);
        if (gpio7)
                iounmap(gpio7);
        if (iocfg)
                iounmap(iocfg);
        if (iomux)
                iounmap(iomux);
}

static int __init hi3620_mediapad_touch_pad_prepare(void)
{
        void __iomem *iocfg = NULL;
        void __iomem *gpio = NULL;
        void __iomem *pmu = NULL;
        u32 cfg156;
        u32 cfg157;
        u8 dir;
        u8 attn;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        pr_info("HI3620-TOUCH: S10 pad/reset setup (dedicated 3V3 + GPIO61 1V8 topology)\n");

        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_MAP_SIZE);
        if (!iocfg || !gpio || !pmu) {
                pr_err("HI3620-TOUCH: failed to map IOCG/GPIO19/PMUSPI\n");
                goto out;
        }

        /* S10 TP power is not LDO13.  Record the legacy PMIC values so the
         * ramoops proves that this path no longer modifies them. */
        pr_info("HI3620-TOUCH-S10-PWR: pad stage leaves PMIC untouched ldo5=%02x ldo13=%02x\n",
                readb(pmu + PMU_LDO5_CTRL), readb(pmu + PMU_LDO13_CTRL));

        cfg156 = readl(iocfg + IOCG_GPIO156);
        cfg157 = readl(iocfg + IOCG_GPIO157);
        cfg156 = (cfg156 & ~IOCG_PULL_MASK) | IOCG_NOPULL;
        cfg157 = (cfg157 & ~IOCG_PULL_MASK) | IOCG_PULLUP;
        writel(cfg156, iocfg + IOCG_GPIO156);
        writel(cfg157, iocfg + IOCG_GPIO157);

        dir = readb(gpio + PL061_GPIODIR);
        dir |= BIT(TOUCH_RESET_PIN);
        dir &= ~BIT(TOUCH_ATTN_PIN);
        writeb(dir, gpio + PL061_GPIODIR);

        /* Keep reset high here; the S10 power helper performs the one
         * deterministic reset/power-switch cycle later in init. */
        writeb(BIT(TOUCH_RESET_PIN), gpio + PL061_DATA(TOUCH_RESET_PIN));
        mb();
        attn = readb(gpio + PL061_DATA(TOUCH_ATTN_PIN));

        pr_info("HI3620-TOUCH-PAD: dir=%02x rst=%u attn=%u pull=%08x/%08x\n",
                readb(gpio + PL061_GPIODIR),
                !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                !!(attn & BIT(TOUCH_ATTN_PIN)),
                readl(iocfg + IOCG_GPIO156), readl(iocfg + IOCG_GPIO157));

        hi3620_mediapad_touch_bitbang_prepare();

out:
        if (pmu)
                iounmap(pmu);
        if (gpio)
                iounmap(gpio);
        if (iocfg)
                iounmap(iocfg);
        return 0;
}

postcore_initcall(hi3620_mediapad_touch_pad_prepare);
