// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD touchscreen bus isolation diagnostic.
 *
 * The GPIO bitbang tests showed both I2C2 lines stuck low even after the
 * Hi3620's internal pull-ups were enabled.  Run one controlled reset/power
 * isolation sequence before i2c-gpio probes so we can tell whether the
 * Synaptics controller itself is clamping SCL/SDA or the fault is elsewhere
 * in the pad/board path.
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

#define IOMG26_I2C2_SCL                0x068
#define IOMG27_I2C2_SDA                0x06c
#define IOMG_GPIO_FUNC                 0x1

#define IOCG_I2C2_SCL                  0x118
#define IOCG_I2C2_SDA                  0x11c
#define IOCG_PULL_MASK                 0x3
#define IOCG_PULLUP                    0x1

#define PMU_LDO5_CTRL                  (0x25 << 2)
#define PMU_LDO13_CTRL                 (0x2d << 2)
#define PMU_LDO_ENABLE                 0x10
#define PMU_LDO_VSEL_MASK              0x07
#define PMU_LDO_CTRL_MASK              (PMU_LDO_ENABLE | PMU_LDO_VSEL_MASK)
#define PMU_LDO5_1V8                   0x01
#define PMU_LDO13_2V85                 0x06

#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TOUCH_RESET_PIN                4       /* GPIO156 = GPIO19_4 */
#define TOUCH_ATTN_PIN                 5       /* GPIO157 = GPIO19_5 */
#define I2C2_SCL_PIN                   7       /* GPIO63  = GPIO7_7 */
#define I2C2_SDA_PIN                   0       /* GPIO64  = GPIO8_0 */

static unsigned int hi3620_line(void __iomem *gpio, unsigned int pin)
{
        return !!(readb(gpio + PL061_DATA(pin)) & BIT(pin));
}

static void hi3620_touch_bus_release(void __iomem *gpio7, void __iomem *gpio8)
{
        u8 dir7 = readb(gpio7 + PL061_GPIODIR);
        u8 dir8 = readb(gpio8 + PL061_GPIODIR);

        dir7 &= ~BIT(I2C2_SCL_PIN);
        dir8 &= ~BIT(I2C2_SDA_PIN);
        writeb(dir7, gpio7 + PL061_GPIODIR);
        writeb(dir8, gpio8 + PL061_GPIODIR);
        mb();
}

static void hi3620_touch_iso_log(const char *phase,
                                  void __iomem *iomux,
                                  void __iomem *iocfg,
                                  void __iomem *gpio7,
                                  void __iomem *gpio8,
                                  void __iomem *gpio19,
                                  void __iomem *pmu)
{
        pr_info("HI3620-TOUCH-ISO[%s]: mux=%08x/%08x pad=%08x/%08x dir7=%02x dir8=%02x dir19=%02x rst=%u attn=%u ldo5=%02x ldo13=%02x scl=%u sda=%u\n",
                phase,
                readl(iomux + IOMG26_I2C2_SCL),
                readl(iomux + IOMG27_I2C2_SDA),
                readl(iocfg + IOCG_I2C2_SCL),
                readl(iocfg + IOCG_I2C2_SDA),
                readb(gpio7 + PL061_GPIODIR),
                readb(gpio8 + PL061_GPIODIR),
                readb(gpio19 + PL061_GPIODIR),
                hi3620_line(gpio19, TOUCH_RESET_PIN),
                hi3620_line(gpio19, TOUCH_ATTN_PIN),
                readb(pmu + PMU_LDO5_CTRL),
                readb(pmu + PMU_LDO13_CTRL),
                hi3620_line(gpio7, I2C2_SCL_PIN),
                hi3620_line(gpio8, I2C2_SDA_PIN));
}

static int __init hi3620_mediapad_touch_bus_isolation(void)
{
        void __iomem *iomux = NULL;
        void __iomem *iocfg = NULL;
        void __iomem *gpio7 = NULL;
        void __iomem *gpio8 = NULL;
        void __iomem *gpio19 = NULL;
        void __iomem *pmu = NULL;
        u8 dir19;
        u8 ldo5_on;
        u8 ldo13_on;

        if (!of_machine_is_compatible("huawei,s10-101x") &&
            !of_machine_is_compatible("hisilicon,hi3620-hi4511"))
                return 0;

        iomux = ioremap(HI3620_IOMUX_PHYS, HI3620_MAP_SIZE);
        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio7 = ioremap(HI3620_GPIO7_PHYS, HI3620_MAP_SIZE);
        gpio8 = ioremap(HI3620_GPIO8_PHYS, HI3620_MAP_SIZE);
        gpio19 = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_MAP_SIZE);
        if (!iomux || !iocfg || !gpio7 || !gpio8 || !gpio19 || !pmu) {
                pr_err("HI3620-TOUCH-ISO: failed to map diagnostic registers\n");
                goto out;
        }

        /* Keep the bus on PL061 GPIO and use only weak pull-ups. */
        writel(IOMG_GPIO_FUNC, iomux + IOMG26_I2C2_SCL);
        writel(IOMG_GPIO_FUNC, iomux + IOMG27_I2C2_SDA);
        writel((readl(iocfg + IOCG_I2C2_SCL) & ~IOCG_PULL_MASK) |
               IOCG_PULLUP, iocfg + IOCG_I2C2_SCL);
        writel((readl(iocfg + IOCG_I2C2_SDA) & ~IOCG_PULL_MASK) |
               IOCG_PULLUP, iocfg + IOCG_I2C2_SDA);
        hi3620_touch_bus_release(gpio7, gpio8);

        /* RESET is a dedicated GPIO on Hi3620; ATTN stays input. */
        dir19 = readb(gpio19 + PL061_GPIODIR);
        dir19 |= BIT(TOUCH_RESET_PIN);
        dir19 &= ~BIT(TOUCH_ATTN_PIN);
        writeb(dir19, gpio19 + PL061_GPIODIR);
        mb();
        udelay(100);

        hi3620_touch_iso_log("powered-reset-high", iomux, iocfg,
                             gpio7, gpio8, gpio19, pmu);

        /*
         * First isolate reset only.  A correctly reset I2C slave must not
         * permanently clamp both open-drain bus wires low.
         */
        writeb(0, gpio19 + PL061_DATA(TOUCH_RESET_PIN));
        mb();
        msleep(20);
        hi3620_touch_bus_release(gpio7, gpio8);
        udelay(100);
        hi3620_touch_iso_log("reset-low", iomux, iocfg,
                             gpio7, gpio8, gpio19, pmu);

        /*
         * Then remove both touch rails while RESET remains asserted.  Power
         * down in the reverse of Huawei's enable order: VBUS/LDO5 first,
         * then VDD/LDO13.  The SoC internal pull-ups remain enabled, so this
         * distinguishes a powered device clamp from the SoC/board pad path.
         */
        ldo5_on = (readb(pmu + PMU_LDO5_CTRL) & ~PMU_LDO_CTRL_MASK) |
                  PMU_LDO_ENABLE | PMU_LDO5_1V8;
        ldo13_on = (readb(pmu + PMU_LDO13_CTRL) & ~PMU_LDO_CTRL_MASK) |
                   PMU_LDO_ENABLE | PMU_LDO13_2V85;

        writeb(ldo5_on & ~PMU_LDO_ENABLE, pmu + PMU_LDO5_CTRL);
        mb();
        udelay(100);
        writeb(ldo13_on & ~PMU_LDO_ENABLE, pmu + PMU_LDO13_CTRL);
        mb();
        msleep(20);
        hi3620_touch_bus_release(gpio7, gpio8);
        udelay(100);
        hi3620_touch_iso_log("rails-off-reset-low", iomux, iocfg,
                             gpio7, gpio8, gpio19, pmu);

        /* Repower exactly in Huawei order: VDD first, then VBUS. */
        writeb(ldo13_on, pmu + PMU_LDO13_CTRL);
        mb();
        udelay(100);
        writeb(ldo5_on, pmu + PMU_LDO5_CTRL);
        mb();
        msleep(5);
        hi3620_touch_bus_release(gpio7, gpio8);
        udelay(100);
        hi3620_touch_iso_log("repowered-reset-low", iomux, iocfg,
                             gpio7, gpio8, gpio19, pmu);

        /* Match stock reset and leave the device ready for i2c-gpio probe. */
        msleep(10);
        writeb(BIT(TOUCH_RESET_PIN),
               gpio19 + PL061_DATA(TOUCH_RESET_PIN));
        mb();
        msleep(100);
        hi3620_touch_bus_release(gpio7, gpio8);
        udelay(100);
        hi3620_touch_iso_log("repowered-reset-high", iomux, iocfg,
                             gpio7, gpio8, gpio19, pmu);

out:
        if (pmu)
                iounmap(pmu);
        if (gpio19)
                iounmap(gpio19);
        if (gpio8)
                iounmap(gpio8);
        if (gpio7)
                iounmap(gpio7);
        if (iocfg)
                iounmap(iocfg);
        if (iomux)
                iounmap(iomux);
        return 0;
}

/* postcore touch setup runs first; i2c-gpio's subsys_initcall runs after us. */
arch_initcall(hi3620_mediapad_touch_bus_isolation);
