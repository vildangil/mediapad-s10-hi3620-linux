// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD touchscreen power/reset setup.
 *
 * Huawei's K3V2 Synaptics driver uses two HI6421 supplies for the RMI4
 * controller: LDO13 (ts-vdd) and LDO5 (ts-vbus), then enables GPIO61 and
 * performs a 20 ms active-low hardware reset before reading the PDT.
 *
 * Keep the bootloader-selected LDO13 voltage intact: only set its enable bit.
 * LDO5 is the documented 1.8 V source behind GPIO61_TP1V8_EN, so force its
 * VSEL to 1.8 V and enable it.  Do not power-cycle either PMIC rail here.
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

#define IOMG24_TP1V8_EN                0x060
#define IOMG26_I2C2_SCL                0x068
#define IOMG27_I2C2_SDA                0x06c
#define IOMG_GPIO_FUNC                 0x1

#define IOCG_TP1V8_EN                  0x110
#define IOCG_I2C2_SCL                  0x118
#define IOCG_I2C2_SDA                  0x11c
#define IOCG_PULL_MASK                 0x3
#define IOCG_NOPULL                    0x0
#define IOCG_PULLUP                    0x1

#define PMU_LDO5_CTRL                  (0x25 << 2)
#define PMU_LDO13_CTRL                 (0x2d << 2)
#define PMU_LDO_ENABLE                 0x10
#define PMU_LDO_VSEL_MASK              0x07
#define PMU_LDO_CTRL_MASK              (PMU_LDO_ENABLE | PMU_LDO_VSEL_MASK)
#define PMU_LDO5_1V8                   0x01

#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TP1V8_EN_PIN                   5       /* GPIO61  = GPIO7_5 */
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

static void hi3620_touch_1v8_set(void __iomem *iomux,
                                 void __iomem *iocfg,
                                 void __iomem *gpio7,
                                 bool on)
{
        u8 dir7;

        writel(IOMG_GPIO_FUNC, iomux + IOMG24_TP1V8_EN);
        writel((readl(iocfg + IOCG_TP1V8_EN) & ~IOCG_PULL_MASK) |
               IOCG_NOPULL, iocfg + IOCG_TP1V8_EN);
        dir7 = readb(gpio7 + PL061_GPIODIR) | BIT(TP1V8_EN_PIN);
        writeb(dir7, gpio7 + PL061_GPIODIR);
        writeb(on ? BIT(TP1V8_EN_PIN) : 0,
               gpio7 + PL061_DATA(TP1V8_EN_PIN));
        mb();
}

static void hi3620_touch_power_log(const char *phase,
                                   void __iomem *iomux,
                                   void __iomem *iocfg,
                                   void __iomem *gpio7,
                                   void __iomem *gpio8,
                                   void __iomem *gpio19,
                                   void __iomem *pmu)
{
        pr_info("HI3620-TOUCH-VENDOR-PWR[%s]: tp1v8=%u rst=%u attn=%u ldo5=%02x ldo13=%02x scl=%u sda=%u mux=%08x/%08x pad=%08x/%08x\n",
                phase,
                hi3620_line(gpio7, TP1V8_EN_PIN),
                hi3620_line(gpio19, TOUCH_RESET_PIN),
                hi3620_line(gpio19, TOUCH_ATTN_PIN),
                readb(pmu + PMU_LDO5_CTRL),
                readb(pmu + PMU_LDO13_CTRL),
                hi3620_line(gpio7, I2C2_SCL_PIN),
                hi3620_line(gpio8, I2C2_SDA_PIN),
                readl(iomux + IOMG26_I2C2_SCL),
                readl(iomux + IOMG27_I2C2_SDA),
                readl(iocfg + IOCG_I2C2_SCL),
                readl(iocfg + IOCG_I2C2_SDA));
}

static int __init hi3620_mediapad_touch_vendor_power(void)
{
        void __iomem *iomux = NULL;
        void __iomem *iocfg = NULL;
        void __iomem *gpio7 = NULL;
        void __iomem *gpio8 = NULL;
        void __iomem *gpio19 = NULL;
        void __iomem *pmu = NULL;
        u8 dir19;
        u8 ldo5_old, ldo5_on;
        u8 ldo13_old, ldo13_on;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        iomux = ioremap(HI3620_IOMUX_PHYS, HI3620_MAP_SIZE);
        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio7 = ioremap(HI3620_GPIO7_PHYS, HI3620_MAP_SIZE);
        gpio8 = ioremap(HI3620_GPIO8_PHYS, HI3620_MAP_SIZE);
        gpio19 = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_MAP_SIZE);
        if (!iomux || !iocfg || !gpio7 || !gpio8 || !gpio19 || !pmu) {
                pr_err("HI3620-TOUCH-VENDOR-PWR: failed to map controls\n");
                goto out;
        }

        /* Keep the fitted I2C2 pads in GPIO/open-drain mode for i2c-gpio. */
        writel(IOMG_GPIO_FUNC, iomux + IOMG26_I2C2_SCL);
        writel(IOMG_GPIO_FUNC, iomux + IOMG27_I2C2_SDA);
        writel((readl(iocfg + IOCG_I2C2_SCL) & ~IOCG_PULL_MASK) |
               IOCG_PULLUP, iocfg + IOCG_I2C2_SCL);
        writel((readl(iocfg + IOCG_I2C2_SDA) & ~IOCG_PULL_MASK) |
               IOCG_PULLUP, iocfg + IOCG_I2C2_SDA);
        hi3620_touch_bus_release(gpio7, gpio8);

        dir19 = readb(gpio19 + PL061_GPIODIR);
        dir19 |= BIT(TOUCH_RESET_PIN);
        dir19 &= ~BIT(TOUCH_ATTN_PIN);
        writeb(dir19, gpio19 + PL061_GPIODIR);

        ldo13_old = readb(pmu + PMU_LDO13_CTRL);
        ldo5_old = readb(pmu + PMU_LDO5_CTRL);

        /* Huawei K3V2: ts-vdd (LDO13) first.  Preserve VSEL and all unrelated
         * bits, setting only LDO_ENA (0x10).  On this tablet the bootloader
         * leaves VSEL=6, i.e. 2.85 V. */
        ldo13_on = ldo13_old | PMU_LDO_ENABLE;
        writeb(ldo13_on, pmu + PMU_LDO13_CTRL);
        mb();
        udelay(250);

        /* Then ts-vbus (LDO5) at 1.8 V. */
        ldo5_on = (ldo5_old & ~PMU_LDO_CTRL_MASK) |
                  PMU_LDO_ENABLE | PMU_LDO5_1V8;
        writeb(ldo5_on, pmu + PMU_LDO5_CTRL);
        mb();
        udelay(250);

        /* MediaPad-specific load switch on the 1.8 V I/O rail. */
        hi3620_touch_1v8_set(iomux, iocfg, gpio7, true);
        hi3620_touch_bus_release(gpio7, gpio8);
        msleep(5);
        hi3620_touch_power_log("rails-on", iomux, iocfg,
                               gpio7, gpio8, gpio19, pmu);

        /* Huawei vendor driver: RESET low for 20 ms, then high. */
        writeb(0, gpio19 + PL061_DATA(TOUCH_RESET_PIN));
        mb();
        msleep(20);
        hi3620_touch_power_log("reset-low", iomux, iocfg,
                               gpio7, gpio8, gpio19, pmu);

        writeb(BIT(TOUCH_RESET_PIN),
               gpio19 + PL061_DATA(TOUCH_RESET_PIN));
        mb();
        msleep(2);
        hi3620_touch_bus_release(gpio7, gpio8);
        hi3620_touch_power_log("ready", iomux, iocfg,
                               gpio7, gpio8, gpio19, pmu);

        pr_info("HI3620-TOUCH-VENDOR-PWR: LDO13 %02x->%02x, LDO5 %02x->%02x, reset 20ms\n",
                ldo13_old, readb(pmu + PMU_LDO13_CTRL),
                ldo5_old, readb(pmu + PMU_LDO5_CTRL));

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

/* After pad preparation and before the i2c-gpio transport probes RMI4. */
arch_initcall(hi3620_mediapad_touch_vendor_power);
