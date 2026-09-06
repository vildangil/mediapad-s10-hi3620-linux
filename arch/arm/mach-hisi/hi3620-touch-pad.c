// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD touchscreen power/pad/reset setup.
 *
 * The stock K3V2 board powers the I2C2 Synaptics device from HI6421 LDO5
 * ("ts-vbus", 1.8 V) and LDO13 ("ts-vdd", 2.85 V), then configures
 * GPIO156 as reset and GPIO157 as the active-low RMI attention input.
 *
 * During bring-up the Hi3620 DesignWare I2C2 block accepts commands into its
 * TX FIFO but never starts a transfer.  For the current diagnostic boot we
 * leave the physical I2C2 pins in GPIO mode so the generic i2c-gpio adapter
 * can bitbang the exact same SCL/SDA wires independently of DesignWare.
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
#define IOCG_PULL_MASK                 0x3
#define IOCG_NOPULL                    0x0
#define IOCG_PULLUP                    0x1

/* I2C2 mux from Huawei block_i2c2: FUNC0=I2C2, FUNC1=GPIO/idle. */
#define IOMG26_I2C2_SCL                0x068
#define IOMG27_I2C2_SDA                0x06c
#define IOMG_GPIO_FUNC                 0x1

/*
 * HI6421's in-tree regmap is reg_stride=4, val_bits=8.  The PMIC control
 * registers therefore live four bytes apart but must be accessed as bytes.
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
#define TOUCH_RESET_PIN                4       /* GPIO156 = GPIO19_4 */
#define TOUCH_ATTN_PIN                 5       /* GPIO157 = GPIO19_5 */
#define I2C2_SCL_PIN                   7       /* GPIO63  = GPIO7_7 */
#define I2C2_SDA_PIN                   0       /* GPIO64  = GPIO8_0 */

static void hi3620_mediapad_touch_bitbang_prepare(void)
{
        void __iomem *iomux;
        void __iomem *gpio7;
        void __iomem *gpio8;
        u8 dir7;
        u8 dir8;
        u8 scl;
        u8 sda;

        iomux = ioremap(HI3620_IOMUX_PHYS, HI3620_MAP_SIZE);
        gpio7 = ioremap(HI3620_GPIO7_PHYS, HI3620_MAP_SIZE);
        gpio8 = ioremap(HI3620_GPIO8_PHYS, HI3620_MAP_SIZE);
        if (!iomux || !gpio7 || !gpio8) {
                pr_err("HI3620-TOUCH-BITBANG: failed to map IOMUX/GPIO7/GPIO8\n");
                goto out;
        }

        /* Disconnect the DesignWare block and route both pads to PL061 GPIO. */
        writel(IOMG_GPIO_FUNC, iomux + IOMG26_I2C2_SCL);
        writel(IOMG_GPIO_FUNC, iomux + IOMG27_I2C2_SDA);
        mb();

        /*
         * Release both lines.  i2c-gpio emulates open-drain signaling by
         * switching each PL061 pin between input (logic high via pull-up) and
         * output-low.  Starting as input also lets us verify the physical bus
         * idle level before the bitbang adapter probes.
         */
        dir7 = readb(gpio7 + PL061_GPIODIR) & ~BIT(I2C2_SCL_PIN);
        dir8 = readb(gpio8 + PL061_GPIODIR) & ~BIT(I2C2_SDA_PIN);
        writeb(dir7, gpio7 + PL061_GPIODIR);
        writeb(dir8, gpio8 + PL061_GPIODIR);
        mb();
        udelay(20);

        scl = !!(readb(gpio7 + PL061_DATA(I2C2_SCL_PIN)) & BIT(I2C2_SCL_PIN));
        sda = !!(readb(gpio8 + PL061_DATA(I2C2_SDA_PIN)) & BIT(I2C2_SDA_PIN));

        pr_info("HI3620-TOUCH-BITBANG: mux=%08x/%08x dir=%02x/%02x released scl=%u sda=%u\n",
                readl(iomux + IOMG26_I2C2_SCL),
                readl(iomux + IOMG27_I2C2_SDA),
                readb(gpio7 + PL061_GPIODIR),
                readb(gpio8 + PL061_GPIODIR), scl, sda);

out:
        if (gpio8)
                iounmap(gpio8);
        if (gpio7)
                iounmap(gpio7);
        if (iomux)
                iounmap(iomux);
}

static int __init hi3620_mediapad_touch_pad_prepare(void)
{
        void __iomem *iocfg;
        void __iomem *gpio;
        void __iomem *pmu;
        bool is_mediapad;
        bool is_current_dt;
        u32 cfg156;
        u32 cfg157;
        u8 ldo5_old;
        u8 ldo13_old;
        u8 ldo5_new;
        u8 ldo13_new;
        u8 dir;
        u8 attn;

        is_mediapad = of_machine_is_compatible("huawei,s10-101x");
        is_current_dt = of_machine_is_compatible("hisilicon,hi3620-hi4511");

        pr_info("HI3620-TOUCH: init compatible mediapad=%u hi4511=%u\n",
                is_mediapad, is_current_dt);

        if (!is_mediapad && !is_current_dt)
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

        /* Stock RMI4 rails: VDD/LDO13=2.85 V first, VBUS/LDO5=1.8 V second. */
        ldo5_old = readb(pmu + PMU_LDO5_CTRL);
        ldo13_old = readb(pmu + PMU_LDO13_CTRL);
        ldo5_new = (ldo5_old & ~PMU_LDO_CTRL_MASK) |
                   PMU_LDO_ENABLE | PMU_LDO5_1V8;
        ldo13_new = (ldo13_old & ~PMU_LDO_CTRL_MASK) |
                    PMU_LDO_ENABLE | PMU_LDO13_2V85;
        writeb(ldo13_new, pmu + PMU_LDO13_CTRL);
        writeb(ldo5_new, pmu + PMU_LDO5_CTRL);
        mb();

        /* Huawei's RMI4 probe waits 5 ms after enabling both supplies. */
        msleep(5);

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

        /* Match Huawei gpio_config(): RESET low for 10 ms, then release high. */
        writeb(0, gpio + PL061_DATA(TOUCH_RESET_PIN));
        msleep(10);
        writeb(BIT(TOUCH_RESET_PIN), gpio + PL061_DATA(TOUCH_RESET_PIN));

        attn = readb(gpio + PL061_DATA(TOUCH_ATTN_PIN));
        pr_info("HI3620-TOUCH-PAD: dir=%02x rst=%u attn=%u pull=%08x/%08x\n",
                readb(gpio + PL061_GPIODIR),
                !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                !!(attn & BIT(TOUCH_ATTN_PIN)),
                readl(iocfg + IOCG_GPIO156), readl(iocfg + IOCG_GPIO157));

        /* Leave physical I2C2 SCL/SDA in GPIO mode for the i2c-gpio adapter. */
        hi3620_mediapad_touch_bitbang_prepare();

        iounmap(pmu);
        iounmap(gpio);
        iounmap(iocfg);
        return 0;
}

/* Must run before the GPIO/I2C platform drivers start probing. */
postcore_initcall(hi3620_mediapad_touch_pad_prepare);
