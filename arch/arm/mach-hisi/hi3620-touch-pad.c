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
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/workqueue.h>

#define HI3620_IOMUX_PHYS              0xfc803000
#define HI3620_IOCFG_PHYS              0xfc803800
#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_GPIO7_PHYS              0xfc80d000
#define HI3620_GPIO8_PHYS              0xfc80e000
#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_PMUSPI_PHYS             0xfcc00000
#define HI3620_I2C2_PHYS               0xfcb0c000
#define HI3620_MAP_SIZE                0x1000

#define IOCG_GPIO156                   0x00c
#define IOCG_GPIO157                   0x010
#define IOCG_PULL_MASK                 0x3
#define IOCG_NOPULL                    0x0
#define IOCG_PULLUP                    0x1

/* I2C2 mux/pads from Huawei block_i2c2. NORMAL is FUNC0, GPIO/idle is FUNC1. */
#define IOMG26_I2C2_SCL                0x068
#define IOMG27_I2C2_SDA                0x06c
#define IOMG_I2C_FUNC                  0x0
#define IOMG_GPIO_FUNC                 0x1

/* Vendor I2C2 controller reset + 300 ns SDA delay. */
#define SCTRL_I2C_RST_EN               0x098
#define SCTRL_I2C_RST_DIS              0x09c
#define SCTRL_I2C_RST_STAT             0x0a0
#define I2C2_RESET_BIT                 BIT(28)
#define PCTRL_I2C23_DELAY              0x00c
#define I2C2_ENABLE_DELAY_SDA          0x00100010

/* DesignWare I2C registers used by the temporary touchscreen trace. */
#define DW_IC_CON                       0x000
#define DW_IC_TAR                       0x004
#define DW_IC_SS_SCL_HCNT               0x014
#define DW_IC_SS_SCL_LCNT               0x018
#define DW_IC_FS_SCL_HCNT               0x01c
#define DW_IC_FS_SCL_LCNT               0x020
#define DW_IC_RAW_INTR_STAT             0x034
#define DW_IC_ENABLE                    0x06c
#define DW_IC_STATUS                    0x070
#define DW_IC_TXFLR                     0x074
#define DW_IC_RXFLR                     0x078
#define DW_IC_TX_ABRT_SOURCE            0x080
#define DW_IC_ENABLE_STATUS             0x09c

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
#define I2C2_SCL_PIN                   7       /* GPIO63 = GPIO7_7 */
#define I2C2_SDA_PIN                   0       /* GPIO64 = GPIO8_0 */

#define TOUCH_I2C_DIAG_SAMPLES         60
#define TOUCH_I2C_DIAG_PERIOD_MS       50

static struct delayed_work hi3620_touch_i2c_diag_work;
static unsigned int hi3620_touch_i2c_diag_iter;

static void hi3620_touch_i2c_diag_workfn(struct work_struct *work)
{
        void __iomem *i2c;
        unsigned int n = hi3620_touch_i2c_diag_iter;

        (void)work;

        if (n >= TOUCH_I2C_DIAG_SAMPLES)
                return;

        i2c = ioremap(HI3620_I2C2_PHYS, HI3620_MAP_SIZE);
        if (!i2c) {
                pr_err("HI3620-TOUCH-I2C-DIAG: failed to map I2C2\n");
                return;
        }

        pr_info("HI3620-TOUCH-I2C-DIAG[%u]: con=%08x tar=%08x ss=%08x/%08x fs=%08x/%08x raw=%08x en=%08x stat=%08x tx=%08x rx=%08x abrt=%08x enstat=%08x\n",
                n,
                readl(i2c + DW_IC_CON),
                readl(i2c + DW_IC_TAR),
                readl(i2c + DW_IC_SS_SCL_HCNT),
                readl(i2c + DW_IC_SS_SCL_LCNT),
                readl(i2c + DW_IC_FS_SCL_HCNT),
                readl(i2c + DW_IC_FS_SCL_LCNT),
                readl(i2c + DW_IC_RAW_INTR_STAT),
                readl(i2c + DW_IC_ENABLE),
                readl(i2c + DW_IC_STATUS),
                readl(i2c + DW_IC_TXFLR),
                readl(i2c + DW_IC_RXFLR),
                readl(i2c + DW_IC_TX_ABRT_SOURCE),
                readl(i2c + DW_IC_ENABLE_STATUS));

        iounmap(i2c);

        hi3620_touch_i2c_diag_iter = n + 1;
        if (hi3620_touch_i2c_diag_iter < TOUCH_I2C_DIAG_SAMPLES)
                schedule_delayed_work(&hi3620_touch_i2c_diag_work,
                                      msecs_to_jiffies(TOUCH_I2C_DIAG_PERIOD_MS));
}

static void hi3620_mediapad_i2c2_bus_recover(void)
{
        void __iomem *iomux;
        void __iomem *gpio7;
        void __iomem *gpio8;
        void __iomem *sctrl;
        void __iomem *pctrl;
        u8 dir7;
        u8 dir8;
        u8 scl;
        u8 sda;
        u32 stat;
        unsigned int timeout;

        iomux = ioremap(HI3620_IOMUX_PHYS, HI3620_MAP_SIZE);
        gpio7 = ioremap(HI3620_GPIO7_PHYS, HI3620_MAP_SIZE);
        gpio8 = ioremap(HI3620_GPIO8_PHYS, HI3620_MAP_SIZE);
        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        pctrl = ioremap(HI3620_PCTRL_PHYS, HI3620_MAP_SIZE);
        if (!iomux || !gpio7 || !gpio8 || !sctrl || !pctrl)
                goto out;

        /*
         * Reproduce Huawei's I2C2_reset() sequence after the touchscreen is
         * powered and released from reset: temporarily mux SCL/SDA as GPIO,
         * pulse SCL low->high and force SDA high, then return to I2C FUNC0.
         */
        writel(IOMG_GPIO_FUNC, iomux + IOMG26_I2C2_SCL);
        writel(IOMG_GPIO_FUNC, iomux + IOMG27_I2C2_SDA);
        mb();

        dir7 = readb(gpio7 + PL061_GPIODIR);
        dir8 = readb(gpio8 + PL061_GPIODIR);
        writeb(dir7 | BIT(I2C2_SCL_PIN), gpio7 + PL061_GPIODIR);
        writeb(dir8 | BIT(I2C2_SDA_PIN), gpio8 + PL061_GPIODIR);

        writeb(0, gpio7 + PL061_DATA(I2C2_SCL_PIN));
        udelay(5);
        writeb(BIT(I2C2_SCL_PIN), gpio7 + PL061_DATA(I2C2_SCL_PIN));
        writeb(BIT(I2C2_SDA_PIN), gpio8 + PL061_DATA(I2C2_SDA_PIN));
        udelay(5);

        scl = !!(readb(gpio7 + PL061_DATA(I2C2_SCL_PIN)) & BIT(I2C2_SCL_PIN));
        sda = !!(readb(gpio8 + PL061_DATA(I2C2_SDA_PIN)) & BIT(I2C2_SDA_PIN));

        writel(IOMG_I2C_FUNC, iomux + IOMG26_I2C2_SCL);
        writel(IOMG_I2C_FUNC, iomux + IOMG27_I2C2_SDA);
        mb();

        /* Reset the DesignWare controller once more after the bus recovery. */
        writel(I2C2_RESET_BIT, sctrl + SCTRL_I2C_RST_EN);
        timeout = 1000;
        do {
                stat = readl(sctrl + SCTRL_I2C_RST_STAT);
                if (stat & I2C2_RESET_BIT)
                        break;
                udelay(1);
        } while (--timeout);

        udelay(2);
        writel(I2C2_RESET_BIT, sctrl + SCTRL_I2C_RST_DIS);
        timeout = 1000;
        do {
                stat = readl(sctrl + SCTRL_I2C_RST_STAT);
                if (!(stat & I2C2_RESET_BIT))
                        break;
                udelay(1);
        } while (--timeout);

        writel(I2C2_ENABLE_DELAY_SDA, pctrl + PCTRL_I2C23_DELAY);
        udelay(10);

        pr_info("HI3620-TOUCH-I2C2: recovered scl=%u sda=%u mux=%08x/%08x rst=%08x\n",
                scl, sda,
                readl(iomux + IOMG26_I2C2_SCL),
                readl(iomux + IOMG27_I2C2_SDA),
                readl(sctrl + SCTRL_I2C_RST_STAT));

out:
        if (pctrl)
                iounmap(pctrl);
        if (sctrl)
                iounmap(sctrl);
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

        /*
         * The branch still boots the upstream hi3620-hi4511 DTB, even though
         * it has been repurposed for the MediaPad 10 FHD.  Accept that root
         * compatible until the board DT gets its own MediaPad compatible.
         */
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

        /* Stock platform data provides I2C2_reset(); perform that recovery now. */
        hi3620_mediapad_i2c2_bus_recover();

        /*
         * Temporary bring-up trace.  Sample I2C2 every 50 ms for three seconds
         * so ramoops catches the controller immediately before, during and
         * after the first RMI page-select transaction.
         */
        hi3620_touch_i2c_diag_iter = 0;
        INIT_DELAYED_WORK(&hi3620_touch_i2c_diag_work,
                          hi3620_touch_i2c_diag_workfn);
        schedule_delayed_work(&hi3620_touch_i2c_diag_work,
                              msecs_to_jiffies(TOUCH_I2C_DIAG_PERIOD_MS));

        iounmap(pmu);
        iounmap(gpio);
        iounmap(iocfg);
        return 0;
}

/* Must run before the DesignWare I2C platform driver's subsys_initcall. */
postcore_initcall(hi3620_mediapad_touch_pad_prepare);
