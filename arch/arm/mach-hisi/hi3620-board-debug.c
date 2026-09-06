// SPDX-License-Identifier: GPL-2.0
/*
 * Small MediaPad 10 FHD board-specific bring-up helpers.
 *
 * Keep only setup/recovery that is still useful in normal boot.  Routine
 * post-login register dumps were valuable during bring-up but now just bury
 * the console and ramoops.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/workqueue.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_IOMUX_PHYS              0xfc803000
#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_I2C2_PHYS               0xfcb0c000
#define HI3620_PWM0_PHYS               0xfca05000
#define HI3620_GPIO21_PHYS             0xfc81b000
#define HI3620_MAP_SIZE                0x1000

/*
 * Upstream hi3620-hi4511.dts documents the silicon mux encoding directly:
 * function 0 is I2C2 SCL/SDA, while function 1 is the GPIO/idle state.
 */
#define IOMG26_I2C2_SCL                0x068
#define IOMG27_I2C2_SDA                0x06c
#define IOMG_I2C_FUNC                  0x0
#define IOCG79_I2C2_SCL                0x918
#define IOCG80_I2C2_SDA                0x91c
#define IOCG_PULL_MASK                 0x3
#define IOCG_NOPULL                    0x0

/* Stock common.c I2C controller reset registers. */
#define SCTRL_I2C_RST_EN               0x098
#define SCTRL_I2C_RST_DIS              0x09c
#define SCTRL_I2C_RST_STAT             0x0a0
#define I2C2_RESET_BIT                 BIT(28)

/* Vendor 300 ns SDA-delay write-mask/value command for I2C2. */
#define PCTRL_I2C23_DELAY              0x00c
#define I2C2_ENABLE_DELAY_SDA          0x00100010

/* Synopsys DesignWare identity registers. */
#define DW_IC_ENABLE                   0x06c
#define DW_IC_STATUS                   0x070
#define DW_IC_COMP_VERSION             0x0f8
#define DW_IC_COMP_TYPE                0x0fc

/* Bootloader-established PWM0 backlight state. */
#define PWM_CTL                        0x000
#define PWM_DIV                        0x008
#define PWM_OUT                        0x010
#define PWM_CTL_ON                     0x00000001
#define PWM_DIV_BOOT                   0x000000e0
#define PWM_OUT_SAFE                   0x00000040

/* Panasonic panel power is stock GPIO_21_3 == GPIO171. */
#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define LCD_POWER_PIN                  3

static unsigned int bl_watch_count;
static void hi3620_mediapad_backlight_watch(struct work_struct *work);
static DECLARE_DELAYED_WORK(hi3620_bl_watch_work,
                            hi3620_mediapad_backlight_watch);

static int __init hi3620_mediapad_i2c2_prepare(void)
{
        void __iomem *sctrl;
        void __iomem *iomux;
        void __iomem *pctrl;
        void __iomem *i2c;
        u32 stat;
        u32 pad;
        unsigned int timeout;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        iomux = ioremap(HI3620_IOMUX_PHYS, HI3620_MAP_SIZE);
        pctrl = ioremap(HI3620_PCTRL_PHYS, HI3620_MAP_SIZE);
        i2c = ioremap(HI3620_I2C2_PHYS, HI3620_MAP_SIZE);
        if (!sctrl || !iomux || !pctrl || !i2c) {
                pr_err("HI3620-I2C2: failed to map setup registers\n");
                if (i2c)
                        iounmap(i2c);
                if (pctrl)
                        iounmap(pctrl);
                if (iomux)
                        iounmap(iomux);
                if (sctrl)
                        iounmap(sctrl);
                return -ENOMEM;
        }

        /* Match the upstream/vendor NORMAL state: function 0, no pulls. */
        writel(IOMG_I2C_FUNC, iomux + IOMG26_I2C2_SCL);
        writel(IOMG_I2C_FUNC, iomux + IOMG27_I2C2_SDA);
        pad = readl(iomux + IOCG79_I2C2_SCL);
        writel((pad & ~IOCG_PULL_MASK) | IOCG_NOPULL,
               iomux + IOCG79_I2C2_SCL);
        pad = readl(iomux + IOCG80_I2C2_SDA);
        writel((pad & ~IOCG_PULL_MASK) | IOCG_NOPULL,
               iomux + IOCG80_I2C2_SDA);

        /* Reset the DesignWare block like vendor common.c. */
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

        pr_info("HI3620-I2C2: mux=%08x/%08x pad=%08x/%08x rst=%08x comp=%08x ver=%08x\n",
                readl(iomux + IOMG26_I2C2_SCL),
                readl(iomux + IOMG27_I2C2_SDA),
                readl(iomux + IOCG79_I2C2_SCL),
                readl(iomux + IOCG80_I2C2_SDA),
                readl(sctrl + SCTRL_I2C_RST_STAT),
                readl(i2c + DW_IC_COMP_TYPE), readl(i2c + DW_IC_COMP_VERSION));

        iounmap(i2c);
        iounmap(pctrl);
        iounmap(iomux);
        iounmap(sctrl);
        return 0;
}
postcore_initcall(hi3620_mediapad_i2c2_prepare);

/*
 * Keep the backlight recovery because it stopped the two-minute black-screen
 * regression, but stay completely silent unless something actually needs to
 * be restored.
 */
static void hi3620_mediapad_backlight_watch(struct work_struct *work)
{
        void __iomem *pwm;
        void __iomem *gpio;
        u32 ctl;
        u32 out;
        u32 power;
        u8 dir;
        bool repaired = false;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return;

        pwm = ioremap(HI3620_PWM0_PHYS, HI3620_MAP_SIZE);
        gpio = ioremap(HI3620_GPIO21_PHYS, HI3620_MAP_SIZE);
        if (!pwm || !gpio) {
                if (gpio)
                        iounmap(gpio);
                if (pwm)
                        iounmap(pwm);
                return;
        }

        ctl = readl(pwm + PWM_CTL);
        out = readl(pwm + PWM_OUT);
        power = readb(gpio + PL061_DATA(LCD_POWER_PIN));
        dir = readb(gpio + PL061_GPIODIR);

        if (!(dir & BIT(LCD_POWER_PIN)) || !(power & BIT(LCD_POWER_PIN))) {
                writeb(dir | BIT(LCD_POWER_PIN), gpio + PL061_GPIODIR);
                writeb(BIT(LCD_POWER_PIN), gpio + PL061_DATA(LCD_POWER_PIN));
                repaired = true;
        }

        if (!(ctl & 1) || out == 0) {
                writel(PWM_DIV_BOOT, pwm + PWM_DIV);
                writel(PWM_OUT_SAFE, pwm + PWM_OUT);
                writel(PWM_CTL_ON, pwm + PWM_CTL);
                repaired = true;
        }

        if (repaired)
                pr_warn("HI3620-BACKLIGHT: restored inherited panel/backlight state\n");

        iounmap(gpio);
        iounmap(pwm);

        bl_watch_count++;
        if (bl_watch_count < 20)
                schedule_delayed_work(&hi3620_bl_watch_work, 15 * HZ);
}

static int __init hi3620_mediapad_late_recovery_start(void)
{
        if (of_machine_is_compatible("huawei,s10-101x"))
                schedule_delayed_work(&hi3620_bl_watch_work, 15 * HZ);
        return 0;
}
late_initcall(hi3620_mediapad_late_recovery_start);
