// SPDX-License-Identifier: GPL-2.0
/*
 * Small MediaPad 10 FHD board-specific bring-up helpers.
 *
 * Keep only recovery that is useful in normal boot.  I2C2 clock/reset/MMIO
 * setup deliberately lives out of this file for the current touchscreen
 * GPIO-bitbang isolation test: the hardware DesignWare I2C2 controller is
 * disabled in DT and must not touch the physical SCL/SDA pads.
 */

#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/workqueue.h>

#define HI3620_PWM0_PHYS               0xfca05000
#define HI3620_GPIO21_PHYS             0xfc81b000
#define HI3620_MAP_SIZE                0x1000

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

        (void)work;

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
