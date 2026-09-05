// SPDX-License-Identifier: GPL-2.0
/*
 * Small MediaPad 10 FHD board-specific bring-up helpers.
 *
 * Keep this separate from generic Hi3620 support: it only runs on the
 * huawei,s10-101x compatible and exists to reproduce vendor setup that is
 * missing from the upstream DT/DesignWare drivers while bring-up is ongoing.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
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

/* Stock K3V2 iomux: IOMG26/IOMG27 function 0 = I2C2 SCL/SDA. */
#define IOMG26_I2C2_SCL                0x068
#define IOMG27_I2C2_SDA                0x06c

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
static void hi3620_mediapad_i2c_probe(struct work_struct *work);
static DECLARE_DELAYED_WORK(hi3620_bl_watch_work,
                            hi3620_mediapad_backlight_watch);
static DECLARE_DELAYED_WORK(hi3620_i2c_probe_work,
                            hi3620_mediapad_i2c_probe);

static int __init hi3620_mediapad_i2c2_prepare(void)
{
        void __iomem *sctrl;
        void __iomem *iomux;
        void __iomem *pctrl;
        void __iomem *i2c;
        u32 stat;
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

        pr_info("HI3620-I2C2: pre mux_scl=%08x mux_sda=%08x rst=%08x enable=%08x status=%08x comp=%08x ver=%08x\n",
                readl(iomux + IOMG26_I2C2_SCL),
                readl(iomux + IOMG27_I2C2_SDA),
                readl(sctrl + SCTRL_I2C_RST_STAT),
                readl(i2c + DW_IC_ENABLE), readl(i2c + DW_IC_STATUS),
                readl(i2c + DW_IC_COMP_TYPE), readl(i2c + DW_IC_COMP_VERSION));

        /* Force the physical pads to the function documented by Huawei. */
        writel(0, iomux + IOMG26_I2C2_SCL);
        writel(0, iomux + IOMG27_I2C2_SDA);

        /* Reset the DesignWare block exactly like vendor common.c. */
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

        /* This write uses Huawei's write-mask convention and only touches I2C2. */
        writel(I2C2_ENABLE_DELAY_SDA, pctrl + PCTRL_I2C23_DELAY);
        udelay(10);

        pr_info("HI3620-I2C2: post mux_scl=%08x mux_sda=%08x rst=%08x enable=%08x status=%08x comp=%08x ver=%08x\n",
                readl(iomux + IOMG26_I2C2_SCL),
                readl(iomux + IOMG27_I2C2_SDA),
                readl(sctrl + SCTRL_I2C_RST_STAT),
                readl(i2c + DW_IC_ENABLE), readl(i2c + DW_IC_STATUS),
                readl(i2c + DW_IC_COMP_TYPE), readl(i2c + DW_IC_COMP_VERSION));

        iounmap(i2c);
        iounmap(pctrl);
        iounmap(iomux);
        iounmap(sctrl);
        return 0;
}
postcore_initcall(hi3620_mediapad_i2c2_prepare);

/*
 * Probe the Linux-visible I2C adapters after OF population.  The stock
 * K3V2OEM1 build uses Synaptics RMI4 at 0x70.  We deliberately probe all
 * early adapter numbers because upstream DT does not guarantee that the
 * physical I2C2 controller becomes /dev/i2c-2.  A successful page-select and
 * PDT read proves the electrical bus/device separately from the RMI driver.
 */
static void hi3620_mediapad_i2c_probe(struct work_struct *work)
{
        struct i2c_adapter *adap;
        struct i2c_msg msg;
        struct i2c_msg msgs[2];
        u8 page[2] = { 0xff, 0x00 };
        u8 reg = 0xe9;
        u8 pdt[6] = { 0 };
        int nr;
        int ret_page;
        int ret_pdt;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return;

        for (nr = 0; nr < 4; nr++) {
                adap = i2c_get_adapter(nr);
                if (!adap) {
                        pr_info("HI3620-TOUCH-PROBE: i2c-%d absent\n", nr);
                        continue;
                }

                memset(&msg, 0, sizeof(msg));
                msg.addr = 0x70;
                msg.len = sizeof(page);
                msg.buf = page;
                ret_page = i2c_transfer(adap, &msg, 1);

                memset(msgs, 0, sizeof(msgs));
                msgs[0].addr = 0x70;
                msgs[0].len = 1;
                msgs[0].buf = &reg;
                msgs[1].addr = 0x70;
                msgs[1].flags = I2C_M_RD;
                msgs[1].len = sizeof(pdt);
                msgs[1].buf = pdt;
                ret_pdt = i2c_transfer(adap, msgs, 2);

                pr_info("HI3620-TOUCH-PROBE: i2c-%d name='%s' page=%d pdt=%d data=%02x %02x %02x %02x %02x %02x\n",
                        nr, adap->name, ret_page, ret_pdt,
                        pdt[0], pdt[1], pdt[2], pdt[3], pdt[4], pdt[5]);
                i2c_put_adapter(adap);
        }
}

/*
 * The inherited EDC/LDI/MIPI path stays stable while the panel sometimes
 * later becomes black.  Sample PWM0 and LCD_POWER too.  If either inherited
 * backlight control is actually turned off, restore only the already-proven
 * bootloader values so bring-up remains usable while we find the writer.
 */
static void hi3620_mediapad_backlight_watch(struct work_struct *work)
{
        void __iomem *pwm;
        void __iomem *gpio;
        u32 ctl;
        u32 div;
        u32 out;
        u32 power;
        u8 dir;
        bool repaired = false;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return;

        pwm = ioremap(HI3620_PWM0_PHYS, HI3620_MAP_SIZE);
        gpio = ioremap(HI3620_GPIO21_PHYS, HI3620_MAP_SIZE);
        if (!pwm || !gpio) {
                pr_err("HI3620-BL-WATCH: failed to map PWM0/GPIO21\n");
                if (gpio)
                        iounmap(gpio);
                if (pwm)
                        iounmap(pwm);
                return;
        }

        ctl = readl(pwm + PWM_CTL);
        div = readl(pwm + PWM_DIV);
        out = readl(pwm + PWM_OUT);
        power = readb(gpio + PL061_DATA(LCD_POWER_PIN));
        dir = readb(gpio + PL061_GPIODIR);

        pr_info("HI3620-BL-WATCH[%u]: pwm_ctl=%08x pwm_div=%08x pwm_out=%08x lcd_power=%u gpio21_dir=%02x\n",
                bl_watch_count, ctl, div, out,
                !!(power & BIT(LCD_POWER_PIN)), dir);

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
                pr_warn("HI3620-BL-WATCH: restored inherited backlight: pwm_ctl=%08x pwm_div=%08x pwm_out=%08x lcd_power=%u dir=%02x\n",
                        readl(pwm + PWM_CTL), readl(pwm + PWM_DIV),
                        readl(pwm + PWM_OUT),
                        !!(readb(gpio + PL061_DATA(LCD_POWER_PIN)) & BIT(LCD_POWER_PIN)),
                        readb(gpio + PL061_GPIODIR));

        iounmap(gpio);
        iounmap(pwm);

        bl_watch_count++;
        if (bl_watch_count < 20)
                schedule_delayed_work(&hi3620_bl_watch_work, 15 * HZ);
}

static int __init hi3620_mediapad_late_debug_start(void)
{
        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        schedule_delayed_work(&hi3620_i2c_probe_work, 15 * HZ);
        schedule_delayed_work(&hi3620_bl_watch_work, 15 * HZ);
        return 0;
}
late_initcall(hi3620_mediapad_late_debug_start);
