// SPDX-License-Identifier: GPL-2.0
/*
 * MediaPad 10 FHD alternate touchscreen-bus bring-up.
 *
 * Huawei's K3V2 board files mention units where the touch controller may be
 * wired differently across board revisions.  The production S10-101x source
 * registers Synaptics RMI4 at 0x70 on I2C2, but keeping I2C1 alive as an
 * alternate path is useful while identifying the exact panel fitted to this
 * tablet.
 *
 * I2C1 is DesignWare at 0xfcb09000, GIC SPI 29.  Stock iomux identifies
 * GPIO86/GPIO87 as i2c1_scl/i2c1_sda through IOMG46 function 1.  Vendor
 * common.c also resets the block with SCTRL bit 25 and enables the 300 ns SDA
 * delay using the PCTRL write-mask command 0x00020002.
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
#define HI3620_IOCFG_PHYS              0xfc803800
#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_I2C1_PHYS               0xfcb09000
#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_MAP_SIZE                0x1000

#define IOMG46_I2C1                    0x0b8
#define IOMG46_FUNC_I2C1               0x1

#define IOCG_GPIO156                   0x00c
#define IOCG_GPIO157                   0x010
#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TOUCH_RESET_PIN                4
#define TOUCH_ATTN_PIN                 5

#define SCTRL_I2C_RST_EN               0x098
#define SCTRL_I2C_RST_DIS              0x09c
#define SCTRL_I2C_RST_STAT             0x0a0
#define I2C1_RESET_BIT                 BIT(25)

#define PCTRL_I2C_DELAY                0x00c
#define I2C1_ENABLE_DELAY_SDA          0x00020002

#define DW_IC_ENABLE                   0x06c
#define DW_IC_STATUS                   0x070
#define DW_IC_COMP_VERSION             0x0f8
#define DW_IC_COMP_TYPE                0x0fc

static unsigned int hi3620_touch_scan_count;
static void hi3620_touch_scan_workfn(struct work_struct *work);
static DECLARE_DELAYED_WORK(hi3620_touch_scan_work,
                            hi3620_touch_scan_workfn);

static int __init hi3620_mediapad_i2c1_prepare(void)
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
        i2c = ioremap(HI3620_I2C1_PHYS, HI3620_MAP_SIZE);
        if (!sctrl || !iomux || !pctrl || !i2c) {
                pr_err("HI3620-I2C1: failed to map setup registers\n");
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

        pr_info("HI3620-I2C1: pre mux=%08x rst=%08x enable=%08x status=%08x comp=%08x ver=%08x\n",
                readl(iomux + IOMG46_I2C1),
                readl(sctrl + SCTRL_I2C_RST_STAT),
                readl(i2c + DW_IC_ENABLE), readl(i2c + DW_IC_STATUS),
                readl(i2c + DW_IC_COMP_TYPE), readl(i2c + DW_IC_COMP_VERSION));

        /* GPIO86/GPIO87 -> I2C1 SCL/SDA. */
        writel(IOMG46_FUNC_I2C1, iomux + IOMG46_I2C1);

        /* Mirror Huawei common.c's I2C1 controller reset sequence. */
        writel(I2C1_RESET_BIT, sctrl + SCTRL_I2C_RST_EN);
        timeout = 1000;
        do {
                stat = readl(sctrl + SCTRL_I2C_RST_STAT);
                if (stat & I2C1_RESET_BIT)
                        break;
                udelay(1);
        } while (--timeout);

        udelay(2);
        writel(I2C1_RESET_BIT, sctrl + SCTRL_I2C_RST_DIS);
        timeout = 1000;
        do {
                stat = readl(sctrl + SCTRL_I2C_RST_STAT);
                if (!(stat & I2C1_RESET_BIT))
                        break;
                udelay(1);
        } while (--timeout);

        writel(I2C1_ENABLE_DELAY_SDA, pctrl + PCTRL_I2C_DELAY);
        udelay(10);

        pr_info("HI3620-I2C1: post mux=%08x rst=%08x enable=%08x status=%08x comp=%08x ver=%08x\n",
                readl(iomux + IOMG46_I2C1),
                readl(sctrl + SCTRL_I2C_RST_STAT),
                readl(i2c + DW_IC_ENABLE), readl(i2c + DW_IC_STATUS),
                readl(i2c + DW_IC_COMP_TYPE), readl(i2c + DW_IC_COMP_VERSION));

        iounmap(i2c);
        iounmap(pctrl);
        iounmap(iomux);
        iounmap(sctrl);

        /*
         * Print the actual bus/GPIO state after userspace has nearly reached
         * login.  Earlier bring-up messages scroll off fbcon too quickly to be
         * useful from a photo.  Repeat three times at 30 second intervals.
         */
        schedule_delayed_work(&hi3620_touch_scan_work, 60 * HZ);
        return 0;
}
postcore_initcall(hi3620_mediapad_i2c1_prepare);

/*
 * Read one byte from the common controller addresses used in Huawei K3V2
 * sources.  This is deliberately tiny and read-only: it tells us which bus
 * actually sees a touch IC without performing a destructive full I2C scan.
 *
 *   0x70  Synaptics RMI4
 *   0x4a/0x4b  Atmel maXTouch common application addresses
 *   0x1a  Cypress TTSP address used by Huawei K3V2 variants
 */
static void hi3620_touch_scan_workfn(struct work_struct *work)
{
        static const u8 addrs[] = { 0x70, 0x4a, 0x4b, 0x1a };
        struct i2c_adapter *adap;
        struct i2c_msg msg;
        void __iomem *iocfg;
        void __iomem *gpio;
        u8 value;
        int nr;
        int i;
        int ret;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return;

        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        if (iocfg && gpio) {
                pr_info("HI3620-TOUCH-LATE[%u]: iocg156=%08x iocg157=%08x dir=%02x rst=%u attn=%u\n",
                        hi3620_touch_scan_count,
                        readl(iocfg + IOCG_GPIO156),
                        readl(iocfg + IOCG_GPIO157),
                        readb(gpio + PL061_GPIODIR),
                        !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                        !!(readb(gpio + PL061_DATA(TOUCH_ATTN_PIN)) & BIT(TOUCH_ATTN_PIN)));
        } else {
                pr_err("HI3620-TOUCH-LATE[%u]: failed to map IOCG/GPIO19\n",
                       hi3620_touch_scan_count);
        }
        if (gpio)
                iounmap(gpio);
        if (iocfg)
                iounmap(iocfg);

        for (nr = 0; nr < 4; nr++) {
                adap = i2c_get_adapter(nr);
                if (!adap) {
                        pr_info("HI3620-TOUCH-SCAN[%u]: i2c-%d absent\n",
                                hi3620_touch_scan_count, nr);
                        continue;
                }

                pr_info("HI3620-TOUCH-SCAN[%u]: i2c-%d name='%s'\n",
                        hi3620_touch_scan_count, nr, adap->name);

                for (i = 0; i < ARRAY_SIZE(addrs); i++) {
                        value = 0;
                        memset(&msg, 0, sizeof(msg));
                        msg.addr = addrs[i];
                        msg.flags = I2C_M_RD;
                        msg.len = 1;
                        msg.buf = &value;
                        ret = i2c_transfer(adap, &msg, 1);

                        pr_info("HI3620-TOUCH-SCAN[%u]: i2c-%d addr=0x%02x ret=%d byte=%02x\n",
                                hi3620_touch_scan_count, nr,
                                addrs[i], ret, value);
                }

                i2c_put_adapter(adap);
        }

        hi3620_touch_scan_count++;
        if (hi3620_touch_scan_count < 3)
                schedule_delayed_work(&hi3620_touch_scan_work, 30 * HZ);
}
