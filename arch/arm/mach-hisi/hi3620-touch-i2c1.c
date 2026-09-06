// SPDX-License-Identifier: GPL-2.0
/*
 * MediaPad 10 FHD touchscreen bus diagnostics.
 *
 * Stock places Synaptics RMI4 at 0x70 on physical I2C2.  Keep I2C1 alive as
 * a comparison path, but late diagnostics now probe only 0x70 and print only
 * the two relevant adapters instead of flooding the console after login.
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

static unsigned int hi3620_touch_scan_count;
static void hi3620_touch_scan_workfn(struct work_struct *work);
static DECLARE_DELAYED_WORK(hi3620_touch_scan_work,
                            hi3620_touch_scan_workfn);

static int __init hi3620_mediapad_i2c1_prepare(void)
{
        void __iomem *sctrl;
        void __iomem *iomux;
        void __iomem *pctrl;
        u32 stat;
        unsigned int timeout;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        iomux = ioremap(HI3620_IOMUX_PHYS, HI3620_MAP_SIZE);
        pctrl = ioremap(HI3620_PCTRL_PHYS, HI3620_MAP_SIZE);
        if (!sctrl || !iomux || !pctrl) {
                if (pctrl)
                        iounmap(pctrl);
                if (iomux)
                        iounmap(iomux);
                if (sctrl)
                        iounmap(sctrl);
                return -ENOMEM;
        }

        writel(IOMG46_FUNC_I2C1, iomux + IOMG46_I2C1);

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

        iounmap(pctrl);
        iounmap(iomux);
        iounmap(sctrl);

        /* First concise check near login, then at most two retries. */
        schedule_delayed_work(&hi3620_touch_scan_work, 60 * HZ);
        return 0;
}
postcore_initcall(hi3620_mediapad_i2c1_prepare);

static int hi3620_touch_probe_70(unsigned int scan, int nr)
{
        struct i2c_adapter *adap;
        struct i2c_msg msg;
        u8 value = 0;
        int ret;

        adap = i2c_get_adapter(nr);
        if (!adap) {
                pr_info("HI3620-TOUCH-SCAN[%u]: i2c-%d absent\n", scan, nr);
                return -ENODEV;
        }

        memset(&msg, 0, sizeof(msg));
        msg.addr = 0x70;
        msg.flags = I2C_M_RD;
        msg.len = 1;
        msg.buf = &value;
        ret = i2c_transfer(adap, &msg, 1);

        pr_info("HI3620-TOUCH-SCAN[%u]: i2c-%d 0x70 ret=%d byte=%02x\n",
                scan, nr, ret, value);
        i2c_put_adapter(adap);
        return ret;
}

static void hi3620_touch_scan_workfn(struct work_struct *work)
{
        void __iomem *iocfg;
        void __iomem *gpio;
        int ret0;
        int ret1;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return;

        iocfg = ioremap(HI3620_IOCFG_PHYS, HI3620_MAP_SIZE);
        gpio = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
        if (iocfg && gpio) {
                pr_info("HI3620-TOUCH-LATE[%u]: dir=%02x rst=%u attn=%u pull=%08x/%08x\n",
                        hi3620_touch_scan_count,
                        readb(gpio + PL061_GPIODIR),
                        !!(readb(gpio + PL061_DATA(TOUCH_RESET_PIN)) & BIT(TOUCH_RESET_PIN)),
                        !!(readb(gpio + PL061_DATA(TOUCH_ATTN_PIN)) & BIT(TOUCH_ATTN_PIN)),
                        readl(iocfg + IOCG_GPIO156),
                        readl(iocfg + IOCG_GPIO157));
        }
        if (gpio)
                iounmap(gpio);
        if (iocfg)
                iounmap(iocfg);

        ret0 = hi3620_touch_probe_70(hi3620_touch_scan_count, 0);
        ret1 = hi3620_touch_probe_70(hi3620_touch_scan_count, 1);

        /* Once either bus answers, stop polluting the console. */
        if (ret0 == 1 || ret1 == 1)
                return;

        hi3620_touch_scan_count++;
        if (hi3620_touch_scan_count < 3)
                schedule_delayed_work(&hi3620_touch_scan_work, 30 * HZ);
}
