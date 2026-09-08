// SPDX-License-Identifier: GPL-2.0
/*
 * MediaPad 10 FHD USB session-valid workaround.
 *
 * Hi3620's DWC2 core may come up with B-session invalid even though the
 * postmarketOS configfs gadget is ready.  Force the external VBUS-valid path
 * and generate a reconnect edge early enough for the host to enumerate it.
 * Retry a few times only while B-session is still invalid.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/workqueue.h>

#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_USB2DVC_PHYS            0xfd240000
#define HI3620_MAP_SIZE                0x1000

#define PCTRL_PERI_CTRL16              0x040
#define USB_GOTGCTL                    0x000
#define USB_DCTL                       0x804

#define GOTGCTL_BSESVLD                BIT(19)
#define DCTL_SFTDISCON                 BIT(1)
#define PCTRL16_VBUSVLDEXT_MASK        (0x3 << 10)

#define HI3620_USB_FIRST_DELAY         (5 * HZ)
#define HI3620_USB_RETRY_DELAY         (3 * HZ)
#define HI3620_USB_MAX_ATTEMPTS        3

static unsigned int hi3620_usb_session_attempts;
static void hi3620_usb_session_once(struct work_struct *work);
static DECLARE_DELAYED_WORK(hi3620_usb_session_work,
                            hi3620_usb_session_once);

static void hi3620_usb_session_once(struct work_struct *work)
{
        void __iomem *pctrl;
        void __iomem *usb;
        u32 gotgctl;
        u32 dctl;
        u32 p16;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return;

        pctrl = ioremap(HI3620_PCTRL_PHYS, HI3620_MAP_SIZE);
        usb = ioremap(HI3620_USB2DVC_PHYS, HI3620_MAP_SIZE);
        if (!pctrl || !usb) {
                if (usb)
                        iounmap(usb);
                if (pctrl)
                        iounmap(pctrl);
                return;
        }

        hi3620_usb_session_attempts++;
        gotgctl = readl(usb + USB_GOTGCTL);

        if (!(gotgctl & GOTGCTL_BSESVLD)) {
                p16 = readl(pctrl + PCTRL_PERI_CTRL16);
                p16 |= PCTRL16_VBUSVLDEXT_MASK;
                writel(p16, pctrl + PCTRL_PERI_CTRL16);
                mb();
                msleep(5);

                dctl = readl(usb + USB_DCTL);
                writel(dctl | DCTL_SFTDISCON, usb + USB_DCTL);
                mb();
                msleep(20);
                writel(dctl & ~DCTL_SFTDISCON, usb + USB_DCTL);
                mb();
                msleep(250);

                gotgctl = readl(usb + USB_GOTGCTL);
                pr_info("HI3620-USB-SESSION: attempt=%u gotgctl=%08x dctl=%08x pctrl16=%08x\n",
                        hi3620_usb_session_attempts, gotgctl,
                        readl(usb + USB_DCTL),
                        readl(pctrl + PCTRL_PERI_CTRL16));

                if (!(gotgctl & GOTGCTL_BSESVLD) &&
                    hi3620_usb_session_attempts < HI3620_USB_MAX_ATTEMPTS)
                        schedule_delayed_work(&hi3620_usb_session_work,
                                              HI3620_USB_RETRY_DELAY);
        }

        iounmap(usb);
        iounmap(pctrl);
}

static int __init hi3620_usb_session_start(void)
{
        if (of_machine_is_compatible("huawei,s10-101x"))
                schedule_delayed_work(&hi3620_usb_session_work,
                                      HI3620_USB_FIRST_DELAY);
        return 0;
}
late_initcall(hi3620_usb_session_start);
