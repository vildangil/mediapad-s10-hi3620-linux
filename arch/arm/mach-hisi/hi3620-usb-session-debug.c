// SPDX-License-Identifier: GPL-2.0
/*
 * MediaPad 10 FHD USB session-valid workaround.
 *
 * Keep the one operation that made the host notice the gadget: if DWC2 still
 * sees B-session invalid after userspace binds configfs, force Hi3620's
 * external VBUS-valid controls and generate one reconnect edge.  The repeated
 * EP0/register dumps are intentionally gone; USB protocol debugging can be
 * re-enabled later without filling the console after login.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
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

        gotgctl = readl(usb + USB_GOTGCTL);
        if (!(gotgctl & GOTGCTL_BSESVLD)) {
                p16 = readl(pctrl + PCTRL_PERI_CTRL16);
                p16 |= PCTRL16_VBUSVLDEXT_MASK;
                writel(p16, pctrl + PCTRL_PERI_CTRL16);
                msleep(5);

                dctl = readl(usb + USB_DCTL);
                writel(dctl | DCTL_SFTDISCON, usb + USB_DCTL);
                msleep(20);
                writel((dctl | DCTL_SFTDISCON) & ~DCTL_SFTDISCON,
                       usb + USB_DCTL);
        }

        iounmap(usb);
        iounmap(pctrl);
}

static int __init hi3620_usb_session_start(void)
{
        if (of_machine_is_compatible("huawei,s10-101x"))
                schedule_delayed_work(&hi3620_usb_session_work, 20 * HZ);
        return 0;
}
late_initcall(hi3620_usb_session_start);
