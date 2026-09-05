// SPDX-License-Identifier: GPL-2.0
/*
 * MediaPad 10 FHD USB session-valid bring-up experiment.
 *
 * Generic DWC2 reaches peripheral mode and clears DCTL.SFTDISCON, but current
 * logs show GOTGCTL.BSESVLD=0 while a cable is physically connected.  The
 * Hi3620 PicoPHY exposes two adjacent vbusvldext signals through PERI_CTRL16
 * bits 10/11.  Huawei's normal path clears both to use the internal detector.
 * During bring-up, if that detector is still low after userspace bound the
 * gadget, force both external-valid controls high and pulse soft-disconnect so
 * the host gets a fresh connect edge.  This is deliberately board-specific
 * and only runs when BSESVLD is missing.
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
#define USB_GINTSTS                    0x014
#define USB_DCTL                       0x804
#define USB_DSTS                       0x808

#define GOTGCTL_BSESVLD                BIT(19)
#define DCTL_SFTDISCON                 BIT(1)
#define PCTRL16_VBUSVLDEXT_MASK        (0x3 << 10)

static unsigned int hi3620_session_watch_count;

static void hi3620_usb_session_watch(struct work_struct *work);
static DECLARE_DELAYED_WORK(hi3620_usb_session_work,
                            hi3620_usb_session_watch);

static void hi3620_usb_session_watch(struct work_struct *work)
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
                pr_err("HI3620-USB-SESSION: failed to map registers\n");
                if (usb)
                        iounmap(usb);
                if (pctrl)
                        iounmap(pctrl);
                return;
        }

        gotgctl = readl(usb + USB_GOTGCTL);
        dctl = readl(usb + USB_DCTL);
        p16 = readl(pctrl + PCTRL_PERI_CTRL16);

        pr_info("HI3620-USB-SESSION[%u]: gotgctl=%08x bsesvld=%u p16=%08x dctl=%08x dsts=%08x gintsts=%08x\n",
                hi3620_session_watch_count, gotgctl,
                !!(gotgctl & GOTGCTL_BSESVLD), p16, dctl,
                readl(usb + USB_DSTS), readl(usb + USB_GINTSTS));

        if (!(gotgctl & GOTGCTL_BSESVLD)) {
                /*
                 * Treat bit11 as external-valid select and bit10 as the
                 * external-valid value.  Force both high for this diagnostic
                 * peripheral-only build.
                 */
                p16 |= PCTRL16_VBUSVLDEXT_MASK;
                writel(p16, pctrl + PCTRL_PERI_CTRL16);
                msleep(5);

                pr_warn("HI3620-USB-SESSION: forced vbusvldext bits, p16=%08x gotgctl=%08x\n",
                        readl(pctrl + PCTRL_PERI_CTRL16),
                        readl(usb + USB_GOTGCTL));

                /* Generate a clean device reconnect edge for the PC. */
                dctl = readl(usb + USB_DCTL);
                writel(dctl | DCTL_SFTDISCON, usb + USB_DCTL);
                msleep(20);
                writel((dctl | DCTL_SFTDISCON) & ~DCTL_SFTDISCON,
                       usb + USB_DCTL);
                msleep(5);

                pr_warn("HI3620-USB-SESSION: reconnect pulse done gotgctl=%08x dctl=%08x dsts=%08x\n",
                        readl(usb + USB_GOTGCTL), readl(usb + USB_DCTL),
                        readl(usb + USB_DSTS));
        }

        iounmap(usb);
        iounmap(pctrl);

        hi3620_session_watch_count++;
        if (hi3620_session_watch_count < 4)
                schedule_delayed_work(&hi3620_usb_session_work, 30 * HZ);
}

static int __init hi3620_usb_session_debug_start(void)
{
        if (of_machine_is_compatible("huawei,s10-101x"))
                schedule_delayed_work(&hi3620_usb_session_work, 20 * HZ);
        return 0;
}
late_initcall(hi3620_usb_session_debug_start);
