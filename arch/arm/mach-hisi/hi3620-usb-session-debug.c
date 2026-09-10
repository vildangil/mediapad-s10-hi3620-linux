// SPDX-License-Identifier: GPL-2.0
/*
 * MediaPad 10 FHD DWC2 session diagnostics.
 *
 * Keep Huawei's real PicoPHY VBUS selector (PERI_CTRL16[11:10] = 0).  An
 * earlier diagnostic forced this field to 3 and toggled soft-disconnect when
 * B-session was invalid; stock Huawei explicitly clears the same field during
 * PHY setup, so the workaround could mask real cable/session detection.
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
#define USB_GAHBCFG                    0x008
#define USB_GUSBCFG                    0x00c
#define USB_GINTSTS                    0x014
#define USB_GINTMSK                    0x018
#define USB_DCFG                       0x800
#define USB_DCTL                       0x804
#define USB_DSTS                       0x808
#define USB_DAINT                      0x818
#define USB_DIEPCTL0                   0x900
#define USB_DOEPCTL0                   0xb00

#define GOTGCTL_BSESVLD                BIT(19)
#define PCTRL16_VBUSVLDEXTSEL_MASK     (0x3 << 10)

#define HI3620_USB_FIRST_DELAY         (2 * HZ)
#define HI3620_USB_RETRY_DELAY         (3 * HZ)
#define HI3620_USB_MAX_ATTEMPTS        4

static unsigned int hi3620_usb_state_attempts;
static void hi3620_usb_state_once(struct work_struct *work);
static DECLARE_DELAYED_WORK(hi3620_usb_state_work, hi3620_usb_state_once);

static void hi3620_usb_state_once(struct work_struct *work)
{
        void __iomem *pctrl;
        void __iomem *usb;
        u32 p16_before;
        u32 p16;
        u32 gotgctl;

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

        hi3620_usb_state_attempts++;

        /* Restore the selector used by Huawei setup_dvc_and_phy(). */
        p16_before = readl(pctrl + PCTRL_PERI_CTRL16);
        p16 = p16_before & ~PCTRL16_VBUSVLDEXTSEL_MASK;
        if (p16 != p16_before) {
                writel(p16, pctrl + PCTRL_PERI_CTRL16);
                mb();
                msleep(5);
        }

        gotgctl = readl(usb + USB_GOTGCTL);
        pr_info("HI3620-USB-STATE: attempt=%u bsesvld=%u gotgctl=%08x gahbcfg=%08x gusbcfg=%08x gintsts=%08x gintmsk=%08x dcfg=%08x dctl=%08x dsts=%08x daint=%08x diepctl0=%08x doepctl0=%08x pctrl16=%08x->%08x\n",
                hi3620_usb_state_attempts,
                !!(gotgctl & GOTGCTL_BSESVLD), gotgctl,
                readl(usb + USB_GAHBCFG), readl(usb + USB_GUSBCFG),
                readl(usb + USB_GINTSTS), readl(usb + USB_GINTMSK),
                readl(usb + USB_DCFG), readl(usb + USB_DCTL),
                readl(usb + USB_DSTS), readl(usb + USB_DAINT),
                readl(usb + USB_DIEPCTL0), readl(usb + USB_DOEPCTL0),
                p16_before, readl(pctrl + PCTRL_PERI_CTRL16));

        if (!(gotgctl & GOTGCTL_BSESVLD) &&
            hi3620_usb_state_attempts < HI3620_USB_MAX_ATTEMPTS)
                schedule_delayed_work(&hi3620_usb_state_work,
                                      HI3620_USB_RETRY_DELAY);

        iounmap(usb);
        iounmap(pctrl);
}

static int __init hi3620_usb_state_start(void)
{
        if (of_machine_is_compatible("huawei,s10-101x"))
                schedule_delayed_work(&hi3620_usb_state_work,
                                      HI3620_USB_FIRST_DELAY);
        return 0;
}
late_initcall(hi3620_usb_state_start);
