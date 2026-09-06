// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Hi3620 USB2DVC/PicoPHY setup for Huawei MediaPad 10 FHD.
 *
 * The former delayed USB/display register watchdogs were bring-up aids and
 * produced a large amount of output after login.  Keep only the hardware
 * initialization needed by generic DWC2; touchscreen reset/power now lives in
 * the dedicated touch helper.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_USB2DVC_PHYS            0xfd240000
#define HI3620_MAP_SIZE                0x1000

#define SCTRL_CLK_EN1                  0x030
#define SCTRL_CLK_DIS1                 0x034
#define SCTRL_CLK_STATUS1              0x03c
#define SCTRL_CLK_EN3                  0x050
#define SCTRL_CLK_DIS3                 0x054
#define SCTRL_CLK_STATUS3              0x05c
#define SCTRL_RST_EN1                  0x08c
#define SCTRL_RST_DIS1                 0x090
#define SCTRL_RST_STATUS1              0x094
#define SCTRL_RST_EN3                  0x0a4
#define SCTRL_RST_DIS3                 0x0a8
#define SCTRL_RST_STATUS3              0x0ac

#define PCTRL_PERI_CTRL16              0x040
#define PCTRL_PERI_CTRL17              0x044
#define PCTRL_PERI_CTRL21              0x1f4
#define USB_GSNPSID                    0x040

#define CLK_USBPICOPHY                 BIT(24)
#define CLK_USB2DVC                    BIT(17)
#define RST_PICOPHY                    BIT(24)
#define PICOPHY_POR                    BIT(31)
#define RST_USB2DVC_PHY                BIT(28)
#define RST_USB2DVC                    BIT(17)

static int __init hi3620_mediapad_usb_prepare(void)
{
        void __iomem *sctrl;
        void __iomem *pctrl;
        void __iomem *usb;
        u32 val;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        pctrl = ioremap(HI3620_PCTRL_PHYS, HI3620_MAP_SIZE);
        usb = ioremap(HI3620_USB2DVC_PHYS, HI3620_MAP_SIZE);
        if (!sctrl || !pctrl || !usb) {
                pr_err("HI3620-USB: failed to map bring-up registers\n");
                if (usb)
                        iounmap(usb);
                if (pctrl)
                        iounmap(pctrl);
                if (sctrl)
                        iounmap(sctrl);
                return -ENOMEM;
        }

        writel(RST_USB2DVC | RST_USB2DVC_PHY | PICOPHY_POR,
               sctrl + SCTRL_RST_EN3);
        writel(RST_PICOPHY, sctrl + SCTRL_RST_EN1);
        writel(CLK_USBPICOPHY, sctrl + SCTRL_CLK_DIS1);
        writel(CLK_USB2DVC, sctrl + SCTRL_CLK_DIS3);

        val = readl(pctrl + PCTRL_PERI_CTRL16);
        val &= ~BIT(0);
        val |= BIT(31);
        val &= ~BIT(9);
        val &= ~(0x3 << 10);
        val &= ~(0x7 << 17);
        val |= (0x6 << 17);
        writel(val, pctrl + PCTRL_PERI_CTRL16);

        /* K3OEM/MediaPad stock PHY tune. */
        val = readl(pctrl + PCTRL_PERI_CTRL17);
        val &= ~0x3f;
        val |= 0x23;
        writel(val, pctrl + PCTRL_PERI_CTRL17);

        val = readl(pctrl + PCTRL_PERI_CTRL21);
        val &= ~((0x3 << 1) | (0x3 << 8) | (0x3 << 10));
        val |= ((0x1 << 1) | (0x3 << 8) | (0x1 << 10));
        writel(val, pctrl + PCTRL_PERI_CTRL21);

        writel(CLK_USBPICOPHY, sctrl + SCTRL_CLK_EN1);
        udelay(10);
        writel(RST_PICOPHY, sctrl + SCTRL_RST_DIS1);
        writel(PICOPHY_POR, sctrl + SCTRL_RST_DIS3);
        udelay(1000);
        writel(CLK_USB2DVC, sctrl + SCTRL_CLK_EN3);
        writel(RST_USB2DVC_PHY, sctrl + SCTRL_RST_DIS3);
        udelay(1);
        writel(RST_USB2DVC, sctrl + SCTRL_RST_DIS3);
        udelay(10);

        pr_info("HI3620-USB: ready gsnpsid=%08x clk1=%08x clk3=%08x rst1=%08x rst3=%08x\n",
                readl(usb + USB_GSNPSID),
                readl(sctrl + SCTRL_CLK_STATUS1),
                readl(sctrl + SCTRL_CLK_STATUS3),
                readl(sctrl + SCTRL_RST_STATUS1),
                readl(sctrl + SCTRL_RST_STATUS3));

        iounmap(usb);
        iounmap(pctrl);
        iounmap(sctrl);
        return 0;
}
arch_initcall(hi3620_mediapad_usb_prepare);
