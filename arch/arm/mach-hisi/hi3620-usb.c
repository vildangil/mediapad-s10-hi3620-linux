// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Hi3620 USB2DVC/PicoPHY setup for Huawei MediaPad 10 FHD.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_PMUSPI_PHYS             0xfcc00000
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

/* Huawei's stock USB driver gets usb20-vcc from HI6421 LDO4 and usbphy-vcc
 * from LDO8 before touching PicoPHY. Preserve the bootloader voltage select
 * and only assert the normal LDO enable bit. */
#define PMU_LDO4_CTRL                  (0x24 << 2)
#define PMU_LDO8_CTRL                  (0x28 << 2)
#define PMU_LDO_ENABLE                 0x10

#define CLK_USBPICOPHY                 BIT(24)
#define CLK_USB2DVC                    BIT(17)
#define RST_PICOPHY                    BIT(24)
#define PICOPHY_POR                    BIT(31)
#define RST_USB2DVC_PHY                BIT(28)
#define RST_USB2DVC                    BIT(17)

static int __init hi3620_mediapad_usb_prepare(void)
{
        void __iomem *sctrl = NULL;
        void __iomem *pctrl = NULL;
        void __iomem *pmu = NULL;
        void __iomem *usb = NULL;
        u32 val;
        u8 ldo4_old;
        u8 ldo8_old;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        pctrl = ioremap(HI3620_PCTRL_PHYS, HI3620_MAP_SIZE);
        pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_MAP_SIZE);
        usb = ioremap(HI3620_USB2DVC_PHYS, HI3620_MAP_SIZE);
        if (!sctrl || !pctrl || !pmu || !usb) {
                pr_err("HI3620-USB: failed to map bring-up registers\n");
                goto out;
        }

        /* This was missing from the generic-DWC2 bring-up. With both rails
         * absent the core itself was readable and configfs bound, but the host
         * reported descriptor errors and B-session never became valid. */
        ldo4_old = readb(pmu + PMU_LDO4_CTRL);
        ldo8_old = readb(pmu + PMU_LDO8_CTRL);
        writeb(ldo4_old | PMU_LDO_ENABLE, pmu + PMU_LDO4_CTRL);
        writeb(ldo8_old | PMU_LDO_ENABLE, pmu + PMU_LDO8_CTRL);
        mb();
        udelay(250);
        pr_info("HI3620-USB-PWR: ldo4 %02x->%02x ldo8 %02x->%02x\n",
                ldo4_old, readb(pmu + PMU_LDO4_CTRL),
                ldo8_old, readb(pmu + PMU_LDO8_CTRL));

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

out:
        if (usb)
                iounmap(usb);
        if (pmu)
                iounmap(pmu);
        if (pctrl)
                iounmap(pctrl);
        if (sctrl)
                iounmap(sctrl);
        return 0;
}
arch_initcall(hi3620_mediapad_usb_prepare);
