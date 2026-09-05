// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Hi3620 USB2DVC/PicoPHY bring-up for the MediaPad 10 FHD.
 *
 * The generic DWC2 driver can handle the Synopsys core once it is clocked
 * and released from reset, but Huawei's boot flow also requires a small SoC
 * specific PicoPHY sequence.  This is intentionally limited to the register
 * programming performed by the stock K3V2 USB glue; the Android gadget stack
 * is not carried over.
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

	pr_info("HI3620-USB: pre clk1=%08x clk3=%08x rst1=%08x rst3=%08x p16=%08x p17=%08x p21=%08x gsnpsid=%08x\n",
		readl(sctrl + SCTRL_CLK_STATUS1),
		readl(sctrl + SCTRL_CLK_STATUS3),
		readl(sctrl + SCTRL_RST_STATUS1),
		readl(sctrl + SCTRL_RST_STATUS3),
		readl(pctrl + PCTRL_PERI_CTRL16),
		readl(pctrl + PCTRL_PERI_CTRL17),
		readl(pctrl + PCTRL_PERI_CTRL21),
		readl(usb + USB_GSNPSID));

	/* The stock driver enables the USB rails before this point.  Regulators
	 * are not described yet in the upstream Hi3620 DT, so retain the rails
	 * left by the bootloader and reproduce only the reset/clock/PHY sequence.
	 */
	udelay(200);

	/* Hold DWC2 core, its interface PHY and PicoPHY in reset. */
	writel(RST_USB2DVC | RST_USB2DVC_PHY | PICOPHY_POR,
	       sctrl + SCTRL_RST_EN3);
	writel(RST_PICOPHY, sctrl + SCTRL_RST_EN1);

	/* Stop both clocks while programming the PHY controls. */
	writel(CLK_USBPICOPHY, sctrl + SCTRL_CLK_DIS1);
	writel(CLK_USB2DVC, sctrl + SCTRL_CLK_DIS3);

	/* Exact common PicoPHY settings from Huawei's setup_dvc_and_phy(). */
	val = readl(pctrl + PCTRL_PERI_CTRL16);
	val &= ~BIT(0);             /* PICOPHY_SIDDQ */
	val |= BIT(31);             /* TX pre-emphasis tune */
	val &= ~BIT(9);             /* usb1_phy_otgdisable */
	val &= ~(0x3 << 10);        /* usb1_phy_vbusvldextsel */
	val &= ~(0x7 << 17);        /* disconnect threshold */
	val |= (0x6 << 17);
	writel(val, pctrl + PCTRL_PERI_CTRL16);

	/* Keep PERI_CTRL17's bootloader board-specific tune value. */

	val = readl(pctrl + PCTRL_PERI_CTRL21);
	val &= ~((0x3 << 1) | (0x3 << 8) | (0x3 << 10));
	val |= ((0x1 << 1) | (0x3 << 8) | (0x1 << 10));
	writel(val, pctrl + PCTRL_PERI_CTRL21);

	/* Start PicoPHY, then release its reset/POR. */
	writel(CLK_USBPICOPHY, sctrl + SCTRL_CLK_EN1);
	udelay(10);
	writel(RST_PICOPHY, sctrl + SCTRL_RST_DIS1);
	writel(PICOPHY_POR, sctrl + SCTRL_RST_DIS3);
	udelay(1000);

	/* Start the DWC2 clock, release interface PHY, then the core itself. */
	writel(CLK_USB2DVC, sctrl + SCTRL_CLK_EN3);
	writel(RST_USB2DVC_PHY, sctrl + SCTRL_RST_DIS3);
	udelay(1);
	writel(RST_USB2DVC, sctrl + SCTRL_RST_DIS3);
	udelay(10);

	pr_info("HI3620-USB: post clk1=%08x clk3=%08x rst1=%08x rst3=%08x p16=%08x p17=%08x p21=%08x gsnpsid=%08x\n",
		readl(sctrl + SCTRL_CLK_STATUS1),
		readl(sctrl + SCTRL_CLK_STATUS3),
		readl(sctrl + SCTRL_RST_STATUS1),
		readl(sctrl + SCTRL_RST_STATUS3),
		readl(pctrl + PCTRL_PERI_CTRL16),
		readl(pctrl + PCTRL_PERI_CTRL17),
		readl(pctrl + PCTRL_PERI_CTRL21),
		readl(usb + USB_GSNPSID));

	iounmap(usb);
	iounmap(pctrl);
	iounmap(sctrl);

	return 0;
}
arch_initcall(hi3620_mediapad_usb_prepare);
