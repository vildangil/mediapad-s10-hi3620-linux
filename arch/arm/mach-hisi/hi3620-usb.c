// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Hi3620 USB2DVC/PicoPHY, touchscreen and late display diagnostics
 * for the Huawei MediaPad 10 FHD.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/workqueue.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_USB2DVC_PHYS            0xfd240000
#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_EDC0_PHYS               0xfa202000

#define HI3620_MAP_SIZE                0x1000
#define HI3620_EDC0_MAP_SIZE           0x2000

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

#define USB_GOTGCTL                    0x000
#define USB_GUSBCFG                    0x00c
#define USB_GINTSTS                    0x014
#define USB_GSNPSID                    0x040
#define USB_DCTL                       0x804
#define USB_DSTS                       0x808
#define USB_DCTL_SFTDISCON             BIT(1)

/* Bootloader-established EDC0/LDI/MIPI state used by simplefb. */
#define EDC_CH2_ADDR                   0x024
#define EDC_CH2_STRIDE                 0x02c
#define EDC_CH2_SIZE                   0x034
#define EDC_CH2_CTL                    0x038
#define EDC_DISP_SIZE                  0x090
#define EDC_DISP_CTL                   0x094
#define EDC_STS                        0x09c
#define LDI_DSP_SIZE                   0x814
#define LDI_CTRL                       0x820
#define LDI_WORK_MODE                  0x830
#define MIPI_PWR_UP                    0x904
#define MIPI_VID_MODE_CFG              0x91c
#define MIPI_PHY_RSTZ                  0x954
#define MIPI_PHY_STATUS                0x960

/* ARM PL061 register addressing. GPIO156/157 are GPIO19 pins 4/5. */
#define PL061_GPIODIR                  0x400
#define PL061_DATA(pin)                (BIT(pin) << 2)
#define TOUCH_RESET_PIN                4
#define TOUCH_ATTN_PIN                 5

#define CLK_USBPICOPHY                 BIT(24)
#define CLK_USB2DVC                    BIT(17)
#define RST_PICOPHY                    BIT(24)
#define PICOPHY_POR                    BIT(31)
#define RST_USB2DVC_PHY                BIT(28)
#define RST_USB2DVC                    BIT(17)

static unsigned int usb_watch_count;
static unsigned int display_watch_count;

static void hi3620_mediapad_usb_watch(struct work_struct *work);
static void hi3620_mediapad_display_watch(struct work_struct *work);
static DECLARE_DELAYED_WORK(hi3620_usb_watch_work,
			    hi3620_mediapad_usb_watch);
static DECLARE_DELAYED_WORK(hi3620_display_watch_work,
			    hi3620_mediapad_display_watch);

/*
 * The vendor board code explicitly drives GPIO156 low, waits, then releases it
 * high before registering the Synaptics RMI4 device. The upstream 4.9 RMI4
 * I2C transport has no reset-gpio handling. Do the same pulse here before
 * I2C/RMI probing. GPIO156/157 are dedicated pads without an IOMG selector.
 */
static void __init hi3620_mediapad_touch_reset(void)
{
	void __iomem *gpio;
	u8 dir;
	u8 attn;

	gpio = ioremap(HI3620_GPIO19_PHYS, HI3620_MAP_SIZE);
	if (!gpio) {
		pr_err("HI3620-TOUCH: failed to map GPIO19\n");
		return;
	}

	dir = readb(gpio + PL061_GPIODIR);
	writeb(dir | BIT(TOUCH_RESET_PIN), gpio + PL061_GPIODIR);

	writeb(0, gpio + PL061_DATA(TOUCH_RESET_PIN));
	msleep(10);
	writeb(BIT(TOUCH_RESET_PIN), gpio + PL061_DATA(TOUCH_RESET_PIN));
	msleep(100);

	attn = readb(gpio + PL061_DATA(TOUCH_ATTN_PIN));
	pr_info("HI3620-TOUCH: GPIO156 reset pulse complete; GPIO157(attn)=%u dir=%02x\n",
		!!(attn & BIT(TOUCH_ATTN_PIN)),
		readb(gpio + PL061_GPIODIR));

	iounmap(gpio);
}

/*
 * The console still sometimes turns black long after consoleblank=0, so sample
 * the inherited EDC/LDI/MIPI registers at 30 second intervals. This is
 * deliberately read-only: if the screen dies we can tell whether scanout,
 * LDI or the MIPI PHY was actually powered down rather than guessing.
 */
static void hi3620_mediapad_display_watch(struct work_struct *work)
{
	void __iomem *edc;

	if (!of_machine_is_compatible("huawei,s10-101x"))
		return;

	edc = ioremap(HI3620_EDC0_PHYS, HI3620_EDC0_MAP_SIZE);
	if (!edc) {
		pr_err("HI3620-DISP-WATCH: failed to map EDC0\n");
		return;
	}

	pr_info("HI3620-DISP-WATCH[%u]: ch2_addr=%08x stride=%08x size=%08x ctl=%08x disp_size=%08x disp_ctl=%08x sts=%08x\n",
		display_watch_count,
		readl(edc + EDC_CH2_ADDR), readl(edc + EDC_CH2_STRIDE),
		readl(edc + EDC_CH2_SIZE), readl(edc + EDC_CH2_CTL),
		readl(edc + EDC_DISP_SIZE), readl(edc + EDC_DISP_CTL),
		readl(edc + EDC_STS));
	pr_info("HI3620-DISP-WATCH[%u]: ldi_size=%08x ldi_ctrl=%08x ldi_work=%08x mipi_pwr=%08x vid=%08x phy_rst=%08x phy_sts=%08x\n",
		display_watch_count,
		readl(edc + LDI_DSP_SIZE), readl(edc + LDI_CTRL),
		readl(edc + LDI_WORK_MODE), readl(edc + MIPI_PWR_UP),
		readl(edc + MIPI_VID_MODE_CFG), readl(edc + MIPI_PHY_RSTZ),
		readl(edc + MIPI_PHY_STATUS));

	iounmap(edc);

	display_watch_count++;
	if (display_watch_count < 8)
		schedule_delayed_work(&hi3620_display_watch_work, 30 * HZ);
}

/*
 * Sample the DWC2 device-side state after configfs has had enough time to bind
 * its gadget. If the legacy DWC2 path left SoftDisconnect asserted, clear it
 * once. Subsequent samples make physical-enumeration failures visible.
 */
static void hi3620_mediapad_usb_watch(struct work_struct *work)
{
	void __iomem *usb;
	u32 dctl;

	if (!of_machine_is_compatible("huawei,s10-101x"))
		return;

	usb = ioremap(HI3620_USB2DVC_PHYS, HI3620_MAP_SIZE);
	if (!usb) {
		pr_err("HI3620-USB-WATCH: failed to map DWC2 registers\n");
		return;
	}

	dctl = readl(usb + USB_DCTL);
	pr_info("HI3620-USB-WATCH[%u]: gotgctl=%08x gusbcfg=%08x gintsts=%08x dctl=%08x dsts=%08x\n",
		usb_watch_count,
		readl(usb + USB_GOTGCTL), readl(usb + USB_GUSBCFG),
		readl(usb + USB_GINTSTS), dctl, readl(usb + USB_DSTS));

	if (dctl & USB_DCTL_SFTDISCON) {
		writel(dctl & ~USB_DCTL_SFTDISCON, usb + USB_DCTL);
		udelay(10);
		pr_warn("HI3620-USB-WATCH: cleared unexpected DCTL.SFTDISCON, dctl=%08x\n",
			readl(usb + USB_DCTL));
	}

	iounmap(usb);

	usb_watch_count++;
	if (usb_watch_count < 4)
		schedule_delayed_work(&hi3620_usb_watch_work, 30 * HZ);
}

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
		readl(sctrl + SCTRL_CLK_STATUS1), readl(sctrl + SCTRL_CLK_STATUS3),
		readl(sctrl + SCTRL_RST_STATUS1), readl(sctrl + SCTRL_RST_STATUS3),
		readl(pctrl + PCTRL_PERI_CTRL16), readl(pctrl + PCTRL_PERI_CTRL17),
		readl(pctrl + PCTRL_PERI_CTRL21), readl(usb + USB_GSNPSID));

	udelay(200);

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

	/* K3OEM/MediaPad stock tune: E_USBPHY_TUNE_PLATFORM == 0. */
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

	pr_info("HI3620-USB: post clk1=%08x clk3=%08x rst1=%08x rst3=%08x p16=%08x p17=%08x p21=%08x gsnpsid=%08x\n",
		readl(sctrl + SCTRL_CLK_STATUS1), readl(sctrl + SCTRL_CLK_STATUS3),
		readl(sctrl + SCTRL_RST_STATUS1), readl(sctrl + SCTRL_RST_STATUS3),
		readl(pctrl + PCTRL_PERI_CTRL16), readl(pctrl + PCTRL_PERI_CTRL17),
		readl(pctrl + PCTRL_PERI_CTRL21), readl(usb + USB_GSNPSID));

	iounmap(usb);
	iounmap(pctrl);
	iounmap(sctrl);

	hi3620_mediapad_touch_reset();

	schedule_delayed_work(&hi3620_display_watch_work, 30 * HZ);
	schedule_delayed_work(&hi3620_usb_watch_work, 45 * HZ);

	return 0;
}
arch_initcall(hi3620_mediapad_usb_prepare);
