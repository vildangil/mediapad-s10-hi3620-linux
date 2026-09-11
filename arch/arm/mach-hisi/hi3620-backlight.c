// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD S10-101x PWM0 backlight bridge.
 *
 * The bootloader already leaves the Panasonic VVX10F002A00 panel and its
 * GPIO18_5 PWM mux configured.  Do not disturb the panel or pinmux here;
 * simply expose the existing Hi3620 PWM0 block through the Linux backlight
 * class so XFCE/UPower can control brightness.
 */

#include <linux/backlight.h>
#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PWM0_PHYS               0xfca05000
#define HI3620_MAP_SIZE                0x1000

#define SCTRL_CLK_EN2                  0x040
#define SCTRL_CLK_STATUS2              0x04c
#define SCTRL_RST_DIS2                 0x09c
#define SCTRL_RST_STATUS2              0x0a0

#define CLK_PWM0                       BIT(7)
#define RST_PWM0                       BIT(7)

/* Huawei drivers/video/k3/backlight_pwm.c programming model. */
#define PWM_ENABLE                     0x00
#define PWM_DIV                        0x08
#define PWM_OUT                        0x10
#define PWM_MAX_DIV                    0xe0
#define PANEL_BL_MAX                   100

struct hi3620_backlight {
	void __iomem *sctrl;
	void __iomem *pwm;
	struct backlight_device *bd;
};

static struct hi3620_backlight s10_bl;

static unsigned int hi3620_bl_level_from_hw(struct hi3620_backlight *bl)
{
	u32 enabled = readl(bl->pwm + PWM_ENABLE);
	u32 duty = readl(bl->pwm + PWM_OUT);

	if (!(enabled & 1) || !duty)
		return 0;
	if (duty >= PWM_MAX_DIV)
		return PANEL_BL_MAX;

	return DIV_ROUND_CLOSEST(duty * PANEL_BL_MAX, PWM_MAX_DIV);
}

static int hi3620_bl_update_status(struct backlight_device *bd)
{
	struct hi3620_backlight *bl = bl_get_data(bd);
	unsigned int level = bd->props.brightness;
	u32 duty;

	if (bd->props.power != FB_BLANK_UNBLANK ||
	    bd->props.fb_blank != FB_BLANK_UNBLANK ||
	    (bd->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)))
		level = 0;

	if (level > PANEL_BL_MAX)
		level = PANEL_BL_MAX;

	if (!level) {
		writel(0, bl->pwm + PWM_OUT);
		mb();
		writel(0, bl->pwm + PWM_ENABLE);
		mb();
		pr_info_ratelimited("HI3620-BACKLIGHT: level=0 pwm disabled\n");
		return 0;
	}

	/* Vendor formula: duty = level * 0xe0 / bl_max. */
	duty = DIV_ROUND_CLOSEST(level * PWM_MAX_DIV, PANEL_BL_MAX);
	if (!duty)
		duty = 1;

	/* The bootloader already selects/configures the PWM parent/divider.
	 * Only ensure the gate is enabled and reset deasserted.  Changing the
	 * clock divider here can glitch a currently visible simplefb panel.
	 */
	writel(CLK_PWM0, bl->sctrl + SCTRL_CLK_EN2);
	mb();
	if (readl(bl->sctrl + SCTRL_RST_STATUS2) & RST_PWM0) {
		writel(RST_PWM0, bl->sctrl + SCTRL_RST_DIS2);
		mb();
	}

	writel(PWM_MAX_DIV, bl->pwm + PWM_DIV);
	writel(duty, bl->pwm + PWM_OUT);
	mb();
	writel(1, bl->pwm + PWM_ENABLE);
	mb();

	pr_info_ratelimited("HI3620-BACKLIGHT: level=%u duty=%u div=%08x enable=%08x\n",
			    level, duty, readl(bl->pwm + PWM_DIV),
			    readl(bl->pwm + PWM_ENABLE));
	return 0;
}

static int hi3620_bl_get_brightness(struct backlight_device *bd)
{
	struct hi3620_backlight *bl = bl_get_data(bd);

	return hi3620_bl_level_from_hw(bl);
}

static const struct backlight_ops hi3620_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = hi3620_bl_update_status,
	.get_brightness = hi3620_bl_get_brightness,
};

static int __init hi3620_mediapad_backlight_init(void)
{
	struct backlight_properties props;
	u32 clk2, rst2, enable, div, duty;
	unsigned int level;

	if (!of_machine_is_compatible("huawei,s10-101x"))
		return 0;

	s10_bl.sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
	s10_bl.pwm = ioremap(HI3620_PWM0_PHYS, HI3620_MAP_SIZE);
	if (!s10_bl.sctrl || !s10_bl.pwm) {
		pr_err("HI3620-BACKLIGHT: ioremap failed\n");
		if (s10_bl.pwm)
			iounmap(s10_bl.pwm);
		if (s10_bl.sctrl)
			iounmap(s10_bl.sctrl);
		return -ENOMEM;
	}

	clk2 = readl(s10_bl.sctrl + SCTRL_CLK_STATUS2);
	rst2 = readl(s10_bl.sctrl + SCTRL_RST_STATUS2);
	enable = readl(s10_bl.pwm + PWM_ENABLE);
	div = readl(s10_bl.pwm + PWM_DIV);
	duty = readl(s10_bl.pwm + PWM_OUT);
	level = hi3620_bl_level_from_hw(&s10_bl);

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = PANEL_BL_MAX;
	props.brightness = level;
	props.power = FB_BLANK_UNBLANK;
	props.fb_blank = FB_BLANK_UNBLANK;

	s10_bl.bd = backlight_device_register("hi3620-backlight", NULL,
					       &s10_bl, &hi3620_bl_ops, &props);
	if (IS_ERR(s10_bl.bd)) {
		int ret = PTR_ERR(s10_bl.bd);

		pr_err("HI3620-BACKLIGHT: register failed rc=%d\n", ret);
		iounmap(s10_bl.pwm);
		iounmap(s10_bl.sctrl);
		return ret;
	}

	pr_info("HI3620-BACKLIGHT: registered current=%u/100 pwm_enable=%08x div=%08x duty=%08x clk2=%08x rst2=%08x\n",
		level, enable, div, duty, clk2, rst2);
	return 0;
}
device_initcall(hi3620_mediapad_backlight_init);
