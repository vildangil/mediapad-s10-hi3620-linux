// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Huawei MediaPad 10 FHD Synaptics reset helper.
 *
 * The vendor K3V2 board file drives GPIO156 (GPIO19 pin 4) low for 10 ms,
 * then high for 10 ms before the RMI4 transport probes.  Linux 4.9's generic
 * RMI4 I2C transport has no reset-gpio DT support, so reproduce just that
 * board-level pulse early enough that the normal upstream RMI4 driver can be
 * used unchanged.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_clk.h>

#define PL061_GPIODIR		0x400
#define PL061_DATA_MASK(pin)	(BIT(pin) << 2)

#define TOUCH_RESET_PIN		4
#define TOUCH_ATTN_PIN		5

static int __init hi3620_mediapad_touch_reset(void)
{
	struct device_node *np;
	struct clk *clk = NULL;
	void __iomem *base;
	u32 dir;
	u32 attn;
	int ret;

	if (!of_machine_is_compatible("huawei,s10-101x"))
		return 0;

	np = of_find_node_by_path("/amba/gpio@819000");
	if (!np) {
		pr_warn("HI3620-TOUCH: GPIO19 DT node not found\n");
		return 0;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("HI3620-TOUCH: failed to map GPIO19\n");
		of_node_put(np);
		return 0;
	}

	clk = of_clk_get(np, 0);
	if (!IS_ERR(clk)) {
		ret = clk_prepare_enable(clk);
		if (ret)
			pr_warn("HI3620-TOUCH: GPIO19 clock enable failed: %d\n", ret);
	} else {
		pr_warn("HI3620-TOUCH: GPIO19 clock lookup failed: %ld\n",
			PTR_ERR(clk));
		clk = NULL;
	}

	/* PL061 masked-data addressing: offset = (1 << pin) << 2. */
	dir = readl(base + PL061_GPIODIR);
	writel(dir | BIT(TOUCH_RESET_PIN), base + PL061_GPIODIR);

	/* Assert active-low reset, matching Huawei's vendor board code. */
	writel(0, base + PL061_DATA_MASK(TOUCH_RESET_PIN));
	msleep(10);

	/* Deassert reset and give the controller time to boot. */
	writel(BIT(TOUCH_RESET_PIN),
	       base + PL061_DATA_MASK(TOUCH_RESET_PIN));
	msleep(20);

	attn = readl(base + PL061_DATA_MASK(TOUCH_ATTN_PIN));
	pr_info("HI3620-TOUCH: reset GPIO156 pulsed, GPIO157(attn)=%u dir=%08x\n",
		 !!(attn & BIT(TOUCH_ATTN_PIN)),
		 readl(base + PL061_GPIODIR));

	if (clk) {
		clk_disable_unprepare(clk);
		clk_put(clk);
	}

	iounmap(base);
	of_node_put(np);
	return 0;
}
arch_initcall(hi3620_mediapad_touch_reset);
