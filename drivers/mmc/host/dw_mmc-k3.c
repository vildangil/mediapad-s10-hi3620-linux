/*
 * Copyright (c) 2013 Linaro Ltd.
 * Copyright (c) 2013 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/host.h>
#include <linux/mmc/dw_mmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

/*
 * hi6220 sd only support io voltage 1.8v and 3v
 * Also need config AO_SCTRL_SEL18 accordingly
 */
#define AO_SCTRL_SEL18		BIT(10)
#define AO_SCTRL_CTRL3		0x40C

/* MediaPad K3V2 SCTRL clock/reset wiring. */
#define K3V2_SCTRL_PHYS		0xfc802000
#define K3V2_SCTRL_SIZE		0x1000
#define K3V2_CLK_EN3		0x050
#define K3V2_CLK_STATUS3	0x05c
#define K3V2_RST_EN3		0x0a4
#define K3V2_RST_DIS3		0x0a8
#define K3V2_RST_STATUS3	0x0ac
#define K3V2_CLK_DDRC_PER	BIT(9)
#define K3V2_CLK_SD		BIT(20)
#define K3V2_RST_SD		BIT(23)
#define K3V2_CLK_MMC3		BIT(23)
#define K3V2_RST_MMC3		BIT(26)
#define K3V2_SD_BASE		0xfcd03000
#define K3V2_MMC3_BASE		0xfcd06000

struct k3_priv {
	struct regmap	*reg;
};

static unsigned long dw_mci_hi6220_caps[] = {
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
	0
};

static void dw_mci_k3_dump_reset_state(struct dw_mci *host, const char *tag)
{
	dev_info(host->dev,
		 "MediaPad %s CTRL=%08x CLKENA=%08x STATUS=%08x BMOD=%08x FIFOTH=%08x HCON=%08x\n",
		 tag, mci_readl(host, CTRL), mci_readl(host, CLKENA),
		 mci_readl(host, STATUS), mci_readl(host, BMOD),
		 mci_readl(host, FIFOTH), mci_readl(host, HCON));
}

/*
 * The vendor K3V2 clock framework couples the SD/MMC3 controller gate to
 * clk_ddrc_per and owns a separate module reset.  Do the same immediately
 * before the DesignWare internal reset, after dw_mmc has enabled CIU/BIU.
 * The working eMMC controller at fcd04000 is deliberately never touched.
 */
static void dw_mci_k3_external_reset(struct dw_mci *host)
{
	struct platform_device *pdev = to_platform_device(host->dev);
	struct resource *res;
	void __iomem *sctrl;
	u32 gate, reset, before_clk, before_rst;
	unsigned int timeout;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return;

	if (res->start == K3V2_SD_BASE) {
		gate = K3V2_CLK_SD;
		reset = K3V2_RST_SD;
	} else if (res->start == K3V2_MMC3_BASE) {
		gate = K3V2_CLK_MMC3;
		reset = K3V2_RST_MMC3;
	} else {
		return;
	}

	sctrl = ioremap(K3V2_SCTRL_PHYS, K3V2_SCTRL_SIZE);
	if (!sctrl) {
		dev_warn(host->dev, "MediaPad: cannot map SCTRL for external reset\n");
		return;
	}

	before_clk = readl(sctrl + K3V2_CLK_STATUS3);
	before_rst = readl(sctrl + K3V2_RST_STATUS3);

	writel(K3V2_CLK_DDRC_PER | gate, sctrl + K3V2_CLK_EN3);
	mb();
	udelay(10);

	writel(reset, sctrl + K3V2_RST_EN3);
	mb();
	for (timeout = 0; timeout < 100; timeout++) {
		if (readl(sctrl + K3V2_RST_STATUS3) & reset)
			break;
		udelay(1);
	}

	udelay(5);
	writel(reset, sctrl + K3V2_RST_DIS3);
	mb();
	for (timeout = 0; timeout < 100; timeout++) {
		if (!(readl(sctrl + K3V2_RST_STATUS3) & reset))
			break;
		udelay(1);
	}
	udelay(20);

	dev_info(host->dev,
		 "MediaPad external reset base=%pa clk3=%08x->%08x rst3=%08x->%08x ddrc_per=%u gate=%u reset=%u\n",
		 &res->start, before_clk, readl(sctrl + K3V2_CLK_STATUS3),
		 before_rst, readl(sctrl + K3V2_RST_STATUS3),
		 !!(readl(sctrl + K3V2_CLK_STATUS3) & K3V2_CLK_DDRC_PER),
		 !!(readl(sctrl + K3V2_CLK_STATUS3) & gate),
		 !!(readl(sctrl + K3V2_RST_STATUS3) & reset));

	iounmap(sctrl);
}

/*
 * The Huawei K3V2 vendor dw_mmc driver resets CTRL/FIFO/DMA together and
 * waits up to 500 ms for all three bits to self-clear.  Match that sequence
 * exactly, board-scoped.  For SD/MMC3 also repeat the SoC-level module reset
 * at probe time while their CIU/BIU clocks are unquestionably running.
 */
static int dw_mci_k3_init(struct dw_mci *host)
{
	unsigned long timeout;
	u32 mask = SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
		   SDMMC_CTRL_DMA_RESET;
	u32 ctrl;

	if (!of_machine_is_compatible("huawei,s10-101x"))
		return 0;

	dw_mci_k3_dump_reset_state(host, "pre-reset");
	dw_mci_k3_external_reset(host);

	dev_info(host->dev, "MediaPad K3V2 Huawei combined controller reset\n");
	mci_writel(host, CTRL, mask);
	timeout = jiffies + msecs_to_jiffies(500);

	do {
		ctrl = mci_readl(host, CTRL);
		if (!(ctrl & mask)) {
			dw_mci_k3_dump_reset_state(host, "post-reset");
			dev_info(host->dev,
				 "MediaPad K3V2 combined controller reset complete (CTRL=%08x)\n",
				 ctrl);
			return 0;
		}
		cpu_relax();
	} while (time_before(jiffies, timeout));

	dw_mci_k3_dump_reset_state(host, "reset-timeout");
	dev_err(host->dev,
		"MediaPad Huawei combined reset timed out (CTRL=%08x)\n",
		mci_readl(host, CTRL));
	return -ETIMEDOUT;
}

static void dw_mci_k3_set_ios(struct dw_mci *host, struct mmc_ios *ios)
{
	int ret;

	ret = clk_set_rate(host->ciu_clk, ios->clock);
	if (ret)
		dev_warn(host->dev, "failed to set rate %uHz\n", ios->clock);

	host->bus_hz = clk_get_rate(host->ciu_clk);
}

static const struct dw_mci_drv_data k3_drv_data = {
	.init			= dw_mci_k3_init,
	.set_ios		= dw_mci_k3_set_ios,
};

static int dw_mci_hi6220_parse_dt(struct dw_mci *host)
{
	struct k3_priv *priv;

	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->reg = syscon_regmap_lookup_by_phandle(host->dev->of_node,
					 "hisilicon,peripheral-syscon");
	if (IS_ERR(priv->reg))
		priv->reg = NULL;

	host->priv = priv;
	return 0;
}

static int dw_mci_hi6220_switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct k3_priv *priv;
	struct dw_mci *host;
	int min_uv, max_uv;
	int ret;

	host = slot->host;
	priv = host->priv;

	if (!priv || !priv->reg)
		return 0;

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		ret = regmap_update_bits(priv->reg, AO_SCTRL_CTRL3,
					 AO_SCTRL_SEL18, 0);
		min_uv = 3000000;
		max_uv = 3000000;
	} else if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		ret = regmap_update_bits(priv->reg, AO_SCTRL_CTRL3,
					 AO_SCTRL_SEL18, AO_SCTRL_SEL18);
		min_uv = 1800000;
		max_uv = 1800000;
	} else {
		dev_dbg(host->dev, "voltage not supported\n");
		return -EINVAL;
	}

	if (ret) {
		dev_dbg(host->dev, "switch voltage failed\n");
		return ret;
	}

	if (IS_ERR_OR_NULL(mmc->supply.vqmmc))
		return 0;

	ret = regulator_set_voltage(mmc->supply.vqmmc, min_uv, max_uv);
	if (ret) {
		dev_dbg(host->dev, "Regulator set error %d: %d - %d\n",
				 ret, min_uv, max_uv);
		return ret;
	}

	return 0;
}

static void dw_mci_hi6220_set_ios(struct dw_mci *host, struct mmc_ios *ios)
{
	int ret;
	unsigned int clock;

	clock = (ios->clock <= 25000000) ? 25000000 : ios->clock;

	ret = clk_set_rate(host->biu_clk, clock);
	if (ret)
		dev_warn(host->dev, "failed to set rate %uHz\n", clock);

	host->bus_hz = clk_get_rate(host->biu_clk);
}

static int dw_mci_hi6220_execute_tuning(struct dw_mci_slot *slot, u32 opcode)
{
	return 0;
}

static const struct dw_mci_drv_data hi6220_data = {
	.caps			= dw_mci_hi6220_caps,
	.switch_voltage		= dw_mci_hi6220_switch_voltage,
	.set_ios		= dw_mci_hi6220_set_ios,
	.parse_dt		= dw_mci_hi6220_parse_dt,
	.execute_tuning		= dw_mci_hi6220_execute_tuning,
};

static const struct of_device_id dw_mci_k3_match[] = {
	{ .compatible = "hisilicon,hi4511-dw-mshc", .data = &k3_drv_data, },
	{ .compatible = "hisilicon,hi6220-dw-mshc", .data = &hi6220_data, },
	{},
};
MODULE_DEVICE_TABLE(of, dw_mci_k3_match);

static int dw_mci_k3_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;

	match = of_match_node(dw_mci_k3_match, pdev->dev.of_node);
	drv_data = match->data;

	return dw_mci_pltfm_register(pdev, drv_data);
}

#ifdef CONFIG_PM_SLEEP
static int dw_mci_k3_suspend(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret;

	ret = dw_mci_suspend(host);
	if (!ret)
		clk_disable_unprepare(host->ciu_clk);

	return ret;
}

static int dw_mci_k3_resume(struct device *dev)
{
	struct dw_mci *host = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(host->ciu_clk);
	if (ret) {
		dev_err(host->dev, "failed to enable ciu clock\n");
		return ret;
	}

	return dw_mci_resume(host);
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(dw_mci_k3_pmops, dw_mci_k3_suspend, dw_mci_k3_resume);

static struct platform_driver dw_mci_k3_pltfm_driver = {
	.probe		= dw_mci_k3_probe,
	.remove		= dw_mci_pltfm_remove,
	.driver		= {
		.name		= "dwmmc_k3",
		.of_match_table	= dw_mci_k3_match,
		.pm		= &dw_mci_k3_pmops,
	},
};

module_platform_driver(dw_mci_k3_pltfm_driver);

MODULE_DESCRIPTION("K3 Specific DW-MSHC Driver Extension");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dwmmc_k3");
