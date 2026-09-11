// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD external GPU DCDC bring-up.
 *
 * Huawei's S10 vendor kernel enables CONFIG_EXTRAL_DYNAMIC_DCDC and places a
 * TPS6236x at I2C3 address 0x60.  Mainline has no I2C3 DT node/clock consumer,
 * so keep its physical SCTRL gate live before DesignWare probes the new node.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_MAP_SIZE                0x1000
#define SCTRL_CLK_EN2                  0x040
#define SCTRL_CLK_STATUS2              0x04c
#define CLK_I2C3                       BIT(29)

static int __init hi3620_s10_gpu_dcdc_clock(void)
{
        void __iomem *sctrl;
        u32 before;
        u32 after;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        if (!sctrl)
                return 0;

        before = readl(sctrl + SCTRL_CLK_STATUS2);
        writel(CLK_I2C3, sctrl + SCTRL_CLK_EN2);
        mb();
        udelay(10);
        after = readl(sctrl + SCTRL_CLK_STATUS2);

        pr_info("HI3620-GPU-DCDC: I2C3 gate clk2=%08x->%08x enabled=%u\n",
                before, after, !!(after & CLK_I2C3));

        iounmap(sctrl);
        return 0;
}
arch_initcall(hi3620_s10_gpu_dcdc_clock);
