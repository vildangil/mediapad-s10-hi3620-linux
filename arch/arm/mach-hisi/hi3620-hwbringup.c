// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD S10-101x hardware bring-up helpers.
 *
 * The upstream Hi3620 clock description only models several clock gates and
 * misses reset/power sequencing that Huawei's K3V2 platform code performed.
 * Keep these board-specific writes isolated here while the individual blocks
 * are being converted to proper clock/reset/regulator drivers.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PMCTRL_PHYS             0xfca08000
#define HI3620_PMUSPI_PHYS             0xfcc00000
#define HI3620_MAP_SIZE                0x1000

/* SYSCTRL clock/reset registers, matching Huawei K3V2 clock.h. */
#define SCTRL_CLK_EN1                  0x030
#define SCTRL_CLK_STATUS1              0x038
#define SCTRL_CLK_EN3                  0x050
#define SCTRL_CLK_STATUS3              0x058
#define SCTRL_RST_DIS1                 0x090
#define SCTRL_RST_STATUS1              0x094
#define SCTRL_RST_DIS2                 0x09c
#define SCTRL_RST_STATUS2              0x0a0
#define SCTRL_RST_DIS3                 0x0a8
#define SCTRL_RST_STATUS3              0x0ac

/* DesignWare I2C reset bits from Huawei's K3V2 clock definitions. */
#define RST_I2C0                       BIT(24)
#define RST_I2C2                       BIT(28)

/* MMC3 is the MediaPad SDIO1 host used by BCM4330 Wi-Fi. */
#define RST_MMC3                       BIT(26)

/* Vivante GC4000 / K3V2 G3D clocks and reset. */
#define CLK_G3D                        BIT(0)
#define CLK_DDRC_GPU                   BIT(4)
#define RST_G3D                        BIT(14)

/* Huawei programs all three G3D clock channels from the peripheral PLL. */
#define PMCTRL_G3D_AXI_DIV             0x044
#define PMCTRL_G3D_CORE_DIV            0x0b8
#define PMCTRL_G3D_SHADER_DIV          0x0bc
#define G3D_DIV_EN_VAL                 ((0x0b << 7) | 0x0b)

/* HI6421V200 PMU registers are byte-wide on a 32-bit-spaced PMUSPI window. */
#define PMU_BUCK2_CTRL                 (0x10 << 2)
#define PMU_BUCK2_VSET                 (0x11 << 2)
#define PMU_LDO14_CTRL                 (0x2e << 2)
#define PMU_LDO15_CTRL                 (0x2f << 2)
#define PMU_32K_EN                     (0x50 << 2)

#define PMU_BUCK_ENABLE                BIT(0)
#define PMU_BUCK012_VSEL_MASK          0x7f
#define PMU_BUCK2_1V1_VSEL             56
#define PMU_LDO_ENABLE                 0x10
#define PMU_LDO_VSEL_MASK              0x07
#define PMU_LDO14_2V85_VSEL            6
#define PMU_LDO15_3V3_VSEL             7
#define PMU_32KB_ENABLE                BIT(1)

static u8 hi3620_pmu_ldo_enable(void __iomem *pmu, unsigned int reg,
                                u8 vsel)
{
        u8 old = readb(pmu + reg);
        u8 val = (old & ~(PMU_LDO_ENABLE | PMU_LDO_VSEL_MASK)) |
                 PMU_LDO_ENABLE | vsel;

        writeb(val, pmu + reg);
        mb();
        udelay(250);
        return old;
}

static void hi3620_mediapad_prepare_resets(void __iomem *sctrl)
{
        u32 rst2_before = readl(sctrl + SCTRL_RST_STATUS2);
        u32 rst3_before = readl(sctrl + SCTRL_RST_STATUS3);

        /* Native I2C0 is needed by the BQ27510 fuel gauge.  I2C2 is kept on
         * i2c-gpio for this image, but release its hardware master from reset
         * as a diagnostic for the next touchscreen-native-I2C test. */
        writel(RST_I2C0 | RST_I2C2, sctrl + SCTRL_RST_DIS2);

        /* Upstream clk-hi3620 gates MMC3 but never releases this reset bit. */
        writel(RST_MMC3, sctrl + SCTRL_RST_DIS3);
        mb();
        udelay(10);

        pr_info("HI3620-HW-RESET: rst2 %08x->%08x (i2c0=%u i2c2=%u) rst3 %08x->%08x (mmc3=%u)\n",
                rst2_before, readl(sctrl + SCTRL_RST_STATUS2),
                !!(readl(sctrl + SCTRL_RST_STATUS2) & RST_I2C0),
                !!(readl(sctrl + SCTRL_RST_STATUS2) & RST_I2C2),
                rst3_before, readl(sctrl + SCTRL_RST_STATUS3),
                !!(readl(sctrl + SCTRL_RST_STATUS3) & RST_MMC3));
}

static void hi3620_mediapad_prepare_wifi(void __iomem *pmu)
{
        u8 ldo14_old, ldo15_old, clk32_old;

        /* MediaPad 10 FHD service data assigns Wi-Fi rails to HI6421 LDO14
         * (2.85 V) and LDO15 (3.3 V).  Program the documented voltages before
         * mmc-pwrseq-simple releases BCM4330 WL_REG_ON. */
        ldo14_old = hi3620_pmu_ldo_enable(pmu, PMU_LDO14_CTRL,
                                          PMU_LDO14_2V85_VSEL);
        ldo15_old = hi3620_pmu_ldo_enable(pmu, PMU_LDO15_CTRL,
                                          PMU_LDO15_3V3_VSEL);

        /* K3V2's clk_pmu32kb is PMU register 0x50 bit 1. BCM4330 needs this
         * sleep clock even though the SDIO bus itself is clocked by MMC3. */
        clk32_old = readb(pmu + PMU_32K_EN);
        writeb(clk32_old | PMU_32KB_ENABLE, pmu + PMU_32K_EN);
        mb();
        udelay(100);

        pr_info("HI3620-WIFI-PWR: ldo14 %02x->%02x ldo15 %02x->%02x pmu32k %02x->%02x\n",
                ldo14_old, readb(pmu + PMU_LDO14_CTRL),
                ldo15_old, readb(pmu + PMU_LDO15_CTRL),
                clk32_old, readb(pmu + PMU_32K_EN));
}

static void hi3620_mediapad_prepare_gpu(void __iomem *sctrl,
                                        void __iomem *pmctrl,
                                        void __iomem *pmu)
{
        u8 buck2_ctrl_old = readb(pmu + PMU_BUCK2_CTRL);
        u8 buck2_vset_old = readb(pmu + PMU_BUCK2_VSET);
        u8 buck2_vset;
        u32 rst1_before = readl(sctrl + SCTRL_RST_STATUS1);

        /* The tablet schematic/service manual powers G3D from HI6421 BUCK2.
         * Huawei's regulator driver uses VSEL 56 for ~1.1 V and bit0 as EN. */
        buck2_vset = (buck2_vset_old & ~PMU_BUCK012_VSEL_MASK) |
                     PMU_BUCK2_1V1_VSEL;
        writeb(buck2_vset, pmu + PMU_BUCK2_VSET);
        writeb(buck2_ctrl_old | PMU_BUCK_ENABLE, pmu + PMU_BUCK2_CTRL);
        mb();
        udelay(250);

        /* Reproduce k3v2_g3d_clk_enable(): select/divide peripheral PLL for
         * core, shader and AXI.  The DDRC_GPU friend clock must be on first. */
        writel(G3D_DIV_EN_VAL, pmctrl + PMCTRL_G3D_CORE_DIV);
        writel(G3D_DIV_EN_VAL, pmctrl + PMCTRL_G3D_SHADER_DIV);
        writel(G3D_DIV_EN_VAL, pmctrl + PMCTRL_G3D_AXI_DIV);
        writel(CLK_DDRC_GPU, sctrl + SCTRL_CLK_EN3);
        writel(CLK_G3D, sctrl + SCTRL_CLK_EN1);
        mb();
        udelay(10);

        writel(RST_G3D, sctrl + SCTRL_RST_DIS1);
        mb();
        udelay(20);

        pr_info("HI3620-GPU-PWR: buck2 ctrl %02x->%02x vset %02x->%02x (1.1V code=%u)\n",
                buck2_ctrl_old, readb(pmu + PMU_BUCK2_CTRL),
                buck2_vset_old, readb(pmu + PMU_BUCK2_VSET),
                PMU_BUCK2_1V1_VSEL);
        pr_info("HI3620-GPU-CLK: div core=%08x shader=%08x axi=%08x clk1=%08x clk3=%08x rst1=%08x->%08x\n",
                readl(pmctrl + PMCTRL_G3D_CORE_DIV),
                readl(pmctrl + PMCTRL_G3D_SHADER_DIV),
                readl(pmctrl + PMCTRL_G3D_AXI_DIV),
                readl(sctrl + SCTRL_CLK_STATUS1),
                readl(sctrl + SCTRL_CLK_STATUS3),
                rst1_before, readl(sctrl + SCTRL_RST_STATUS1));
}

static int __init hi3620_mediapad_hwbringup(void)
{
        void __iomem *sctrl = NULL;
        void __iomem *pmctrl = NULL;
        void __iomem *pmu = NULL;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        pmctrl = ioremap(HI3620_PMCTRL_PHYS, HI3620_MAP_SIZE);
        pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_MAP_SIZE);
        if (!sctrl || !pmctrl || !pmu) {
                pr_err("HI3620-HW: failed to map SCTRL/PMCTRL/PMUSPI\n");
                goto out;
        }

        pr_info("HI3620-HW: MediaPad hardware bring-up start\n");
        hi3620_mediapad_prepare_wifi(pmu);
        hi3620_mediapad_prepare_resets(sctrl);
        hi3620_mediapad_prepare_gpu(sctrl, pmctrl, pmu);
        pr_info("HI3620-HW: MediaPad hardware bring-up complete\n");

out:
        if (pmu)
                iounmap(pmu);
        if (pmctrl)
                iounmap(pmctrl);
        if (sctrl)
                iounmap(sctrl);
        return 0;
}
arch_initcall(hi3620_mediapad_hwbringup);
