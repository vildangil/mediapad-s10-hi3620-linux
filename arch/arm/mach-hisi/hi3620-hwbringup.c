// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD S10-101x hardware bring-up helpers.
 *
 * Keep board-specific K3V2 power/reset sequences isolated here while the
 * individual blocks are being converted to proper clock/reset/regulator
 * drivers.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PMCTRL_PHYS             0xfca08000
#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_PMUSPI_PHYS             0xfcc00000
#define HI3620_G3D_PHYS                0xfa000000
#define HI3620_MAP_SIZE                0x1000

/* SYSCTRL clock/reset/power registers from Huawei K3V2. */
#define SCTRL_CLK_EN1                  0x030
#define SCTRL_CLK_STATUS1              0x03c
#define SCTRL_CLK_EN3                  0x050
#define SCTRL_CLK_STATUS3              0x05c
#define SCTRL_RST_EN1                  0x08c
#define SCTRL_RST_DIS1                 0x090
#define SCTRL_RST_STATUS1              0x094
#define SCTRL_RST_EN2                  0x098
#define SCTRL_RST_DIS2                 0x09c
#define SCTRL_RST_STATUS2              0x0a0
#define SCTRL_RST_EN3                  0x0a4
#define SCTRL_RST_DIS3                 0x0a8
#define SCTRL_RST_STATUS3              0x0ac
#define SCTRL_ISO_DIS                  0x0c4
#define SCTRL_ISO_STATUS               0x0c8
#define SCTRL_PWR_EN                   0x0d0
#define SCTRL_PWR_STATUS               0x0d8
#define SCTRL_PWR_ACK                  0x0dc

/* K3V2 PCTRL uses high-word write masks for the I2C SDA delay controls. */
#define PCTRL_PERI_CTRL0               0x000
#define I2C0_ENABLE_DELAY_SDA          0x00010001

/* DesignWare I2C reset bits from Huawei's K3V2 clock definitions. */
#define RST_I2C0                       BIT(24)
#define RST_I2C2                       BIT(28)

/* MMC3 is the MediaPad SDIO1 host used by BCM4330 Wi-Fi. */
#define CLK_MMC3                       BIT(23)
#define RST_MMC3                       BIT(26)

/* Vivante GC4000 / K3V2 G3D clocks, power domain and resets. */
#define CLK_G3D                        BIT(0)
#define CLK_DDRC_GPU                   BIT(4)
#define G3D_POWER_BIT                  BIT(7)
#define G3D_RST_CORE                   (BIT(0) | BIT(13) | BIT(14))
#define G3D_RST_INTF                   BIT(1)
#define G3D_RST_ALL                    (G3D_RST_CORE | G3D_RST_INTF)

/* Huawei programs all three G3D clock channels from the peripheral PLL. */
#define PMCTRL_G3D_AXI_DIV             0x044
#define PMCTRL_G3D_CORE_DIV            0x0b8
#define PMCTRL_G3D_SHADER_DIV          0x0bc
#define G3D_DIV_EN_VAL                 ((0x0b << 7) | 0x0b)

/* Vivante HI registers used by both the vendor galcore and Etnaviv. */
#define G3D_HI_CLOCK_CONTROL           0x000
#define G3D_HI_IDLE_STATE              0x004
#define G3D_HI_CHIP_IDENTITY           0x018
#define G3D_HI_CHIP_FEATURE            0x01c
#define G3D_HI_CHIP_MODEL              0x020
#define G3D_HI_CHIP_REV                0x024
#define G3D_HI_CHIP_DATE               0x028
#define G3D_HI_CHIP_TIME               0x02c
#define G3D_HI_CHIP_MINOR_FEATURE0     0x034

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

static void hi3620_wait_reset(void __iomem *sctrl, u32 status_reg,
                              u32 mask, bool asserted)
{
        unsigned int timeout = 50;

        while (timeout--) {
                bool state = !!(readl(sctrl + status_reg) & mask);

                if (state == asserted)
                        return;
                udelay(1);
        }
}

static void hi3620_mediapad_prepare_resets(void __iomem *sctrl,
                                           void __iomem *pctrl)
{
        u32 rst2_before = readl(sctrl + SCTRL_RST_STATUS2);
        u32 rst3_before = readl(sctrl + SCTRL_RST_STATUS3);
        u32 clk3_before = readl(sctrl + SCTRL_CLK_STATUS3);

        /* Huawei's K3V2 DesignWare-I2C glue always pulses the controller
         * reset before use. Merely clearing reset was not enough: BQ27510
         * registered but every FLAGS/data read failed and PRESENT stayed 0. */
        writel(RST_I2C0, sctrl + SCTRL_RST_EN2);
        mb();
        hi3620_wait_reset(sctrl, SCTRL_RST_STATUS2, RST_I2C0, true);
        udelay(1);
        writel(RST_I2C0, sctrl + SCTRL_RST_DIS2);
        mb();
        hi3620_wait_reset(sctrl, SCTRL_RST_STATUS2, RST_I2C0, false);

        /* Vendor common.c enables the I2C0 SDA input delay with this
         * high-word-masked PCTRL write before the controller is used. */
        writel(I2C0_ENABLE_DELAY_SDA, pctrl + PCTRL_PERI_CTRL0);
        mb();
        udelay(1);

        /* I2C2 remains bit-banged for the working touchscreen; just leave the
         * unused native block out of reset so it cannot surprise later tests. */
        writel(RST_I2C2, sctrl + SCTRL_RST_DIS2);

        /* Huawei's clk_mmc3 couples gate bit23 with external reset bit26.
         * Pulse that reset while the CIU clock is live before DW-MSHC tries
         * its own CTRL_RESET. */
        writel(RST_MMC3, sctrl + SCTRL_RST_EN3);
        mb();
        hi3620_wait_reset(sctrl, SCTRL_RST_STATUS3, RST_MMC3, true);
        writel(CLK_MMC3, sctrl + SCTRL_CLK_EN3);
        mb();
        udelay(10);
        writel(RST_MMC3, sctrl + SCTRL_RST_DIS3);
        mb();
        hi3620_wait_reset(sctrl, SCTRL_RST_STATUS3, RST_MMC3, false);
        udelay(10);

        pr_info("HI3620-HW-RESET: rst2 %08x->%08x i2c0=%u i2c2=%u pctrl0=%08x rst3 %08x->%08x mmc3_rst=%u clk3 %08x->%08x mmc3_clk=%u\n",
                rst2_before, readl(sctrl + SCTRL_RST_STATUS2),
                !!(readl(sctrl + SCTRL_RST_STATUS2) & RST_I2C0),
                !!(readl(sctrl + SCTRL_RST_STATUS2) & RST_I2C2),
                readl(pctrl + PCTRL_PERI_CTRL0),
                rst3_before, readl(sctrl + SCTRL_RST_STATUS3),
                !!(readl(sctrl + SCTRL_RST_STATUS3) & RST_MMC3),
                clk3_before, readl(sctrl + SCTRL_CLK_STATUS3),
                !!(readl(sctrl + SCTRL_CLK_STATUS3) & CLK_MMC3));
}

static void hi3620_mediapad_prepare_wifi(void __iomem *pmu)
{
        u8 ldo14_old, ldo15_old, clk32_old;

        ldo14_old = hi3620_pmu_ldo_enable(pmu, PMU_LDO14_CTRL,
                                          PMU_LDO14_2V85_VSEL);
        ldo15_old = hi3620_pmu_ldo_enable(pmu, PMU_LDO15_CTRL,
                                          PMU_LDO15_3V3_VSEL);

        clk32_old = readb(pmu + PMU_32K_EN);
        writeb(clk32_old | PMU_32KB_ENABLE, pmu + PMU_32K_EN);
        mb();
        udelay(100);

        pr_info("HI3620-WIFI-PWR: ldo14 %02x->%02x ldo15 %02x->%02x pmu32k %02x->%02x\n",
                ldo14_old, readb(pmu + PMU_LDO14_CTRL),
                ldo15_old, readb(pmu + PMU_LDO15_CTRL),
                clk32_old, readb(pmu + PMU_32K_EN));
}

static void hi3620_mediapad_dump_gpu(void __iomem *g3d)
{
        /* Keep this read-only.  The Etnaviv node is disabled while the raw
         * identity is zero, so a bad DRM node cannot poison Mesa/X.  These
         * offsets are identical in Huawei's galcore and Etnaviv state_hi. */
        pr_info("HI3620-GPU-ID: clock=%08x idle=%08x identity=%08x feature=%08x model=%08x rev=%08x date=%08x time=%08x minor0=%08x\n",
                readl(g3d + G3D_HI_CLOCK_CONTROL),
                readl(g3d + G3D_HI_IDLE_STATE),
                readl(g3d + G3D_HI_CHIP_IDENTITY),
                readl(g3d + G3D_HI_CHIP_FEATURE),
                readl(g3d + G3D_HI_CHIP_MODEL),
                readl(g3d + G3D_HI_CHIP_REV),
                readl(g3d + G3D_HI_CHIP_DATE),
                readl(g3d + G3D_HI_CHIP_TIME),
                readl(g3d + G3D_HI_CHIP_MINOR_FEATURE0));
}

static void hi3620_mediapad_prepare_gpu(void __iomem *sctrl,
                                        void __iomem *pmctrl,
                                        void __iomem *pmu,
                                        void __iomem *g3d)
{
        u8 buck2_ctrl_old = readb(pmu + PMU_BUCK2_CTRL);
        u8 buck2_vset_old = readb(pmu + PMU_BUCK2_VSET);
        u8 buck2_vset;
        u32 rst1_before = readl(sctrl + SCTRL_RST_STATUS1);
        u32 pwr_before = readl(sctrl + SCTRL_PWR_ACK);
        u32 iso_before = readl(sctrl + SCTRL_ISO_STATUS);

        /* HI6421 BUCK2: Huawei uses VSEL=56 for about 1.1 V. */
        buck2_vset = (buck2_vset_old & ~PMU_BUCK012_VSEL_MASK) |
                     PMU_BUCK2_1V1_VSEL;
        writeb(buck2_vset, pmu + PMU_BUCK2_VSET);
        writeb(buck2_ctrl_old | PMU_BUCK_ENABLE, pmu + PMU_BUCK2_CTRL);
        mb();
        udelay(300);

        /* Reproduce Huawei vcc_g3d enable sequence. */
        writel(G3D_POWER_BIT, sctrl + SCTRL_PWR_EN);
        mb();
        udelay(100);

        writel(G3D_RST_ALL, sctrl + SCTRL_RST_EN1);
        mb();
        udelay(5);

        writel(G3D_DIV_EN_VAL, pmctrl + PMCTRL_G3D_CORE_DIV);
        writel(G3D_DIV_EN_VAL, pmctrl + PMCTRL_G3D_SHADER_DIV);
        writel(G3D_DIV_EN_VAL, pmctrl + PMCTRL_G3D_AXI_DIV);
        writel(CLK_DDRC_GPU, sctrl + SCTRL_CLK_EN3);
        writel(CLK_G3D, sctrl + SCTRL_CLK_EN1);
        mb();
        udelay(10);

        writel(G3D_POWER_BIT, sctrl + SCTRL_ISO_DIS);
        mb();
        udelay(5);

        writel(G3D_RST_CORE, sctrl + SCTRL_RST_DIS1);
        mb();
        udelay(1);
        writel(G3D_RST_INTF, sctrl + SCTRL_RST_DIS1);
        mb();
        udelay(20);

        pr_info("HI3620-GPU-PWR: buck2 ctrl %02x->%02x vset %02x->%02x pwrack %08x->%08x iso %08x->%08x\n",
                buck2_ctrl_old, readb(pmu + PMU_BUCK2_CTRL),
                buck2_vset_old, readb(pmu + PMU_BUCK2_VSET),
                pwr_before, readl(sctrl + SCTRL_PWR_ACK),
                iso_before, readl(sctrl + SCTRL_ISO_STATUS));
        pr_info("HI3620-GPU-CLK: div core=%08x shader=%08x axi=%08x clk1=%08x clk3=%08x rst1=%08x->%08x\n",
                readl(pmctrl + PMCTRL_G3D_CORE_DIV),
                readl(pmctrl + PMCTRL_G3D_SHADER_DIV),
                readl(pmctrl + PMCTRL_G3D_AXI_DIV),
                readl(sctrl + SCTRL_CLK_STATUS1),
                readl(sctrl + SCTRL_CLK_STATUS3),
                rst1_before, readl(sctrl + SCTRL_RST_STATUS1));

        hi3620_mediapad_dump_gpu(g3d);
}

static int __init hi3620_mediapad_hwbringup(void)
{
        void __iomem *sctrl = NULL;
        void __iomem *pmctrl = NULL;
        void __iomem *pctrl = NULL;
        void __iomem *pmu = NULL;
        void __iomem *g3d = NULL;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        pmctrl = ioremap(HI3620_PMCTRL_PHYS, HI3620_MAP_SIZE);
        pctrl = ioremap(HI3620_PCTRL_PHYS, HI3620_MAP_SIZE);
        pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_MAP_SIZE);
        g3d = ioremap(HI3620_G3D_PHYS, HI3620_MAP_SIZE);
        if (!sctrl || !pmctrl || !pctrl || !pmu || !g3d) {
                pr_err("HI3620-HW: failed to map SCTRL/PMCTRL/PCTRL/PMUSPI/G3D\n");
                goto out;
        }

        pr_info("HI3620-HW: MediaPad hardware bring-up start\n");
        hi3620_mediapad_prepare_wifi(pmu);
        hi3620_mediapad_prepare_resets(sctrl, pctrl);
        hi3620_mediapad_prepare_gpu(sctrl, pmctrl, pmu, g3d);
        pr_info("HI3620-HW: MediaPad hardware bring-up complete\n");

out:
        if (g3d)
                iounmap(g3d);
        if (pmu)
                iounmap(pmu);
        if (pctrl)
                iounmap(pctrl);
        if (pmctrl)
                iounmap(pmctrl);
        if (sctrl)
                iounmap(sctrl);
        return 0;
}
arch_initcall(hi3620_mediapad_hwbringup);
