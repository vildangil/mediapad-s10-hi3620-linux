// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD S10-101x second-stage hardware bring-up.
 *
 * Keep these experiments board-local.  The eMMC controller, working simplefb
 * display path and the not-yet-ported ASP audio block are deliberately not
 * touched here.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PCTRL_PHYS              0xfca09000
#define HI3620_G3D_PHYS                0xfa000000
#define HI3620_MAP_SIZE                0x1000

/* SCTRL clock/reset registers. */
#define SCTRL_CLK_EN1                  0x030
#define SCTRL_CLK_STATUS1              0x03c
#define SCTRL_CLK_EN2                  0x040
#define SCTRL_CLK_STATUS2              0x04c
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
#define SCTRL_DIV_REG0                 0x100
#define SCTRL_DIV_REG2                 0x108
#define SCTRL_DIV_REG14                0x140

/* PCTRL0 uses the same high-word write-mask convention as the other K3V2
 * peripheral control registers. Huawei's board code enables a 300 ns SDA
 * delay for I2C0 with the exact value 0x00010001 before using the BQ27510.
 * This only touches bit 0 and leaves the USB/PCTRL fields intact.
 */
#define PCTRL_PERI_CTRL0               0x000
#define I2C0_ENABLE_DELAY_SDA          0x00010001

/* Huawei CS clock tree: select PLL2 for CFGAXI, then divide 1440 MHz by
 * 15 * 2 = 30 => 48 MHz.  These are the exact high-word-mask writes used by
 * k3v2_cfgaxi_clk_enable() in the vendor kernel.  The old mainline Hi3620 CCF
 * describes rclk_cfgaxi as 48 MHz but never programs these hardware bits; the
 * boot log showed 0x8024 instead of the vendor 0x802e state.
 */
#define CFGAXI_SEL_VALUE               0x80008000
#define CFGAXI_NORMAL_DIV_VALUE        0x007f002e

#define CLK_G3D                        BIT(0)
#define CLK_DDRC_GPU                   BIT(4)
#define RST_G3D_CORE                   (BIT(0) | BIT(13) | BIT(14))
#define RST_G3D_INTF                   BIT(1)
#define RST_G3D_ALL                    (RST_G3D_CORE | RST_G3D_INTF)

#define CLK_I2C0                       BIT(24)
#define RST_I2C0                       BIT(24)

#define CLK_DDRC_PER                   BIT(9)
#define CLK_SD                         BIT(20)
#define RST_SD                         BIT(23)
#define CLK_MMC3                       BIT(23)
#define RST_MMC3                       BIT(26)

/* DIV_REG2: SD divider bits 3:0, source mux bit 4.  Huawei clk_sd_cs uses
 * PLL23_16DIV with PLL2 selector 0 and divider 15 for PLL2 / 16 = 90 MHz.
 */
#define SD_DIV_MUX_MASK                GENMASK(4, 0)
#define SD_PLL2_DIV16_VALUE            15
#define SD_PLL2_DIV16_WRITE            \
        ((SD_DIV_MUX_MASK << 16) | SD_PLL2_DIV16_VALUE)

/* DIV_REG14: MMC3 divider is bits 8:5; source mux is bit 9.
 * Hi3620 divider/source registers use the upper 16 bits as write enables.
 * PLL2 / 16 is the vendor clock's low/bring-up setting: divider field 15,
 * source field 0 (PLL2).
 */
#define MMC3_DIV_MUX_MASK              GENMASK(9, 5)
#define MMC3_PLL2_DIV16_VALUE          (15 << 5)
#define MMC3_PLL2_DIV16_WRITE          \
        ((MMC3_DIV_MUX_MASK << 16) | MMC3_PLL2_DIV16_VALUE)

/* Vivante registers. Huawei galcore writes 0x900 to CLOCK_CONTROL before
 * _IdentifyHardware(); Linux 4.9 Etnaviv identifies first and therefore never
 * reaches etnaviv_hw_reset() when the power-on value reads as zero.
 */
#define G3D_HI_CLOCK_CONTROL           0x000
#define G3D_HI_IDLE_STATE              0x004
#define G3D_HI_CHIP_IDENTITY           0x018
#define G3D_HI_CHIP_FEATURE            0x01c
#define G3D_HI_CHIP_MODEL              0x020
#define G3D_HI_CHIP_REV                0x024
#define G3D_VENDOR_WAKE                0x00000900

static void hi3620_hwfix2_cfgaxi(void __iomem *sctrl)
{
        u32 before = readl(sctrl + SCTRL_DIV_REG0);
        u32 after;

        writel(CFGAXI_SEL_VALUE, sctrl + SCTRL_DIV_REG0);
        mb();
        writel(CFGAXI_NORMAL_DIV_VALUE, sctrl + SCTRL_DIV_REG0);
        mb();
        udelay(5);
        after = readl(sctrl + SCTRL_DIV_REG0);

        pr_info("HI3620-HWFIX2-CFGAXI: div0=%08x->%08x expected_low=0000802e\n",
                before, after);
}

static void hi3620_hwfix2_i2c0_delay(void __iomem *pctrl)
{
        u32 before = readl(pctrl + PCTRL_PERI_CTRL0);
        u32 after;

        writel(I2C0_ENABLE_DELAY_SDA, pctrl + PCTRL_PERI_CTRL0);
        mb();
        udelay(2);
        after = readl(pctrl + PCTRL_PERI_CTRL0);

        pr_info("HI3620-HWFIX2-I2C-SDA: pctrl0=%08x->%08x delay_bit=%u\n",
                before, after, !!(after & BIT(0)));
}

static void hi3620_hwfix2_i2c0(void __iomem *sctrl)
{
        u32 clk_before = readl(sctrl + SCTRL_CLK_STATUS2);
        u32 rst_before = readl(sctrl + SCTRL_RST_STATUS2);

        writel(CLK_I2C0, sctrl + SCTRL_CLK_EN2);
        mb();
        udelay(2);

        /* Re-pulse reset after CFGAXI is at the real vendor 48 MHz rate. */
        writel(RST_I2C0, sctrl + SCTRL_RST_EN2);
        mb();
        udelay(2);
        writel(RST_I2C0, sctrl + SCTRL_RST_DIS2);
        mb();
        udelay(10);

        pr_info("HI3620-HWFIX2-I2C: clk2=%08x->%08x rst2=%08x->%08x i2c0_clk=%u i2c0_rst=%u\n",
                clk_before, readl(sctrl + SCTRL_CLK_STATUS2),
                rst_before, readl(sctrl + SCTRL_RST_STATUS2),
                !!(readl(sctrl + SCTRL_CLK_STATUS2) & CLK_I2C0),
                !!(readl(sctrl + SCTRL_RST_STATUS2) & RST_I2C0));
}

static void hi3620_hwfix2_sd(void __iomem *sctrl)
{
        u32 div_before = readl(sctrl + SCTRL_DIV_REG2);
        u32 clk_before = readl(sctrl + SCTRL_CLK_STATUS3);
        u32 rst_before = readl(sctrl + SCTRL_RST_STATUS3);

        writel(SD_PLL2_DIV16_WRITE, sctrl + SCTRL_DIV_REG2);
        mb();
        writel(CLK_DDRC_PER | CLK_SD, sctrl + SCTRL_CLK_EN3);
        mb();
        udelay(10);
        writel(RST_SD, sctrl + SCTRL_RST_EN3);
        mb();
        udelay(10);
        writel(RST_SD, sctrl + SCTRL_RST_DIS3);
        mb();
        udelay(20);

        pr_info("HI3620-HWFIX2-SD: div2=%08x->%08x clk3=%08x->%08x rst3=%08x->%08x ddrc_per=%u sd=%u rst=%u\n",
                div_before, readl(sctrl + SCTRL_DIV_REG2),
                clk_before, readl(sctrl + SCTRL_CLK_STATUS3),
                rst_before, readl(sctrl + SCTRL_RST_STATUS3),
                !!(readl(sctrl + SCTRL_CLK_STATUS3) & CLK_DDRC_PER),
                !!(readl(sctrl + SCTRL_CLK_STATUS3) & CLK_SD),
                !!(readl(sctrl + SCTRL_RST_STATUS3) & RST_SD));
}

static void hi3620_hwfix2_mmc3(void __iomem *sctrl)
{
        u32 div_before = readl(sctrl + SCTRL_DIV_REG14);
        u32 clk_before = readl(sctrl + SCTRL_CLK_STATUS3);
        u32 rst_before = readl(sctrl + SCTRL_RST_STATUS3);

        /* Only MMC3/SDIO is changed. MMC1/eMMC lives in DIV_REG2 bits 5+
         * and is left exactly as the bootloader/known-good kernel configured it.
         */
        writel(MMC3_PLL2_DIV16_WRITE, sctrl + SCTRL_DIV_REG14);
        mb();
        udelay(2);

        writel(CLK_DDRC_PER | CLK_MMC3, sctrl + SCTRL_CLK_EN3);
        mb();
        udelay(10);

        writel(RST_MMC3, sctrl + SCTRL_RST_EN3);
        mb();
        udelay(10);
        writel(RST_MMC3, sctrl + SCTRL_RST_DIS3);
        mb();
        udelay(20);

        pr_info("HI3620-HWFIX2-MMC3: div14=%08x->%08x clk3=%08x->%08x rst3=%08x->%08x ddrc_per=%u mmc3=%u rst=%u\n",
                div_before, readl(sctrl + SCTRL_DIV_REG14),
                clk_before, readl(sctrl + SCTRL_CLK_STATUS3),
                rst_before, readl(sctrl + SCTRL_RST_STATUS3),
                !!(readl(sctrl + SCTRL_CLK_STATUS3) & CLK_DDRC_PER),
                !!(readl(sctrl + SCTRL_CLK_STATUS3) & CLK_MMC3),
                !!(readl(sctrl + SCTRL_RST_STATUS3) & RST_MMC3));
}

static void hi3620_hwfix2_gpu(void __iomem *sctrl, void __iomem *g3d)
{
        u32 model_before;
        u32 clock_before;
        u32 model;
        unsigned int poll;

        /* CFGAXI changed after the first-stage Huawei power sequence.  Re-sync
         * the G3D AXI interface using the exact vendor reset ordering while
         * keeping the already-enabled MTCMOS/isolation state intact.
         */
        writel(CLK_G3D, sctrl + SCTRL_CLK_EN1);
        writel(CLK_DDRC_GPU, sctrl + SCTRL_CLK_EN3);
        mb();
        writel(RST_G3D_ALL, sctrl + SCTRL_RST_EN1);
        mb();
        udelay(5);
        writel(RST_G3D_CORE, sctrl + SCTRL_RST_DIS1);
        mb();
        udelay(1);
        writel(RST_G3D_INTF, sctrl + SCTRL_RST_DIS1);
        mb();
        udelay(20);

        model_before = readl(g3d + G3D_HI_CHIP_MODEL);
        clock_before = readl(g3d + G3D_HI_CLOCK_CONTROL);
        model = model_before;

        writel(G3D_VENDOR_WAKE, g3d + G3D_HI_CLOCK_CONTROL);
        mb();

        for (poll = 0; poll < 100; poll++) {
                udelay(10);
                model = readl(g3d + G3D_HI_CHIP_MODEL);
                if (model)
                        break;
        }

        pr_info("HI3620-HWFIX2-GPU: cfgaxi_div0=%08x clk1=%08x clk3=%08x rst1=%08x clock=%08x->%08x model=%08x->%08x rev=%08x identity=%08x feature=%08x idle=%08x polls=%u\n",
                readl(sctrl + SCTRL_DIV_REG0),
                readl(sctrl + SCTRL_CLK_STATUS1),
                readl(sctrl + SCTRL_CLK_STATUS3),
                readl(sctrl + SCTRL_RST_STATUS1),
                clock_before, readl(g3d + G3D_HI_CLOCK_CONTROL),
                model_before, model,
                readl(g3d + G3D_HI_CHIP_REV),
                readl(g3d + G3D_HI_CHIP_IDENTITY),
                readl(g3d + G3D_HI_CHIP_FEATURE),
                readl(g3d + G3D_HI_IDLE_STATE),
                poll < 100 ? poll + 1 : 100);
}

static int __init hi3620_mediapad_hwfix2(void)
{
        void __iomem *sctrl;
        void __iomem *pctrl;
        void __iomem *g3d;

        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_MAP_SIZE);
        pctrl = ioremap(HI3620_PCTRL_PHYS, HI3620_MAP_SIZE);
        g3d = ioremap(HI3620_G3D_PHYS, HI3620_MAP_SIZE);
        if (!sctrl || !g3d) {
                pr_err("HI3620-HWFIX2: failed to map SCTRL/G3D\n");
                if (g3d)
                        iounmap(g3d);
                if (pctrl)
                        iounmap(pctrl);
                if (sctrl)
                        iounmap(sctrl);
                return 0;
        }

        pr_info("HI3620-HWFIX2: second-stage bring-up start\n");
        hi3620_hwfix2_cfgaxi(sctrl);
        if (pctrl)
                hi3620_hwfix2_i2c0_delay(pctrl);
        else
                pr_warn("HI3620-HWFIX2-I2C-SDA: failed to map PCTRL\n");
        hi3620_hwfix2_i2c0(sctrl);
        hi3620_hwfix2_sd(sctrl);
        hi3620_hwfix2_mmc3(sctrl);
        hi3620_hwfix2_gpu(sctrl, g3d);
        pr_info("HI3620-HWFIX2: second-stage bring-up done\n");

        iounmap(g3d);
        if (pctrl)
                iounmap(pctrl);
        iounmap(sctrl);
        return 0;
}
subsys_initcall(hi3620_mediapad_hwfix2);
