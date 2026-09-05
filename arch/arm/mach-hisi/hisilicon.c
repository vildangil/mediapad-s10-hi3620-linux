/*
 * (Hisilicon's SoC based) flattened device tree enabled machine
 *
 * Copyright (c) 2012-2013 Hisilicon Ltd.
 * Copyright (c) 2012-2013 Linaro Ltd.
 *
 * Author: Haojian Zhuang <haojian.zhuang@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/clocksource.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irqchip.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#define HI3620_SYSCTRL_PHYS_BASE		0xfc802000
#define HI3620_SYSCTRL_VIRT_BASE		0xfe802000
#define HI3620_BOOTTRACE_PHYS_BASE		0x1ff00000
#define HI3620_BOOTTRACE_VIRT_BASE		0xfe803000

/*
 * Stock K3V2 maps EDC0 at 0xfa202000.  Keep this mapping strictly
 * read-only during framebuffer bring-up so we can snapshot the scanout state
 * left behind by fastboot without perturbing clocks, MIPI or the panel.
 */
#define HI3620_EDC0_PHYS_BASE			0xfa202000
#define HI3620_EDC0_VIRT_BASE			0xfe804000
#define HI3620_EDC0_MAP_SIZE			0x2000

#define EDC_ID_OFFSET				0x000
#define EDC_CH1_ADDR_OFFSET			0x004
#define EDC_CH1_STRIDE_OFFSET			0x00c
#define EDC_CH1_SIZE_OFFSET			0x014
#define EDC_CH1_CTL_OFFSET			0x018
#define EDC_CH2_ADDR_OFFSET			0x024
#define EDC_CH2_STRIDE_OFFSET			0x02c
#define EDC_CH2_SIZE_OFFSET			0x034
#define EDC_CH2_CTL_OFFSET			0x038
#define EDC_DISP_SIZE_OFFSET			0x090
#define EDC_DISP_CTL_OFFSET			0x094
#define EDC_STS_OFFSET				0x09c
#define EDC_INTE_OFFSET				0x0a4

#define LDI_HRZ_CTRL0_OFFSET			0x800
#define LDI_HRZ_CTRL1_OFFSET			0x804
#define LDI_VRT_CTRL0_OFFSET			0x808
#define LDI_VRT_CTRL1_OFFSET			0x80c
#define LDI_DSP_SIZE_OFFSET			0x814
#define LDI_CTRL_OFFSET				0x820
#define LDI_WORK_MODE_OFFSET			0x830

#define MIPI_VERSION_OFFSET			0x900
#define MIPI_PWR_UP_OFFSET			0x904
#define MIPI_DPI_CFG_OFFSET			0x90c
#define MIPI_VID_MODE_CFG_OFFSET		0x91c
#define MIPI_VID_PKT_CFG_OFFSET		0x920
#define MIPI_PHY_RSTZ_OFFSET			0x954
#define MIPI_PHY_IF_CFG_OFFSET			0x958
#define MIPI_PHY_STATUS_OFFSET			0x960

static inline void hi3620_boottrace(unsigned int slot, unsigned int value)
{
	volatile unsigned int *trace =
		(volatile unsigned int *)HI3620_BOOTTRACE_VIRT_BASE;

	trace[0] = 0x43525442; /* "BTRC" in little endian memory. */
	trace[slot] = value;
	asm volatile("dsb sy" : : : "memory");
}

static inline u32 hi3620_edc0_read(unsigned int offset)
{
	return readl_relaxed((void __iomem *)(HI3620_EDC0_VIRT_BASE + offset));
}

static void hi3620_dump_boot_display(const char *stage)
{
	pr_info("HI3620-DISP[%s]: edc id=%08x disp_size=%08x disp_ctl=%08x sts=%08x inte=%08x\n",
		stage,
		hi3620_edc0_read(EDC_ID_OFFSET),
		hi3620_edc0_read(EDC_DISP_SIZE_OFFSET),
		hi3620_edc0_read(EDC_DISP_CTL_OFFSET),
		hi3620_edc0_read(EDC_STS_OFFSET),
		hi3620_edc0_read(EDC_INTE_OFFSET));
	pr_info("HI3620-DISP[%s]: ch1 addr=%08x stride=%08x size=%08x ctl=%08x ch2 addr=%08x stride=%08x size=%08x ctl=%08x\n",
		stage,
		hi3620_edc0_read(EDC_CH1_ADDR_OFFSET),
		hi3620_edc0_read(EDC_CH1_STRIDE_OFFSET),
		hi3620_edc0_read(EDC_CH1_SIZE_OFFSET),
		hi3620_edc0_read(EDC_CH1_CTL_OFFSET),
		hi3620_edc0_read(EDC_CH2_ADDR_OFFSET),
		hi3620_edc0_read(EDC_CH2_STRIDE_OFFSET),
		hi3620_edc0_read(EDC_CH2_SIZE_OFFSET),
		hi3620_edc0_read(EDC_CH2_CTL_OFFSET));
	pr_info("HI3620-DISP[%s]: ldi hrz0=%08x hrz1=%08x vrt0=%08x vrt1=%08x dsp=%08x ctrl=%08x work=%08x\n",
		stage,
		hi3620_edc0_read(LDI_HRZ_CTRL0_OFFSET),
		hi3620_edc0_read(LDI_HRZ_CTRL1_OFFSET),
		hi3620_edc0_read(LDI_VRT_CTRL0_OFFSET),
		hi3620_edc0_read(LDI_VRT_CTRL1_OFFSET),
		hi3620_edc0_read(LDI_DSP_SIZE_OFFSET),
		hi3620_edc0_read(LDI_CTRL_OFFSET),
		hi3620_edc0_read(LDI_WORK_MODE_OFFSET));
	pr_info("HI3620-DISP[%s]: mipi ver=%08x pwr=%08x dpi=%08x vid=%08x pkt=%08x phy_rst=%08x phy_if=%08x phy_sts=%08x\n",
		stage,
		hi3620_edc0_read(MIPI_VERSION_OFFSET),
		hi3620_edc0_read(MIPI_PWR_UP_OFFSET),
		hi3620_edc0_read(MIPI_DPI_CFG_OFFSET),
		hi3620_edc0_read(MIPI_VID_MODE_CFG_OFFSET),
		hi3620_edc0_read(MIPI_VID_PKT_CFG_OFFSET),
		hi3620_edc0_read(MIPI_PHY_RSTZ_OFFSET),
		hi3620_edc0_read(MIPI_PHY_IF_CFG_OFFSET),
		hi3620_edc0_read(MIPI_PHY_STATUS_OFFSET));
}

/*
 * This table is only for optimization. Since ioremap() could always share
 * the same mapping if it's defined as static IO mapping.
 *
 * Without this table, system could also work. The cost is some virtual address
 * spaces wasted since ioremap() may be called multi times for the same
 * IO space.
 */
static struct map_desc hi3620_io_desc[] __initdata = {
	{
		/* sysctrl */
		.pfn		= __phys_to_pfn(HI3620_SYSCTRL_PHYS_BASE),
		.virtual	= HI3620_SYSCTRL_VIRT_BASE,
		.length		= 0x1000,
		.type		= MT_DEVICE,
	},
	{
		/* Persistent early-boot breadcrumb page. */
		.pfn		= __phys_to_pfn(HI3620_BOOTTRACE_PHYS_BASE),
		.virtual	= HI3620_BOOTTRACE_VIRT_BASE,
		.length		= 0x1000,
		.type		= MT_DEVICE,
	},
	{
		/* Read-only diagnostics for the bootloader-programmed display. */
		.pfn		= __phys_to_pfn(HI3620_EDC0_PHYS_BASE),
		.virtual	= HI3620_EDC0_VIRT_BASE,
		.length		= HI3620_EDC0_MAP_SIZE,
		.type		= MT_DEVICE,
	},
};

static void __init hi3620_map_io(void)
{
	debug_ll_io_init();
	iotable_init(hi3620_io_desc, ARRAY_SIZE(hi3620_io_desc));
	hi3620_boottrace(4, 0xa004a004);
}

static void __init hi3620_init_early(void)
{
	hi3620_boottrace(5, 0xa005a005);
	hi3620_dump_boot_display("early");
}

static int __init hi3620_trace_pure(void)
{
	hi3620_boottrace(6, 0xa006a006);
	return 0;
}
pure_initcall(hi3620_trace_pure);

static int __init hi3620_trace_core(void)
{
	hi3620_boottrace(7, 0xa007a007);
	return 0;
}
core_initcall(hi3620_trace_core);

static int __init hi3620_trace_postcore(void)
{
	hi3620_boottrace(10, 0xa00aa00a);
	return 0;
}
postcore_initcall(hi3620_trace_postcore);

static int __init hi3620_trace_arch(void)
{
	hi3620_boottrace(8, 0xa008a008);
	hi3620_dump_boot_display("arch");
	return 0;
}
arch_initcall(hi3620_trace_arch);

static int __init hi3620_trace_subsys(void)
{
	hi3620_boottrace(11, 0xa00ba00b);
	return 0;
}
subsys_initcall(hi3620_trace_subsys);

static int __init hi3620_trace_fs(void)
{
	hi3620_boottrace(12, 0xa00ca00c);
	return 0;
}
fs_initcall(hi3620_trace_fs);

static int __init hi3620_trace_rootfs(void)
{
	hi3620_boottrace(13, 0xa00da00d);
	return 0;
}
rootfs_initcall(hi3620_trace_rootfs);

static int __init hi3620_trace_device(void)
{
	hi3620_boottrace(9, 0xa009a009);
	hi3620_dump_boot_display("device");
	return 0;
}
device_initcall(hi3620_trace_device);

static int __init hi3620_trace_late(void)
{
	hi3620_boottrace(14, 0xa00ea00e);
	hi3620_dump_boot_display("late");
	return 0;
}
late_initcall(hi3620_trace_late);

static const char *const hi3xxx_compat[] __initconst = {
	"hisilicon,hi3620-hi4511",
	"hisilicon,hi3620",
	NULL,
};

DT_MACHINE_START(HI3620, "Hisilicon Hi3620 (Flattened Device Tree)")
	.map_io		= hi3620_map_io,
	.init_early	= hi3620_init_early,
	.dt_compat	= hi3xxx_compat,
MACHINE_END
