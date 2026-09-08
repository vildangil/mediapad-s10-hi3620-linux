/*
 * Copyright (c) 2013 Linaro Ltd.
 * Copyright (c) 2013 Hisilicon Limited.
 * Based on arch/arm/mach-vexpress/platsmp.c, Copyright (C) 2002 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>
#include <asm/mach/map.h>

#include "core.h"

#define HIX5HD2_BOOT_ADDRESS		0xffff0000
#define K3V2_BOOTMON_ADDR_OFFSET	0x314

static void __iomem *ctrl_base;
static void __iomem *hi3620_sysctrl_base;
static bool hi3620_use_k3v2_bootmon;
static DEFINE_RAW_SPINLOCK(hi3620_boot_lock);

/* Read by hi3620-k3v2-headsmp.S before the secondary CPU enables its MMU. */
volatile int hi3620_pen_release = -1;
extern void hi3620_k3v2_secondary_startup(void);

static void hi3620_write_pen_release(int val)
{
	WRITE_ONCE(hi3620_pen_release, val);
	smp_wmb();
	/* The secondary is not coherent while it sits in the BootMonitor pen. */
	sync_cache_w(&hi3620_pen_release);
}

void hi3xxx_set_cpu_jump(int cpu, void *jump_addr)
{
	cpu = cpu_logical_map(cpu);
	if (!cpu || !ctrl_base)
		return;
	writel_relaxed(virt_to_phys(jump_addr), ctrl_base + ((cpu - 1) << 2));
}

int hi3xxx_get_cpu_jump(int cpu)
{
	cpu = cpu_logical_map(cpu);
	if (!cpu || !ctrl_base)
		return 0;
	return readl_relaxed(ctrl_base + ((cpu - 1) << 2));
}

static void __init hisi_enable_scu_a9(void)
{
	unsigned long base = 0;
	void __iomem *scu_base = NULL;

	if (scu_a9_has_base()) {
		base = scu_a9_get_base();
		scu_base = ioremap(base, SZ_4K);
		if (!scu_base) {
			pr_err("ioremap(scu_base) failed\n");
			return;
		}
		scu_enable(scu_base);
		iounmap(scu_base);
	}
}

static void __init hi3xxx_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *np = NULL;
	u32 offset = 0;
	u32 boot_addr;

	hisi_enable_scu_a9();

	np = of_find_compatible_node(NULL, NULL, "hisilicon,sysctrl");
	if (!np) {
		pr_err("failed to find hisilicon,sysctrl node\n");
		return;
	}

	hi3620_sysctrl_base = of_iomap(np, 0);
	if (!hi3620_sysctrl_base) {
		pr_err("failed to map hisilicon,sysctrl\n");
		return;
	}

	/*
	 * Huawei's K3V2 production firmware (used by MediaPad/P6 generation)
	 * does not use the later Hi4511 per-CPU mailboxes at 0x31c.  Its
	 * BootMonitor waits for one shared secondary entry at SYSCTRL+0x314,
	 * then a targeted GIC SGI releases a core into a holding pen.
	 */
	if (of_machine_is_compatible("huawei,s10-101x")) {
		hi3620_use_k3v2_bootmon = true;
		hi3620_write_pen_release(-1);
		boot_addr = virt_to_phys(hi3620_k3v2_secondary_startup);
		writel(boot_addr,
		       hi3620_sysctrl_base + K3V2_BOOTMON_ADDR_OFFSET);
		/* Drain the posted MMIO write before any wakeup SGI can be sent. */
		readl(hi3620_sysctrl_base + K3V2_BOOTMON_ADDR_OFFSET);
		wmb();
		flush_cache_all();
		pr_info("HI3620-SMP: K3V2 BootMonitor mode, startup=%08x sysctrl+%x\n",
			boot_addr, K3V2_BOOTMON_ADDR_OFFSET);
		return;
	}

	if (of_property_read_u32(np, "smp-offset", &offset) < 0) {
		pr_err("failed to find smp-offset property\n");
		return;
	}
	ctrl_base = hi3620_sysctrl_base + offset;
}

static void hi3620_k3v2_secondary_init(unsigned int cpu)
{
	if (!hi3620_use_k3v2_bootmon)
		return;

	/* Tell CPU0 that this core left the non-coherent holding pen. */
	hi3620_write_pen_release(-1);

	/* Match Huawei's vendor boot_lock handshake. */
	raw_spin_lock(&hi3620_boot_lock);
	raw_spin_unlock(&hi3620_boot_lock);
}

static int hi3xxx_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	int hwcpu;

	if (!hi3620_use_k3v2_bootmon) {
		hi3xxx_set_cpu(cpu, true);
		hi3xxx_set_cpu_jump(cpu, secondary_startup);
		arch_send_wakeup_ipi_mask(cpumask_of(cpu));
		return 0;
	}

	hwcpu = cpu_logical_map(cpu);
	raw_spin_lock(&hi3620_boot_lock);
	hi3620_write_pen_release(hwcpu);

	/* BootMonitor wakes on the targeted software interrupt. */
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (READ_ONCE(hi3620_pen_release) == -1)
			break;
		udelay(10);
	}

	raw_spin_unlock(&hi3620_boot_lock);

	if (READ_ONCE(hi3620_pen_release) != -1) {
		pr_err("HI3620-SMP: CPU%u (hw %d) did not leave K3V2 holding pen\n",
		       cpu, hwcpu);
		return -ENOSYS;
	}

	pr_info("HI3620-SMP: CPU%u (hw %d) left K3V2 holding pen\n",
		cpu, hwcpu);
	return 0;
}

static const struct smp_operations hi3xxx_smp_ops __initconst = {
	.smp_prepare_cpus	= hi3xxx_smp_prepare_cpus,
	.smp_secondary_init	= hi3620_k3v2_secondary_init,
	.smp_boot_secondary	= hi3xxx_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= hi3xxx_cpu_die,
	.cpu_kill		= hi3xxx_cpu_kill,
#endif
};

static void __init hisi_common_smp_prepare_cpus(unsigned int max_cpus)
{
	hisi_enable_scu_a9();
}

static void hix5hd2_set_scu_boot_addr(phys_addr_t start_addr, phys_addr_t jump_addr)
{
	void __iomem *virt;

	virt = ioremap(start_addr, PAGE_SIZE);

	writel_relaxed(0xe51ff004, virt);	/* ldr pc, [rc, #-4] */
	writel_relaxed(jump_addr, virt + 4);	/* pc jump phy address */
	iounmap(virt);
}

static int hix5hd2_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	phys_addr_t jumpaddr;

	jumpaddr = virt_to_phys(secondary_startup);
	hix5hd2_set_scu_boot_addr(HIX5HD2_BOOT_ADDRESS, jumpaddr);
	hix5hd2_set_cpu(cpu, true);
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));
	return 0;
}

static const struct smp_operations hix5hd2_smp_ops __initconst = {
	.smp_prepare_cpus	= hisi_common_smp_prepare_cpus,
	.smp_boot_secondary	= hix5hd2_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= hix5hd2_cpu_die,
#endif
};

#define SC_SCTL_REMAP_CLR      0x00000100
#define HIP01_BOOT_ADDRESS     0x80000000
#define REG_SC_CTRL            0x000

static void hip01_set_boot_addr(phys_addr_t start_addr, phys_addr_t jump_addr)
{
	void __iomem *virt;

	virt = phys_to_virt(start_addr);

	writel_relaxed(0xe51ff004, virt);
	writel_relaxed(jump_addr, virt + 4);
}

static int hip01_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	phys_addr_t jumpaddr;
	unsigned int remap_reg_value = 0;
	struct device_node *node;

	jumpaddr = virt_to_phys(secondary_startup);
	hip01_set_boot_addr(HIP01_BOOT_ADDRESS, jumpaddr);

	node = of_find_compatible_node(NULL, NULL, "hisilicon,hip01-sysctrl");
	if (WARN_ON(!node))
		return -1;
	ctrl_base = of_iomap(node, 0);

	/* set the secondary core boot from DDR */
	remap_reg_value = readl_relaxed(ctrl_base + REG_SC_CTRL);
	barrier();
	remap_reg_value |= SC_SCTL_REMAP_CLR;
	barrier();
	writel_relaxed(remap_reg_value, ctrl_base + REG_SC_CTRL);

	hip01_set_cpu(cpu, true);

	return 0;
}

static const struct smp_operations hip01_smp_ops __initconst = {
	.smp_prepare_cpus       = hisi_common_smp_prepare_cpus,
	.smp_boot_secondary     = hip01_boot_secondary,
};

CPU_METHOD_OF_DECLARE(hi3xxx_smp, "hisilicon,hi3620-smp", &hi3xxx_smp_ops);
CPU_METHOD_OF_DECLARE(hix5hd2_smp, "hisilicon,hix5hd2-smp", &hix5hd2_smp_ops);
CPU_METHOD_OF_DECLARE(hip01_smp, "hisilicon,hip01-smp", &hip01_smp_ops);
