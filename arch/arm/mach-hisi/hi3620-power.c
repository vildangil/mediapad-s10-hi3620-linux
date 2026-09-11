// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei MediaPad 10 FHD S10-101x hardware restart/power-off hooks.
 *
 * The old Huawei K3V2 kernel resets the SoC through the HI6421 PMUSPI
 * register block and cuts board power by driving GPIO19_6 low.  Mainline
 * Hi3620 has neither hook, so machine_restart() falls through to
 * "Reboot failed -- System halted" and machine_power_off() cannot remove
 * power.  Keep the vendor sequence board-local while the rest of the Hi3620
 * platform remains generic.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/string.h>

#include <asm/system_misc.h>

#define HI3620_PMUSPI_PHYS             0xfcc00000
#define HI3620_GPIO19_PHYS             0xfc819000
#define HI3620_POWER_MAP_SIZE          0x1000

/* Huawei pm.c: SECRAM_RESET_ADDR = PMUSPI + (0x87 << 2). */
#define PMUSPI_RESTART_TRIGGER         (0x86 << 2)
#define PMUSPI_RESTART_REASON          (0x87 << 2)
#define PMUSPI_RESTART_MAGIC           0x000000ee
#define RESTART_REASON_COLDBOOT        0x10
#define RESTART_REASON_BOOTLOADER      0x01
#define RESTART_REASON_RECOVERY        0x02

/* PL061 register layout. GPIO19_6 is the S10 board power-hold line. */
#define PL061_GPIODIR                  0x400
#define PL061_GPIO_DATA(pin)           BIT((pin) + 2)
#define S10_POWEROFF_PIN               6

static void __iomem *hi3620_pmuspi;
static void __iomem *hi3620_gpio19;

/* arch/arm/kernel/reboot.c owns this legacy callback. */
extern void (*pm_power_off)(void);

static u32 hi3620_restart_reason(const char *cmd)
{
        if (!cmd)
                return RESTART_REASON_COLDBOOT;
        if (!strcmp(cmd, "bootloader"))
                return RESTART_REASON_BOOTLOADER;
        if (!strcmp(cmd, "recovery"))
                return RESTART_REASON_RECOVERY;

        return RESTART_REASON_COLDBOOT;
}

static void hi3620_mediapad_restart(enum reboot_mode mode, const char *cmd)
{
        u32 reason = hi3620_restart_reason(cmd);

        if (!hi3620_pmuspi)
                return;

        pr_emerg("HI3620-POWER: restart cmd=%s reason=%02x\n",
                 cmd ? cmd : "coldboot", reason);

        writel(reason, hi3620_pmuspi + PMUSPI_RESTART_REASON);
        mb();

        /* Huawei's reboot_board() deliberately keeps writing 0xee until the
         * PMU resets the complete SoC.  Do not return to machine_restart().
         */
        for (;;) {
                writel(PMUSPI_RESTART_MAGIC,
                       hi3620_pmuspi + PMUSPI_RESTART_TRIGGER);
                mb();
                udelay(50);
        }
}

static void hi3620_mediapad_power_off(void)
{
        u8 dir;

        if (!hi3620_gpio19 || !hi3620_pmuspi)
                return;

        pr_emerg("HI3620-POWER: asserting GPIO19_6 low for power-off\n");

        /* Vendor hisik3_power_off() clears the abnormal reset marker first. */
        writel(0, hi3620_pmuspi + PMUSPI_RESTART_REASON);
        mb();

        /* Match pl061_direction_output(GPIO19_6, 0) without relying on the
         * dynamically allocated Linux gpiochip base during the final shutdown
         * path: data low, direction output, data low once more.
         */
        writeb(0, hi3620_gpio19 + PL061_GPIO_DATA(S10_POWEROFF_PIN));
        dir = readb(hi3620_gpio19 + PL061_GPIODIR);
        dir |= BIT(S10_POWEROFF_PIN);
        writeb(dir, hi3620_gpio19 + PL061_GPIODIR);
        writeb(0, hi3620_gpio19 + PL061_GPIO_DATA(S10_POWEROFF_PIN));
        mb();

        /* Power should disappear immediately. If the board does not switch
         * off, keep the latch asserted exactly as Huawei's kernel does.
         */
        for (;;)
                cpu_relax();
}

static int __init hi3620_mediapad_power_init(void)
{
        if (!of_machine_is_compatible("huawei,s10-101x"))
                return 0;

        hi3620_pmuspi = ioremap(HI3620_PMUSPI_PHYS, HI3620_POWER_MAP_SIZE);
        hi3620_gpio19 = ioremap(HI3620_GPIO19_PHYS, HI3620_POWER_MAP_SIZE);
        if (!hi3620_pmuspi || !hi3620_gpio19) {
                pr_err("HI3620-POWER: failed to map PMUSPI/GPIO19\n");
                if (hi3620_gpio19) {
                        iounmap(hi3620_gpio19);
                        hi3620_gpio19 = NULL;
                }
                if (hi3620_pmuspi) {
                        iounmap(hi3620_pmuspi);
                        hi3620_pmuspi = NULL;
                }
                return 0;
        }

        arm_pm_restart = hi3620_mediapad_restart;
        pm_power_off = hi3620_mediapad_power_off;

        pr_info("HI3620-POWER: Huawei restart/power-off hooks installed\n");
        return 0;
}
arch_initcall(hi3620_mediapad_power_init);
