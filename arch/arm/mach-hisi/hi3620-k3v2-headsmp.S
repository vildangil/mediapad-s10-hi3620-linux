/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Huawei K3V2 secondary CPU holding pen.
 *
 * MediaPad 10 FHD stock firmware uses the K3V2 BootMonitor protocol: all
 * secondary cores enter the same trampoline and only the CPU selected by
 * hi3620_pen_release is allowed to continue into the generic ARM startup.
 */
#include <linux/linkage.h>

	.section ".text.head", "ax"
	.arm

ENTRY(hi3620_k3v2_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5
	and	r0, r0, #15
	adr	r4, 1f
	ldmia	r4, {r5, r6}
	sub	r4, r4, r5
	add	r6, r6, r4
0:	ldr	r7, [r6]
	cmp	r7, r0
	bne	0b
	b	secondary_startup
1:	.long	.
	.long	hi3620_pen_release
ENDPROC(hi3620_k3v2_secondary_startup)
