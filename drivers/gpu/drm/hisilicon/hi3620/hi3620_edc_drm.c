// SPDX-License-Identifier: GPL-2.0
/*
 * Experimental HiSilicon Hi3620 EDC0 DRM/KMS driver.
 *
 * The MediaPad 10 FHD bootloader leaves EDC0/LDI0/MIPI DSI running with the
 * Panasonic VVX10F002A00 at 1920x1200.  Linux must therefore adopt the live
 * scanout pipeline instead of resetting display clocks, LDI or DSI.
 *
 * Stage 2 is deliberately non-destructive at probe time: DRM/KMS is
 * registered, but no DRM fbdev emulation is created.  This leaves simplefb
 * and the bootloader scanout untouched until userspace explicitly performs a
 * KMS modeset.  When that happens, program whichever EDC channel the
 * bootloader actually left active (CH1 or CH2) instead of assuming CH1.
 */

#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_simple_kms_helper.h>

#define EDC_ID                  0x000

#define EDC_CH1L_ADDR           0x004
#define EDC_CH1R_ADDR           0x008
#define EDC_CH1_STRIDE          0x00c
#define EDC_CH1_XY              0x010
#define EDC_CH1_SIZE            0x014
#define EDC_CH1_CTL             0x018

#define EDC_CH2L_ADDR           0x024
#define EDC_CH2R_ADDR           0x028
#define EDC_CH2_STRIDE          0x02c
#define EDC_CH2_XY              0x030
#define EDC_CH2_SIZE            0x034
#define EDC_CH2_CTL             0x038

#define EDC_CH12_OVLY           0x044
#define EDC_DISP_SIZE           0x090
#define EDC_DISP_CTL            0x094
#define EDC_STS                 0x09c
#define EDC_INTS                0x0a0
#define EDC_INTE                0x0a4

#define EDC_CTL_PIX_FMT_SHIFT   16
#define EDC_CTL_PIX_FMT_MASK    GENMASK(18, 16)
#define EDC_CTL_BGR             BIT(19)
#define EDC_CH1_CTL_ENABLE      BIT(24)
#define EDC_CH2_CTL_ENABLE      BIT(21)

/* Vendor k3_edc.h values. */
#define EDC_FMT_XRGB8888        2
#define EDC_FMT_ARGB8888        3

/* EDC_DISP_CTL bit layout from Huawei's K3V2 vendor driver. */
#define EDC_DISP_CTL_CFG_OK     BIT(1)
#define EDC_DISP_CTL_ENABLE     BIT(10)

/* Known bootloader simplefb physical address on MediaPad 10 FHD. */
#define S10_BOOT_FB_PHYS        0x2f300000

enum hi3620_edc_channel {
	HI3620_EDC_CH1 = 1,
	HI3620_EDC_CH2 = 2,
};

struct hi3620_edc {
	struct drm_device *drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;
	void __iomem *regs;
	enum hi3620_edc_channel channel;
};

static const struct drm_display_mode hi3620_panel_mode = {
	DRM_MODE("1920x1200", DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		 150000, 1920, 2009, 2014, 2063, 0,
		 1200, 1206, 1208, 1212, 0,
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC)
};

static inline struct hi3620_edc *
pipe_to_hi3620(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct hi3620_edc, pipe);
}

static void hi3620_edc_get_channel_regs(struct hi3620_edc *edc,
					u32 *laddr, u32 *raddr,
					u32 *stride, u32 *xy,
					u32 *size, u32 *ctl,
					u32 *enable)
{
	if (edc->channel == HI3620_EDC_CH2) {
		*laddr = EDC_CH2L_ADDR;
		*raddr = EDC_CH2R_ADDR;
		*stride = EDC_CH2_STRIDE;
		*xy = EDC_CH2_XY;
		*size = EDC_CH2_SIZE;
		*ctl = EDC_CH2_CTL;
		*enable = EDC_CH2_CTL_ENABLE;
	} else {
		*laddr = EDC_CH1L_ADDR;
		*raddr = EDC_CH1R_ADDR;
		*stride = EDC_CH1_STRIDE;
		*xy = EDC_CH1_XY;
		*size = EDC_CH1_SIZE;
		*ctl = EDC_CH1_CTL;
		*enable = EDC_CH1_CTL_ENABLE;
	}
}

static void hi3620_edc_detect_boot_channel(struct hi3620_edc *edc)
{
	u32 ch1_addr = readl(edc->regs + EDC_CH1L_ADDR);
	u32 ch2_addr = readl(edc->regs + EDC_CH2L_ADDR);
	u32 ch1_ctl = readl(edc->regs + EDC_CH1_CTL);
	u32 ch2_ctl = readl(edc->regs + EDC_CH2_CTL);
	bool ch1_on = !!(ch1_ctl & EDC_CH1_CTL_ENABLE);
	bool ch2_on = !!(ch2_ctl & EDC_CH2_CTL_ENABLE);

	/*
	 * Prefer an enabled channel.  If both/none are enabled, the known
	 * bootloader framebuffer address is the strongest board-specific hint.
	 * Old MediaPad logs consistently reported 0x2f300000 on channel 2.
	 */
	if (ch2_on && !ch1_on)
		edc->channel = HI3620_EDC_CH2;
	else if (ch1_on && !ch2_on)
		edc->channel = HI3620_EDC_CH1;
	else if (ch2_addr == S10_BOOT_FB_PHYS)
		edc->channel = HI3620_EDC_CH2;
	else if (ch1_addr == S10_BOOT_FB_PHYS)
		edc->channel = HI3620_EDC_CH1;
	else if (ch2_addr)
		edc->channel = HI3620_EDC_CH2;
	else
		edc->channel = HI3620_EDC_CH1;

	dev_info(edc->drm->dev,
		 "HI3620-DRM-BOOT: id=%08x ch1_addr=%08x ch1_ctl=%08x en=%u ch2_addr=%08x ch2_ctl=%08x en=%u ovly=%08x selected=CH%u\n",
		 readl(edc->regs + EDC_ID),
		 ch1_addr, ch1_ctl, ch1_on,
		 ch2_addr, ch2_ctl, ch2_on,
		 readl(edc->regs + EDC_CH12_OVLY), edc->channel);
}

static void hi3620_edc_latch(struct hi3620_edc *edc)
{
	u32 val;

	/* Huawei's vendor path writes cfg_ok=1 after every overlay update. */
	val = readl(edc->regs + EDC_DISP_CTL);
	writel(val | EDC_DISP_CTL_CFG_OK, edc->regs + EDC_DISP_CTL);
	wmb();
}

static void hi3620_edc_program_plane(struct hi3620_edc *edc,
				     struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *gem;
	dma_addr_t paddr;
	u32 laddr_reg, raddr_reg, stride_reg, xy_reg, size_reg, ctl_reg;
	u32 enable_bit;
	u32 ctl, fmt;
	u32 width, height;

	if (!fb)
		return;

	gem = drm_fb_cma_get_gem_obj(fb, 0);
	if (!gem)
		return;

	/* The simple-pipe helper disallows scaling. Account for FB/source offsets. */
	paddr = gem->paddr + fb->offsets[0] +
		((state->src_y >> 16) * fb->pitches[0]) +
		((state->src_x >> 16) * 4);
	width = state->src_w >> 16;
	height = state->src_h >> 16;
	if (!width || !height)
		return;

	fmt = fb->pixel_format == DRM_FORMAT_ARGB8888 ?
		EDC_FMT_ARGB8888 : EDC_FMT_XRGB8888;

	hi3620_edc_get_channel_regs(edc, &laddr_reg, &raddr_reg,
				    &stride_reg, &xy_reg, &size_reg,
				    &ctl_reg, &enable_bit);

	/*
	 * Preserve bootloader/vendor tuning and overlay ordering.  Only replace
	 * the selected live channel's scanout fields.
	 */
	writel(lower_32_bits(paddr), edc->regs + laddr_reg);
	writel(lower_32_bits(paddr), edc->regs + raddr_reg);
	writel(fb->pitches[0] & GENMASK(13, 0), edc->regs + stride_reg);
	writel(0, edc->regs + xy_reg);

	/* Huawei set_EDC_CH{1,2}_SIZE() stores size - 1 in each 12-bit field. */
	writel(((height - 1) & 0xfff) | (((width - 1) & 0xfff) << 16),
	       edc->regs + size_reg);

	ctl = readl(edc->regs + ctl_reg);
	ctl &= ~(EDC_CTL_PIX_FMT_MASK | EDC_CTL_BGR);
	ctl |= fmt << EDC_CTL_PIX_FMT_SHIFT;
	ctl |= enable_bit;
	writel(ctl, edc->regs + ctl_reg);

	hi3620_edc_latch(edc);

	dev_info_ratelimited(edc->drm->dev,
			     "HI3620-DRM-FLIP: ch=%u addr=%08x stride=%u size=%ux%u ctl=%08x disp_ctl=%08x\n",
			     edc->channel, lower_32_bits(paddr), fb->pitches[0],
			     width, height, readl(edc->regs + ctl_reg),
			     readl(edc->regs + EDC_DISP_CTL));
}

static int hi3620_pipe_check(struct drm_simple_display_pipe *pipe,
			     struct drm_plane_state *plane_state,
			     struct drm_crtc_state *crtc_state)
{
	const struct drm_display_mode *m = &crtc_state->mode;

	/* Takeover-only: never touch LDI/DSI timings. */
	if (crtc_state->enable &&
	    (m->hdisplay != 1920 || m->vdisplay != 1200 ||
	     m->htotal != 2063 || m->vtotal != 1212))
		return -EINVAL;

	if (plane_state->fb && plane_state->fb->pitches[0] > 0x3fff)
		return -EINVAL;

	return 0;
}

static void hi3620_pipe_enable(struct drm_simple_display_pipe *pipe,
			       struct drm_crtc_state *crtc_state)
{
	struct hi3620_edc *edc = pipe_to_hi3620(pipe);

	hi3620_edc_program_plane(edc, pipe->plane.state);
}

static void hi3620_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	/* Never blank the bootloader pipeline during early bring-up. */
}

static void hi3620_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old_state)
{
	struct hi3620_edc *edc = pipe_to_hi3620(pipe);

	/* Atomic helpers have already installed the new state in plane.state. */
	hi3620_edc_program_plane(edc, pipe->plane.state);
}

static const struct drm_simple_display_pipe_funcs hi3620_pipe_funcs = {
	.check = hi3620_pipe_check,
	.enable = hi3620_pipe_enable,
	.disable = hi3620_pipe_disable,
	.update = hi3620_pipe_update,
};

static int hi3620_connector_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &hi3620_panel_mode);
	if (!mode)
		return 0;

	drm_mode_probed_add(connector, mode);
	return 1;
}

static const struct drm_connector_helper_funcs hi3620_connector_helper_funcs = {
	.get_modes = hi3620_connector_get_modes,
};

static enum drm_connector_status
hi3620_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static const struct drm_connector_funcs hi3620_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.detect = hi3620_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const uint32_t hi3620_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const struct drm_mode_config_funcs hi3620_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int hi3620_drm_load(struct drm_device *drm, unsigned long flags)
{
	struct hi3620_edc *edc = drm->dev_private;
	int ret;

	/*
	 * Read the bootloader state before creating KMS objects.  This is
	 * diagnostic-only and does not touch the live display.
	 */
	hi3620_edc_detect_boot_channel(edc);

	drm_mode_config_init(drm);
	drm->mode_config.min_width = 1920;
	drm->mode_config.max_width = 1920;
	drm->mode_config.min_height = 1200;
	drm->mode_config.max_height = 1200;
	drm->mode_config.funcs = &hi3620_mode_config_funcs;

	ret = drm_connector_init(drm, &edc->connector, &hi3620_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		goto err_config;

	drm_connector_helper_add(&edc->connector, &hi3620_connector_helper_funcs);

	ret = drm_simple_display_pipe_init(drm, &edc->pipe, &hi3620_pipe_funcs,
					   hi3620_formats,
					   ARRAY_SIZE(hi3620_formats),
					   &edc->connector);
	if (ret)
		goto err_connector;

	drm_mode_config_reset(drm);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto err_connector;

	/*
	 * Intentionally do NOT call drm_fbdev_cma_init() yet.  The previous
	 * experiment performed an automatic KMS commit during probe and could
	 * replace the live bootloader scanout before we had verified the active
	 * EDC channel.  /dev/dri/card* is still fully registered; an explicit
	 * userspace KMS commit will exercise hi3620_edc_program_plane().
	 */
	drm_kms_helper_poll_init(drm);

	dev_info(drm->dev,
		 "HI3620-DRM: KMS registered non-destructive: ch=%u disp_ctl=%08x size=%08x sts=%08x ints=%08x inte=%08x; simplefb retained until explicit modeset\n",
		 edc->channel, readl(edc->regs + EDC_DISP_CTL),
		 readl(edc->regs + EDC_DISP_SIZE),
		 readl(edc->regs + EDC_STS), readl(edc->regs + EDC_INTS),
		 readl(edc->regs + EDC_INTE));
	return 0;

err_connector:
	drm_connector_cleanup(&edc->connector);
err_config:
	drm_mode_config_cleanup(drm);
	return ret;
}

static int hi3620_drm_unload(struct drm_device *drm)
{
	drm_kms_helper_poll_fini(drm);
	drm_vblank_cleanup(drm);
	drm_mode_config_cleanup(drm);
	return 0;
}

static const struct file_operations hi3620_drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.poll = drm_poll,
	.read = drm_read,
	.llseek = no_llseek,
	.mmap = drm_gem_cma_mmap,
};

static struct drm_driver hi3620_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.load = hi3620_drm_load,
	.unload = hi3620_drm_unload,
	.gem_free_object = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,
	.dumb_create = drm_gem_cma_dumb_create,
	.dumb_map_offset = drm_gem_cma_dumb_map_offset,
	.dumb_destroy = drm_gem_dumb_destroy,
	.fops = &hi3620_drm_fops,
	.name = "hi3620-edc",
	.desc = "HiSilicon Hi3620 EDC0 DRM/KMS takeover",
	.date = "20260912",
	.major = 0,
	.minor = 2,
};

static int hi3620_edc_probe(struct platform_device *pdev)
{
	struct hi3620_edc *edc;
	struct drm_device *drm;
	struct resource *res;
	int ret;

	dev_info(&pdev->dev, "HI3620-DRM: EDC0 probe start\n");

	edc = devm_kzalloc(&pdev->dev, sizeof(*edc), GFP_KERNEL);
	if (!edc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	edc->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(edc->regs))
		return PTR_ERR(edc->regs);

	drm = drm_dev_alloc(&hi3620_drm_driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	edc->drm = drm;
	drm->dev_private = edc;
	platform_set_drvdata(pdev, edc);

	ret = drm_dev_register(drm, 0);
	if (ret) {
		dev_err(&pdev->dev, "HI3620-DRM: drm_dev_register failed: %d\n",
			ret);
		drm_dev_unref(drm);
		return ret;
	}

	dev_info(&pdev->dev,
		 "HI3620-DRM: EDC0 registered without automatic scanout takeover\n");
	return 0;
}

static int hi3620_edc_remove(struct platform_device *pdev)
{
	struct hi3620_edc *edc = platform_get_drvdata(pdev);

	drm_dev_unregister(edc->drm);
	drm_dev_unref(edc->drm);
	return 0;
}

static const struct of_device_id hi3620_edc_of_match[] = {
	{ .compatible = "hisilicon,hi3620-edc-kms" },
	{ .compatible = "hisilicon,hi3620-edc-drm" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3620_edc_of_match);

static struct platform_driver hi3620_edc_platform_driver = {
	.probe = hi3620_edc_probe,
	.remove = hi3620_edc_remove,
	.driver = {
		.name = "hi3620-edc-drm",
		.of_match_table = hi3620_edc_of_match,
	},
};
module_platform_driver(hi3620_edc_platform_driver);

MODULE_AUTHOR("VildanG / OpenAI bring-up");
MODULE_DESCRIPTION("Experimental DRM/KMS takeover driver for Huawei MediaPad 10 FHD Hi3620 EDC0");
MODULE_LICENSE("GPL v2");
