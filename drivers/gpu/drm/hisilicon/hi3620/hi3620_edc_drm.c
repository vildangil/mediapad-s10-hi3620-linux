// SPDX-License-Identifier: GPL-2.0
/*
 * Experimental HiSilicon Hi3620 EDC0 DRM/KMS takeover driver.
 *
 * The MediaPad 10 FHD bootloader leaves EDC0/LDI0/MIPI DSI running with the
 * Panasonic VVX10F002A00 at 1920x1200.  The first bring-up step deliberately
 * adopts that live pipeline instead of reinitialising display clocks, LDI or
 * DSI.  Atomic plane updates only replace the EDC0 CH1 framebuffer address,
 * stride/size/format and assert EDC DISP_CTL.cfg_ok.
 *
 * This is intentionally board-scoped and experimental.  simplefb remains in
 * the DT as a console/fallback until the KMS takeover is proven reliable.
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

#define EDC_CH1L_ADDR		0x004
#define EDC_CH1R_ADDR		0x008
#define EDC_CH1_STRIDE		0x00c
#define EDC_CH1_XY		0x010
#define EDC_CH1_SIZE		0x014
#define EDC_CH1_CTL		0x018
#define EDC_DISP_SIZE		0x090
#define EDC_DISP_CTL		0x094
#define EDC_STS			0x09c
#define EDC_INTS		0x0a0
#define EDC_INTE		0x0a4

#define EDC_CH1_CTL_PIX_FMT_SHIFT	16
#define EDC_CH1_CTL_PIX_FMT_MASK	GENMASK(18, 16)
#define EDC_CH1_CTL_BGR		BIT(19)
#define EDC_CH1_CTL_ENABLE		BIT(24)

/* Vendor k3_edc.h values. */
#define EDC_FMT_XRGB8888	2
#define EDC_FMT_ARGB8888	3

/* EDC_DISP_CTL bit layout from Huawei's K3V2 vendor driver. */
#define EDC_DISP_CTL_CFG_OK		BIT(1)
#define EDC_DISP_CTL_ENABLE		BIT(10)

struct hi3620_edc {
	struct drm_device *drm;
	struct drm_fbdev_cma *fbdev;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;
	void __iomem *regs;
};

static const struct drm_display_mode hi3620_panel_mode = {
	DRM_MODE("1920x1200", DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		 150000, 1920, 2009, 2014, 2063, 0,
		 1200, 1206, 1208, 1212, 0,
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC)
};

static inline struct hi3620_edc *pipe_to_hi3620(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct hi3620_edc, pipe);
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

	fmt = fb->pixel_format == DRM_FORMAT_ARGB8888 ?
		EDC_FMT_ARGB8888 : EDC_FMT_XRGB8888;

	/* Preserve bootloader/vendor tuning bits and touch only CH1 scanout fields. */
	writel(lower_32_bits(paddr), edc->regs + EDC_CH1L_ADDR);
	writel(lower_32_bits(paddr), edc->regs + EDC_CH1R_ADDR);
	writel(fb->pitches[0] & GENMASK(13, 0), edc->regs + EDC_CH1_STRIDE);
	writel(0, edc->regs + EDC_CH1_XY);
	writel((height & 0xfff) | ((width & 0xfff) << 16),
	       edc->regs + EDC_CH1_SIZE);

	ctl = readl(edc->regs + EDC_CH1_CTL);
	ctl &= ~(EDC_CH1_CTL_PIX_FMT_MASK | EDC_CH1_CTL_BGR);
	ctl |= fmt << EDC_CH1_CTL_PIX_FMT_SHIFT;
	ctl |= EDC_CH1_CTL_ENABLE;
	writel(ctl, edc->regs + EDC_CH1_CTL);

	hi3620_edc_latch(edc);
}

static int hi3620_pipe_check(struct drm_simple_display_pipe *pipe,
			     struct drm_plane_state *plane_state,
			     struct drm_crtc_state *crtc_state)
{
	const struct drm_display_mode *m = &crtc_state->mode;

	/* Stage 1 is takeover-only: never touch LDI/DSI timings. */
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
	/* Keep the bootloader-programmed pipeline alive during initial takeover. */
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

static void hi3620_drm_output_poll_changed(struct drm_device *drm)
{
	struct hi3620_edc *edc = drm->dev_private;

	if (edc->fbdev)
		drm_fbdev_cma_hotplug_event(edc->fbdev);
}

static const struct drm_mode_config_funcs hi3620_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = hi3620_drm_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int hi3620_drm_load(struct drm_device *drm, unsigned long flags)
{
	struct hi3620_edc *edc = drm->dev_private;
	int ret;

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
					   hi3620_formats, ARRAY_SIZE(hi3620_formats),
					   &edc->connector);
	if (ret)
		goto err_connector;

	drm_mode_config_reset(drm);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto err_connector;

	edc->fbdev = drm_fbdev_cma_init(drm, 32, 1, 1);
	if (IS_ERR(edc->fbdev)) {
		ret = PTR_ERR(edc->fbdev);
		edc->fbdev = NULL;
		goto err_vblank;
	}

	drm_kms_helper_poll_init(drm);

	dev_info(drm->dev,
		 "Hi3620 EDC0 KMS takeover ready: disp_ctl=%08x size=%08x sts=%08x ints=%08x inte=%08x\n",
		 readl(edc->regs + EDC_DISP_CTL), readl(edc->regs + EDC_DISP_SIZE),
		 readl(edc->regs + EDC_STS), readl(edc->regs + EDC_INTS),
		 readl(edc->regs + EDC_INTE));
	return 0;

err_vblank:
	drm_vblank_cleanup(drm);
err_connector:
	drm_connector_cleanup(&edc->connector);
err_config:
	drm_mode_config_cleanup(drm);
	return ret;
}

static int hi3620_drm_unload(struct drm_device *drm)
{
	struct hi3620_edc *edc = drm->dev_private;

	drm_kms_helper_poll_fini(drm);
	if (edc->fbdev) {
		drm_fbdev_cma_fini(edc->fbdev);
		edc->fbdev = NULL;
	}
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
	.minor = 1,
};

static int hi3620_edc_probe(struct platform_device *pdev)
{
	struct hi3620_edc *edc;
	struct drm_device *drm;
	struct resource *res;
	int ret;

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
		drm_dev_unref(drm);
		return ret;
	}

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