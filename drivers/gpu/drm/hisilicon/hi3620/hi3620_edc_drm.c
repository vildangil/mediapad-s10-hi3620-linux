// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal HiSilicon Hi3620 EDC0 DRM/KMS bring-up driver.
 *
 * The MediaPad 10 FHD bootloader already leaves EDC0/LDI0/MIPI running.
 * Probe is non-destructive. An explicit KMS commit changes only the active
 * EDC channel framebuffer address; all bootloader timing/format/overlay
 * registers are preserved.
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
#define EDC_CH1_SIZE            0x014
#define EDC_CH1_CTL             0x018
#define EDC_CH2L_ADDR           0x024
#define EDC_CH2R_ADDR           0x028
#define EDC_CH2_STRIDE          0x02c
#define EDC_CH2_SIZE            0x034
#define EDC_CH2_CTL             0x038
#define EDC_CH12_OVLY           0x044
#define EDC_DISP_SIZE           0x090
#define EDC_DISP_CTL            0x094
#define EDC_STS                 0x09c
#define EDC_INTS                0x0a0
#define EDC_INTE                0x0a4

#define EDC_CH1_ENABLE          BIT(24)
#define EDC_CH2_ENABLE          BIT(21)
#define EDC_CFG_OK              BIT(1)
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
	u32 boot_laddr;
	u32 boot_raddr;
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

static void hi3620_channel_regs(struct hi3620_edc *edc,
				u32 *laddr, u32 *raddr,
				u32 *stride, u32 *size, u32 *ctl)
{
	if (edc->channel == HI3620_EDC_CH2) {
		*laddr = EDC_CH2L_ADDR;
		*raddr = EDC_CH2R_ADDR;
		*stride = EDC_CH2_STRIDE;
		*size = EDC_CH2_SIZE;
		*ctl = EDC_CH2_CTL;
	} else {
		*laddr = EDC_CH1L_ADDR;
		*raddr = EDC_CH1R_ADDR;
		*stride = EDC_CH1_STRIDE;
		*size = EDC_CH1_SIZE;
		*ctl = EDC_CH1_CTL;
	}
}

static void hi3620_detect_boot_channel(struct hi3620_edc *edc)
{
	u32 ch1_l = readl(edc->regs + EDC_CH1L_ADDR);
	u32 ch1_r = readl(edc->regs + EDC_CH1R_ADDR);
	u32 ch1_stride = readl(edc->regs + EDC_CH1_STRIDE);
	u32 ch1_size = readl(edc->regs + EDC_CH1_SIZE);
	u32 ch1_ctl = readl(edc->regs + EDC_CH1_CTL);
	u32 ch2_l = readl(edc->regs + EDC_CH2L_ADDR);
	u32 ch2_r = readl(edc->regs + EDC_CH2R_ADDR);
	u32 ch2_stride = readl(edc->regs + EDC_CH2_STRIDE);
	u32 ch2_size = readl(edc->regs + EDC_CH2_SIZE);
	u32 ch2_ctl = readl(edc->regs + EDC_CH2_CTL);
	bool ch1_on = !!(ch1_ctl & EDC_CH1_ENABLE);
	bool ch2_on = !!(ch2_ctl & EDC_CH2_ENABLE);

	if (ch2_on && !ch1_on)
		edc->channel = HI3620_EDC_CH2;
	else if (ch1_on && !ch2_on)
		edc->channel = HI3620_EDC_CH1;
	else if (ch2_l == S10_BOOT_FB_PHYS)
		edc->channel = HI3620_EDC_CH2;
	else if (ch1_l == S10_BOOT_FB_PHYS)
		edc->channel = HI3620_EDC_CH1;
	else if (ch2_l)
		edc->channel = HI3620_EDC_CH2;
	else
		edc->channel = HI3620_EDC_CH1;

	if (edc->channel == HI3620_EDC_CH2) {
		edc->boot_laddr = ch2_l;
		edc->boot_raddr = ch2_r;
	} else {
		edc->boot_laddr = ch1_l;
		edc->boot_raddr = ch1_r;
	}

	dev_info(edc->drm->dev,
		 "HI3620-DRM-BOOT: id=%08x "
		 "ch1_l=%08x ch1_r=%08x stride=%08x size=%08x ctl=%08x en=%u "
		 "ch2_l=%08x ch2_r=%08x stride=%08x size=%08x ctl=%08x en=%u "
		 "ovly=%08x disp_ctl=%08x selected=CH%u\n",
		 readl(edc->regs + EDC_ID),
		 ch1_l, ch1_r, ch1_stride, ch1_size, ch1_ctl, ch1_on,
		 ch2_l, ch2_r, ch2_stride, ch2_size, ch2_ctl, ch2_on,
		 readl(edc->regs + EDC_CH12_OVLY),
		 readl(edc->regs + EDC_DISP_CTL), edc->channel);
}

static void hi3620_latch(struct hi3620_edc *edc)
{
	u32 val = readl(edc->regs + EDC_DISP_CTL);

	writel(val | EDC_CFG_OK, edc->regs + EDC_DISP_CTL);
	wmb();
}

static void hi3620_program_plane(struct hi3620_edc *edc,
				 struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *gem;
	dma_addr_t paddr;
	u32 laddr_reg, raddr_reg, stride_reg, size_reg, ctl_reg;
	u32 stride, size, width, height, hw_width, hw_height;
	u32 right;

	if (!fb)
		return;

	gem = drm_fb_cma_get_gem_obj(fb, 0);
	if (!gem)
		return;

	paddr = gem->paddr + fb->offsets[0] +
		((state->src_y >> 16) * fb->pitches[0]) +
		((state->src_x >> 16) * 4);
	width = state->src_w >> 16;
	height = state->src_h >> 16;
	if (!width || !height)
		return;

	hi3620_channel_regs(edc, &laddr_reg, &raddr_reg,
			   &stride_reg, &size_reg, &ctl_reg);

	stride = readl(edc->regs + stride_reg) & GENMASK(13, 0);
	size = readl(edc->regs + size_reg);
	hw_width = ((size >> 16) & 0xfff) + 1;
	hw_height = (size & 0xfff) + 1;

	if (fb->pitches[0] != stride ||
	    width != hw_width || height != hw_height) {
		dev_err(edc->drm->dev,
			"HI3620-DRM-REFUSE: ch=%u addr=%08x "
			"pitch=%u/%u size=%ux%u/%ux%u fourcc=%08x\n",
			edc->channel, lower_32_bits(paddr),
			fb->pitches[0], stride,
			width, height, hw_width, hw_height,
			fb->pixel_format);
		return;
	}

	/* Address-only takeover. Leave stride/size/CTL/overlay untouched. */
	writel(lower_32_bits(paddr), edc->regs + laddr_reg);

	if (edc->boot_raddr == edc->boot_laddr) {
		writel(lower_32_bits(paddr), edc->regs + raddr_reg);
	} else if (edc->boot_raddr && edc->boot_laddr) {
		right = lower_32_bits(paddr) +
			(edc->boot_raddr - edc->boot_laddr);
		writel(right, edc->regs + raddr_reg);
	}

	hi3620_latch(edc);

	dev_info_ratelimited(edc->drm->dev,
		"HI3620-DRM-ADDR: ch=%u l=%08x r=%08x "
		"stride=%08x size=%08x ctl=%08x ovly=%08x "
		"disp_ctl=%08x fourcc=%08x\n",
		edc->channel,
		readl(edc->regs + laddr_reg),
		readl(edc->regs + raddr_reg),
		readl(edc->regs + stride_reg),
		readl(edc->regs + size_reg),
		readl(edc->regs + ctl_reg),
		readl(edc->regs + EDC_CH12_OVLY),
		readl(edc->regs + EDC_DISP_CTL),
		fb->pixel_format);
}

static int hi3620_pipe_check(struct drm_simple_display_pipe *pipe,
			     struct drm_plane_state *plane_state,
			     struct drm_crtc_state *crtc_state)
{
	const struct drm_display_mode *m = &crtc_state->mode;

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
	hi3620_program_plane(pipe_to_hi3620(pipe), pipe->plane.state);
}

static void hi3620_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	/* Keep the bootloader pipeline alive. */
}

static void hi3620_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old_state)
{
	hi3620_program_plane(pipe_to_hi3620(pipe), pipe->plane.state);
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

/*
 * Stage-1 vblank support: the 4.9 DRM core calls ->enable_vblank() from
 * drm_atomic_helper_wait_for_vblanks().  Leaving this callback NULL caused the
 * Xorg modeset to jump to PC=0 after the first successful CH2 address update.
 *
 * For now deliberately do not touch EDC interrupt registers.  The Huawei
 * vendor ISR identifies bit 7 (0x80, bas_stat_int) as the video-mode frame
 * boundary, but its acknowledge semantics still need to be ported carefully.
 * Returning success here prevents the NULL callback crash; the atomic helper
 * may time out waiting for a counter change, which is safe for this bring-up
 * image and PageFlip=false userspace configuration.
 */
static int hi3620_enable_vblank(struct drm_device *drm, unsigned int pipe)
{
	if (pipe != 0)
		return -EINVAL;

	DRM_DEBUG_DRIVER("HI3620-DRM-VBLANK: temporary software enable\n");
	return 0;
}

static void hi3620_disable_vblank(struct drm_device *drm, unsigned int pipe)
{
	DRM_DEBUG_DRIVER("HI3620-DRM-VBLANK: temporary software disable\n");
}

static const struct drm_mode_config_funcs hi3620_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int hi3620_drm_load(struct drm_device *drm, unsigned long flags)
{
	struct hi3620_edc *edc = drm->dev_private;
	int ret;

	hi3620_detect_boot_channel(edc);

	drm_mode_config_init(drm);
	drm->mode_config.min_width = 1920;
	drm->mode_config.max_width = 1920;
	drm->mode_config.min_height = 1200;
	drm->mode_config.max_height = 1200;
	drm->mode_config.funcs = &hi3620_mode_config_funcs;

	ret = drm_connector_init(drm, &edc->connector,
				 &hi3620_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		goto err_config;

	drm_connector_helper_add(&edc->connector,
				 &hi3620_connector_helper_funcs);

	ret = drm_simple_display_pipe_init(drm, &edc->pipe,
					   &hi3620_pipe_funcs,
					   hi3620_formats,
					   ARRAY_SIZE(hi3620_formats),
					   &edc->connector);
	if (ret)
		goto err_connector;

	drm_mode_config_reset(drm);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto err_connector;

	drm_kms_helper_poll_init(drm);

	dev_info(drm->dev,
		 "HI3620-DRM: address-only KMS ready: ch=%u "
		 "disp_ctl=%08x size=%08x sts=%08x ints=%08x inte=%08x\n",
		 edc->channel,
		 readl(edc->regs + EDC_DISP_CTL),
		 readl(edc->regs + EDC_DISP_SIZE),
		 readl(edc->regs + EDC_STS),
		 readl(edc->regs + EDC_INTS),
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
	.enable_vblank = hi3620_enable_vblank,
	.disable_vblank = hi3620_disable_vblank,
	.gem_free_object = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,
	.dumb_create = drm_gem_cma_dumb_create,
	.dumb_map_offset = drm_gem_cma_dumb_map_offset,
	.dumb_destroy = drm_gem_dumb_destroy,
	.fops = &hi3620_drm_fops,
	.name = "hi3620-edc",
	.desc = "HiSilicon Hi3620 EDC0 DRM/KMS address-only takeover",
	.date = "20260912",
	.major = 0,
	.minor = 4,
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
		dev_err(&pdev->dev,
			"HI3620-DRM: drm_dev_register failed: %d\n", ret);
		drm_dev_unref(drm);
		return ret;
	}

	dev_info(&pdev->dev,
		 "HI3620-DRM: EDC0 registered; KMS writes address only; vblank crash guard active\n");
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
MODULE_DESCRIPTION("Experimental address-only DRM/KMS takeover for Huawei MediaPad 10 FHD Hi3620 EDC0");
MODULE_LICENSE("GPL v2");