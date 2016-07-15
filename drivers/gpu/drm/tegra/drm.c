/*
 * Copyright (C) 2012 Avionic Design GmbH
 * Copyright (C) 2012-2013 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/host1x.h>
#include <linux/iommu.h>

#include "drm.h"
#include "gem.h"

#define DRIVER_NAME "tegra"
#define DRIVER_DESC "NVIDIA Tegra graphics"
#define DRIVER_DATE "20120330"
#define DRIVER_MAJOR 0
#define DRIVER_MINOR 0
#define DRIVER_PATCHLEVEL 0

struct tegra_drm_file {
	struct list_head contexts;
};

static struct platform_device *tegra_drm_pdev;
static struct drm_driver tegra_drm_driver;
struct iommu_domain *tegradrm_iommu_domain;

static int tegra_drm_load(struct drm_device *drm, unsigned long flags)
{
	struct host1x_device *device = to_host1x_device(drm->dev);
	struct tegra_drm *tegra;
	int err;

	tegra = kzalloc(sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	tegra->domain = tegradrm_iommu_domain;
	drm_mm_init(&tegra->mm, 0, SZ_2G);

	dev_set_drvdata(drm->dev, tegra);
	mutex_init(&tegra->clients_lock);
	INIT_LIST_HEAD(&tegra->clients);
	drm->dev_private = tegra;
	tegra->drm = drm;

	drm_mode_config_init(drm);

	err = tegra_drm_fb_prepare(drm);
	if (err < 0)
		goto config;

	drm_kms_helper_poll_init(drm);

	err = drm_host1x_device_init(drm, device);
	if (err < 0)
		goto fbdev;

	/*
	 * We don't use the drm_irq_install() helpers provided by the DRM
	 * core, so we need to set this manually in order to allow the
	 * DRM_IOCTL_WAIT_VBLANK to operate correctly.
	 */
	drm->irq_enabled = true;

	err = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (err < 0)
		goto device;

	err = tegra_drm_fb_init(drm);
	if (err < 0)
		goto vblank;

	return 0;

vblank:
	drm_vblank_cleanup(drm);
device:
	drm_host1x_exit(&tegra_drm_driver, device);
fbdev:
	drm_kms_helper_poll_fini(drm);
	tegra_drm_fb_free(drm);
config:
	drm_mode_config_cleanup(drm);
	if (tegra->domain)
		drm_mm_takedown(&tegra->mm);
	kfree(tegra);
	return err;
}

static int tegra_drm_unload(struct drm_device *drm)
{
	struct host1x_device *device = to_host1x_device(drm->dev);
	struct tegra_drm *tegra = drm->dev_private;

	drm_kms_helper_poll_fini(drm);
	tegra_drm_fb_exit(drm);
	drm_vblank_cleanup(drm);
	drm_mode_config_cleanup(drm);

	drm_host1x_exit(&tegra_drm_driver, device);

	if (tegra->domain)
		drm_mm_takedown(&tegra->mm);
	kfree(tegra);

	return 0;
}

static int tegra_drm_open(struct drm_device *drm, struct drm_file *filp)
{
	struct tegra_drm_file *fpriv;

	fpriv = kzalloc(sizeof(*fpriv), GFP_KERNEL);
	if (!fpriv)
		return -ENOMEM;

	INIT_LIST_HEAD(&fpriv->contexts);
	filp->driver_priv = fpriv;

	return 0;
}

static void tegra_drm_context_free(struct tegra_drm_context *context)
{
	context->client->ops->close_channel(context);
	kfree(context);
}

static void tegra_drm_lastclose(struct drm_device *drm)
{
#ifdef CONFIG_DRM_TEGRA_FBDEV
	struct tegra_drm *tegra = drm->dev_private;

	tegra_fbdev_restore_mode(tegra->fbdev);
#endif
}

static struct tegra_drm_context *tegra_drm_get_context(__u64 context)
{
	return (struct tegra_drm_context *)(uintptr_t)context;
}

static bool tegra_drm_file_owns_context(struct tegra_drm_file *file,
					struct tegra_drm_context *context)
{
	struct tegra_drm_context *ctx;

	list_for_each_entry(ctx, &file->contexts, list)
		if (ctx == context)
			return true;

	return false;
}

static int tegra_gem_create(struct drm_device *drm, void *data,
			    struct drm_file *file)
{
	struct drm_tegra_gem_create *args = data;
	struct tegra_bo *bo;

	bo = tegra_bo_create_with_handle(file, drm, args->size, args->flags,
					 &args->handle);
	if (IS_ERR(bo))
		return PTR_ERR(bo);

	return 0;
}

static int tegra_gem_mmap(struct drm_device *drm, void *data,
			  struct drm_file *file)
{
	struct drm_tegra_gem_mmap *args = data;
	struct drm_gem_object *gem;
	struct tegra_bo *bo;

	gem = drm_gem_object_lookup(drm, file, args->handle);
	if (!gem)
		return -EINVAL;

	bo = to_tegra_bo(gem);

	args->offset = drm_vma_node_offset_addr(&bo->gem.vma_node);

	drm_gem_object_unreference(gem);

	return 0;
}

#ifdef CONFIG_DRM_TEGRA_STAGING
static int tegra_syncpt_read(struct drm_device *drm, void *data,
			     struct drm_file *file)
{
	struct host1x *host = dev_get_drvdata(drm->dev->parent);
	struct drm_tegra_syncpt_read *args = data;
	struct host1x_syncpt *sp;

	sp = host1x_syncpt_get(host, args->id);
	if (!sp)
		return -EINVAL;

	args->value = host1x_syncpt_read_min(sp);
	return 0;
}

static int tegra_syncpt_incr(struct drm_device *drm, void *data,
			     struct drm_file *file)
{
	struct host1x *host1x = dev_get_drvdata(drm->dev->parent);
	struct drm_tegra_syncpt_incr *args = data;
	struct host1x_syncpt *sp;

	sp = host1x_syncpt_get(host1x, args->id);
	if (!sp)
		return -EINVAL;

	return host1x_syncpt_incr(sp);
}

static int tegra_syncpt_wait(struct drm_device *drm, void *data,
			     struct drm_file *file)
{
	struct host1x *host1x = dev_get_drvdata(drm->dev->parent);
	struct drm_tegra_syncpt_wait *args = data;
	struct host1x_syncpt *sp;

	sp = host1x_syncpt_get(host1x, args->id);
	if (!sp)
		return -EINVAL;

	return host1x_syncpt_wait(sp, args->thresh, args->timeout,
				  &args->value);
}
#endif

static int tegra_open_channel(struct drm_device *drm, void *data,
			      struct drm_file *file)
{
	struct tegra_drm_file *fpriv = file->driver_priv;
	struct tegra_drm *tegra = drm->dev_private;
	struct drm_tegra_open_channel *args = data;
	struct tegra_drm_context *context;
	struct tegra_drm_client *client;
	int err = -ENODEV;

	context = kzalloc(sizeof(*context), GFP_KERNEL);
	if (!context)
		return -ENOMEM;

	list_for_each_entry(client, &tegra->clients, list)
		if (client->base.class == args->client) {
			err = client->ops->open_channel(client, context);
			if (err)
				break;

			list_add(&context->list, &fpriv->contexts);
			args->context = (uintptr_t)context;
			context->client = client;
			return 0;
		}

	kfree(context);
	return err;
}

static int tegra_close_channel(struct drm_device *drm, void *data,
			       struct drm_file *file)
{
	struct tegra_drm_file *fpriv = file->driver_priv;
	struct drm_tegra_close_channel *args = data;
	struct tegra_drm_context *context;

	context = tegra_drm_get_context(args->context);

	if (!tegra_drm_file_owns_context(fpriv, context))
		return -EINVAL;

	list_del(&context->list);
	tegra_drm_context_free(context);

	return 0;
}

#ifdef CONFIG_DRM_TEGRA_STAGING
static int tegra_get_syncpt(struct drm_device *drm, void *data,
			    struct drm_file *file)
{
	struct tegra_drm_file *fpriv = file->driver_priv;
	struct drm_tegra_get_syncpt *args = data;
	struct tegra_drm_context *context;
	struct host1x_syncpt *syncpt;

	context = tegra_drm_get_context(args->context);

	if (!tegra_drm_file_owns_context(fpriv, context))
		return -ENODEV;

	if (args->index >= context->client->base.num_syncpts)
		return -EINVAL;

	syncpt = context->client->base.syncpts[args->index];
	args->id = host1x_syncpt_id(syncpt);

	return 0;
}
#endif

static int tegra_submit(struct drm_device *drm, void *data,
			struct drm_file *file)
{
	struct tegra_drm_file *fpriv = file->driver_priv;
	struct drm_tegra_submit *args = data;
	struct tegra_drm_context *context;

	context = tegra_drm_get_context(args->context);

	if (!tegra_drm_file_owns_context(fpriv, context))
		return -ENODEV;

	return context->client->ops->submit(context, args, drm, file);
}

#ifdef CONFIG_DRM_TEGRA_STAGING
static int tegra_get_syncpt_base(struct drm_device *drm, void *data,
				 struct drm_file *file)
{
	struct tegra_drm_file *fpriv = file->driver_priv;
	struct drm_tegra_get_syncpt_base *args = data;
	struct tegra_drm_context *context;
	struct host1x_syncpt_base *base;
	struct host1x_syncpt *syncpt;

	context = tegra_drm_get_context(args->context);

	if (!tegra_drm_file_owns_context(fpriv, context))
		return -ENODEV;

	if (args->syncpt >= context->client->base.num_syncpts)
		return -EINVAL;

	syncpt = context->client->base.syncpts[args->syncpt];

	base = host1x_syncpt_get_base(syncpt);
	if (!base)
		return -ENXIO;

	args->id = host1x_syncpt_base_id(base);

	return 0;
}
#endif

static int tegra_gem_set_tiling(struct drm_device *drm, void *data,
				struct drm_file *file)
{
	struct drm_tegra_gem_set_tiling *args = data;
	enum tegra_bo_tiling_mode mode;
	struct drm_gem_object *gem;
	unsigned long value = 0;
	struct tegra_bo *bo;

	switch (args->mode) {
	case DRM_TEGRA_GEM_TILING_MODE_PITCH:
		mode = TEGRA_BO_TILING_MODE_PITCH;

		if (args->value != 0)
			return -EINVAL;

		break;

	case DRM_TEGRA_GEM_TILING_MODE_TILED:
		mode = TEGRA_BO_TILING_MODE_TILED;

		if (args->value != 0)
			return -EINVAL;

		break;

	case DRM_TEGRA_GEM_TILING_MODE_BLOCK:
		mode = TEGRA_BO_TILING_MODE_BLOCK;

		if (args->value > 5)
			return -EINVAL;

		value = args->value;
		break;

	default:
		return -EINVAL;
	}

	gem = drm_gem_object_lookup(drm, file, args->handle);
	if (!gem)
		return -ENOENT;

	bo = to_tegra_bo(gem);

	bo->tiling.mode = mode;
	bo->tiling.value = value;

	drm_gem_object_unreference(gem);

	return 0;
}

static int tegra_gem_get_tiling(struct drm_device *drm, void *data,
				struct drm_file *file)
{
	struct drm_tegra_gem_get_tiling *args = data;
	struct drm_gem_object *gem;
	struct tegra_bo *bo;
	int err = 0;

	gem = drm_gem_object_lookup(drm, file, args->handle);
	if (!gem)
		return -ENOENT;

	bo = to_tegra_bo(gem);

	switch (bo->tiling.mode) {
	case TEGRA_BO_TILING_MODE_PITCH:
		args->mode = DRM_TEGRA_GEM_TILING_MODE_PITCH;
		args->value = 0;
		break;

	case TEGRA_BO_TILING_MODE_TILED:
		args->mode = DRM_TEGRA_GEM_TILING_MODE_TILED;
		args->value = 0;
		break;

	case TEGRA_BO_TILING_MODE_BLOCK:
		args->mode = DRM_TEGRA_GEM_TILING_MODE_BLOCK;
		args->value = bo->tiling.value;
		break;

	default:
		err = -EINVAL;
		break;
	}

	drm_gem_object_unreference(gem);

	return err;
}

static int tegra_gem_set_flags(struct drm_device *drm, void *data,
			       struct drm_file *file)
{
	struct drm_tegra_gem_set_flags *args = data;
	struct drm_gem_object *gem;
	struct tegra_bo *bo;

	if (args->flags & ~DRM_TEGRA_GEM_FLAGS)
		return -EINVAL;

	gem = drm_gem_object_lookup(drm, file, args->handle);
	if (!gem)
		return -ENOENT;

	bo = to_tegra_bo(gem);
	bo->flags = 0;

	if (args->flags & DRM_TEGRA_GEM_BOTTOM_UP)
		bo->flags |= TEGRA_BO_BOTTOM_UP;

	drm_gem_object_unreference(gem);

	return 0;
}

static int tegra_gem_get_flags(struct drm_device *drm, void *data,
			       struct drm_file *file)
{
	struct drm_tegra_gem_get_flags *args = data;
	struct drm_gem_object *gem;
	struct tegra_bo *bo;

	gem = drm_gem_object_lookup(drm, file, args->handle);
	if (!gem)
		return -ENOENT;

	bo = to_tegra_bo(gem);
	args->flags = 0;

	if (bo->flags & TEGRA_BO_BOTTOM_UP)
		args->flags |= DRM_TEGRA_GEM_BOTTOM_UP;

	drm_gem_object_unreference(gem);

	return 0;
}

static const struct drm_ioctl_desc tegra_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(TEGRA_GEM_CREATE, tegra_gem_create, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_GEM_MMAP, tegra_gem_mmap, DRM_UNLOCKED|DRM_RENDER_ALLOW),
#ifdef CONFIG_DRM_TEGRA_STAGING
	DRM_IOCTL_DEF_DRV(TEGRA_SYNCPT_READ, tegra_syncpt_read, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_SYNCPT_INCR, tegra_syncpt_incr, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_SYNCPT_WAIT, tegra_syncpt_wait, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_GET_SYNCPT, tegra_get_syncpt, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_GET_SYNCPT_BASE, tegra_get_syncpt_base, DRM_UNLOCKED|DRM_RENDER_ALLOW),
#endif
	DRM_IOCTL_DEF_DRV(TEGRA_OPEN_CHANNEL, tegra_open_channel, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_CLOSE_CHANNEL, tegra_close_channel, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_SUBMIT, tegra_submit, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_GEM_SET_TILING, tegra_gem_set_tiling, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_GEM_GET_TILING, tegra_gem_get_tiling, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_GEM_SET_FLAGS, tegra_gem_set_flags, DRM_UNLOCKED|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(TEGRA_GEM_GET_FLAGS, tegra_gem_get_flags, DRM_UNLOCKED|DRM_RENDER_ALLOW),
};

static const struct file_operations tegra_drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = tegra_drm_mmap,
	.poll = drm_poll,
	.read = drm_read,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.llseek = noop_llseek,
};

static struct drm_crtc *tegra_crtc_from_pipe(struct drm_device *drm, int pipe)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &drm->mode_config.crtc_list, head) {
		struct tegra_dc *dc = to_tegra_dc(crtc);

		if (dc->pipe == pipe)
			return crtc;
	}

	return NULL;
}

static u32 tegra_drm_get_vblank_counter(struct drm_device *dev, int crtc)
{
	/* TODO: implement real hardware counter using syncpoints */
	return drm_vblank_count(dev, crtc);
}

static int tegra_drm_enable_vblank(struct drm_device *drm, int pipe)
{
	struct drm_crtc *crtc = tegra_crtc_from_pipe(drm, pipe);
	struct tegra_dc *dc = to_tegra_dc(crtc);

	if (!crtc)
		return -ENODEV;

	tegra_dc_enable_vblank(dc);

	return 0;
}

static void tegra_drm_disable_vblank(struct drm_device *drm, int pipe)
{
	struct drm_crtc *crtc = tegra_crtc_from_pipe(drm, pipe);
	struct tegra_dc *dc = to_tegra_dc(crtc);

	if (crtc)
		tegra_dc_disable_vblank(dc);
}

static void tegra_drm_preclose(struct drm_device *drm, struct drm_file *file)
{
	struct tegra_drm_file *fpriv = file->driver_priv;
	struct tegra_drm_context *context, *tmp;
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &drm->mode_config.crtc_list, head)
		tegra_dc_cancel_page_flip(crtc, file);

	list_for_each_entry_safe(context, tmp, &fpriv->contexts, list)
		tegra_drm_context_free(context);

	kfree(fpriv);
}

#ifdef CONFIG_DEBUG_FS
static int tegra_debugfs_framebuffers(struct seq_file *s, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *)s->private;
	struct drm_device *drm = node->minor->dev;
	struct drm_framebuffer *fb;

	mutex_lock(&drm->mode_config.fb_lock);

	list_for_each_entry(fb, &drm->mode_config.fb_list, head) {
		seq_printf(s, "%3d: user size: %d x %d, depth %d, %d bpp, refcount %d\n",
			   fb->base.id, fb->width, fb->height, fb->depth,
			   fb->bits_per_pixel,
			   atomic_read(&fb->refcount.refcount));
	}

	mutex_unlock(&drm->mode_config.fb_lock);

	return 0;
}

static struct drm_info_list tegra_debugfs_list[] = {
	{ "framebuffers", tegra_debugfs_framebuffers, 0 },
};

static int tegra_debugfs_init(struct drm_minor *minor)
{
	return drm_debugfs_create_files(tegra_debugfs_list,
					ARRAY_SIZE(tegra_debugfs_list),
					minor->debugfs_root, minor);
}

static void tegra_debugfs_cleanup(struct drm_minor *minor)
{
	drm_debugfs_remove_files(tegra_debugfs_list,
				 ARRAY_SIZE(tegra_debugfs_list), minor);
}
#endif

static struct drm_driver tegra_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME | DRIVER_RENDER,
	.load = tegra_drm_load,
	.unload = tegra_drm_unload,
	.open = tegra_drm_open,
	.preclose = tegra_drm_preclose,
	.lastclose = tegra_drm_lastclose,

	.get_vblank_counter = tegra_drm_get_vblank_counter,
	.enable_vblank = tegra_drm_enable_vblank,
	.disable_vblank = tegra_drm_disable_vblank,

#if defined(CONFIG_DEBUG_FS)
	.debugfs_init = tegra_debugfs_init,
	.debugfs_cleanup = tegra_debugfs_cleanup,
#endif

	.gem_free_object = tegra_bo_free_object,
	.gem_vm_ops = &tegra_bo_vm_ops,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = tegra_gem_prime_export,
	.gem_prime_import = tegra_gem_prime_import,

	.dumb_create = tegra_bo_dumb_create,
	.dumb_map_offset = tegra_bo_dumb_map_offset,
	.dumb_destroy = drm_gem_dumb_destroy,

	.ioctls = tegra_drm_ioctls,
	.num_ioctls = ARRAY_SIZE(tegra_drm_ioctls),
	.fops = &tegra_drm_fops,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

int tegra_drm_register_client(struct tegra_drm *tegra,
			      struct tegra_drm_client *client)
{
	mutex_lock(&tegra->clients_lock);
	list_add_tail(&client->list, &tegra->clients);
	mutex_unlock(&tegra->clients_lock);

	return 0;
}

int tegra_drm_unregister_client(struct tegra_drm *tegra,
				struct tegra_drm_client *client)
{
	mutex_lock(&tegra->clients_lock);
	list_del_init(&client->list);
	mutex_unlock(&tegra->clients_lock);

	return 0;
}

static const struct of_device_id host1x_drm_subdevs[] = {
	{ .compatible = "nvidia,tegra20-dc", },
	{ .compatible = "nvidia,tegra20-hdmi", },
	{ .compatible = "nvidia,tegra20-gr2d", },
	{ .compatible = "nvidia,tegra20-gr3d", },
	{ .compatible = "nvidia,tegra30-dc", },
	{ .compatible = "nvidia,tegra30-hdmi", },
	{ .compatible = "nvidia,tegra30-gr2d", },
	{ .compatible = "nvidia,tegra30-gr3d", },
	{ .compatible = "nvidia,tegra114-dsi", },
	{ .compatible = "nvidia,tegra114-hdmi", },
	{ .compatible = "nvidia,tegra114-gr3d", },
	{ .compatible = "nvidia,tegra124-dc", },
	{ .compatible = "nvidia,tegra124-sor", },
	{ .compatible = "nvidia,tegra124-hdmi", },
	{ /* sentinel */ }
};

static int tegra_drm_platform_probe(struct platform_device *pdev)
{
	if (!drm_host1x_check_clients_probed()) {
		dev_info(&pdev->dev,
			"clients are still probing, wait until they're done.\n");
		return -EPROBE_DEFER;
	}

	return drm_platform_init(&tegra_drm_driver, pdev);
}

static int tegra_drm_platform_remove(struct platform_device *pdev)
{
	drm_platform_exit(&tegra_drm_driver, pdev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_drm_platform_suspend(struct device *dev)
{
	struct tegra_drm *tegra = dev_get_drvdata(dev);
	struct drm_device *drm = tegra->drm;
	struct drm_encoder *encoder;
	struct drm_crtc *crtc;
	struct drm_encoder_helper_funcs *encoder_funcs = NULL;
	struct drm_crtc_helper_funcs *crtc_funcs = NULL;

	drm_modeset_lock_all(drm);
	list_for_each_entry(encoder, &drm->mode_config.encoder_list, head) {
		if (!drm_helper_encoder_in_use(encoder))
			continue;

		encoder_funcs = encoder->helper_private;
		if (encoder_funcs->disable)
			(*encoder_funcs->disable)(encoder);

		list_for_each_entry(crtc, &drm->mode_config.crtc_list, head) {
			crtc_funcs = crtc->helper_private;
			if (encoder->crtc == crtc && crtc_funcs->disable)
				(*crtc_funcs->disable)(crtc);
		}
	}
	drm_modeset_unlock_all(drm);

	drm_kms_helper_poll_disable(drm);
	return 0;
}

static int tegra_drm_platform_resume(struct device *dev)
{
	struct tegra_drm *tegra = dev_get_drvdata(dev);
	struct drm_device *drm = tegra->drm;

	drm_modeset_lock_all(drm);
	drm_helper_resume_force_mode(drm);
	drm_modeset_unlock_all(drm);

	drm_kms_helper_poll_enable(drm);
	return 0;
}
#endif

static const struct dev_pm_ops tegra_drm_platform_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_drm_platform_suspend, tegra_drm_platform_resume)
};

static struct platform_driver tegra_drm_platform_driver = {
	.probe = tegra_drm_platform_probe,
	.remove = tegra_drm_platform_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra-drm",
		.pm = &tegra_drm_platform_pm_ops,
	},
};

static int __init host1x_drm_init(void)
{
	int err;

	/*
	 * Will get the last smmu AS here because smmu probes before tegradrm.
	 * And smmu driver will allocate (num_as - NUM_OF_RESERVED_AS) domains when probe.
	 */
	if (iommu_present(&platform_bus_type)) {
		tegradrm_iommu_domain = iommu_domain_alloc(&platform_bus_type);
		if (IS_ERR(tegradrm_iommu_domain)) {
			err = PTR_ERR(tegradrm_iommu_domain);
			return err;
		}
	}

	err = platform_driver_register(&tegra_dpaux_driver);
	if (err < 0)
		goto out;

	err = platform_driver_register(&tegra_dc_driver);
	if (err < 0)
		goto unregister_dpaux;

	if (err < 0)
		goto unregister_dc;

	err = platform_driver_register(&tegra_sor_driver);
	if (err < 0)
		goto unregister_dsi;

	err = platform_driver_register(&tegra_hdmi_driver);
	if (err < 0)
		goto unregister_sor;

	err = platform_driver_register(&tegra_drm_platform_driver);
	if (err < 0)
		goto unregister_hdmi;

	tegra_drm_pdev = platform_device_register_simple("tegra-drm", -1, NULL, 0);
	if (IS_ERR(tegra_drm_pdev)) {
		err = PTR_ERR(tegra_drm_pdev);
		goto unregister_platform;
	}

	return 0;

unregister_platform:
	platform_driver_unregister(&tegra_drm_platform_driver);
unregister_hdmi:
	platform_driver_unregister(&tegra_hdmi_driver);
unregister_sor:
	platform_driver_unregister(&tegra_sor_driver);
unregister_dsi:
unregister_dc:
	platform_driver_unregister(&tegra_dc_driver);
unregister_dpaux:
	platform_driver_unregister(&tegra_dpaux_driver);
out:
	return err;
}
module_init(host1x_drm_init);

static void __exit host1x_drm_exit(void)
{
	platform_driver_unregister(&tegra_hdmi_driver);
	platform_driver_unregister(&tegra_sor_driver);
	platform_driver_unregister(&tegra_dc_driver);
	platform_driver_unregister(&tegra_dpaux_driver);

	if (tegradrm_iommu_domain)
		iommu_domain_free(tegradrm_iommu_domain);
}
module_exit(host1x_drm_exit);

MODULE_AUTHOR("Thierry Reding <thierry.reding@avionic-design.de>");
MODULE_DESCRIPTION("NVIDIA Tegra DRM driver");
MODULE_LICENSE("GPL v2");
