/*
 * drivers/video/tegra/host/dmabuf.c
 *
 * Tegra Graphics Host DMA-BUF support
 *
 * Copyright (c) 2012-2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/dma-buf.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <linux/slab.h>
#include "chip_support.h"
#include "nvhost_allocator.h"
#include "nvhost_memmgr.h"
#include "../nvmap/nvmap_ioctl.h"

struct nvhost_dmabuf_data {
	struct nvhost_comptags comptags;
	struct nvhost_allocator *comptag_allocator;
};

static inline struct dma_buf_attachment *to_dmabuf_att(struct mem_handle *h)
{
#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	struct dma_buf *dmabuf = (struct dma_buf *) h;

	return nvmap_foreign_dmabuf_get_att(dmabuf);
#else
	return (struct dma_buf_attachment *)h;
#endif
}

static inline struct dma_buf *to_dmabuf(struct mem_handle *h)
{
	return to_dmabuf_att(h)->dmabuf;
}

struct mem_handle *nvhost_dmabuf_alloc(size_t size, size_t align, int flags)
{
	/* TODO: Add allocation via DMA Mapping API */
	WARN_ON(1);
	return NULL;
}

void nvhost_dmabuf_put(struct mem_handle *handle)
{
#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	struct dma_buf_attachment *attach = to_dmabuf_att(handle);
	struct dma_buf *dmabuf = attach->dmabuf;

	nvmap_foreign_dmabuf_put(dmabuf);
	dma_buf_detach(dmabuf, attach);
	dma_buf_put(dmabuf);
#endif
}

/* Callback from the DMA-BUF exporter's release function */
static void delete_drvdata(void *_drvdata)
{
	struct nvhost_dmabuf_data *drvdata = _drvdata;

	if (WARN_ON(!drvdata))
		return;

	if (drvdata->comptags.lines) {
		if (WARN_ON(!drvdata->comptag_allocator))
			goto free;
		drvdata->comptag_allocator->free(drvdata->comptag_allocator,
					      drvdata->comptags.offset,
					      drvdata->comptags.lines);
	}

free:
	kfree(drvdata);
}

struct sg_table *nvhost_dmabuf_pin(struct mem_handle *handle,
				struct device *dev)
{
	struct dma_buf_attachment *attach = to_dmabuf_att(handle);
	struct dma_buf *dmabuf = attach->dmabuf;
	struct nvhost_dmabuf_data *drvdata;
	int err;

	drvdata = dma_buf_get_drvdata(dmabuf, dev);

	if (IS_ERR(drvdata))
		return (void *)drvdata;

	/* We shouldn't have an existing drvdata */
	if (drvdata)
		return ERR_PTR(-EINVAL);

	/* Allocate drvdata struct */
	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return ERR_PTR(-ENOMEM);

	err = dma_buf_set_drvdata(dmabuf, dev, drvdata, delete_drvdata);
	if (err) {
		kfree(drvdata);
		return ERR_PTR(err);
	}

	/* Use attachment's priv to store device information */
	attach->priv = dev;

	return dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
}

void nvhost_dmabuf_unpin(struct mem_handle *handle, struct sg_table *sgt)
{
	struct dma_buf_attachment *attach = to_dmabuf_att(handle);

	dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
}


void *nvhost_dmabuf_mmap(struct mem_handle *handle)
{
	return dma_buf_vmap(to_dmabuf(handle));
}

void nvhost_dmabuf_munmap(struct mem_handle *handle, void *addr)
{
	dma_buf_vunmap(to_dmabuf(handle), addr);
}

void *nvhost_dmabuf_kmap(struct mem_handle *handle, unsigned int pagenum)
{
	return dma_buf_kmap(to_dmabuf(handle), pagenum);
}

void nvhost_dmabuf_kunmap(struct mem_handle *handle, unsigned int pagenum,
		void *addr)
{
	dma_buf_kunmap(to_dmabuf(handle), pagenum, addr);
}

struct mem_handle *nvhost_dmabuf_get(ulong id, struct platform_device *dev)
{
#ifdef CONFIG_NVMAP_USE_FD_FOR_HANDLE
	struct dma_buf *dmabuf;
	struct mem_handle *h;

	dmabuf = dma_buf_get(id);
	h = (struct mem_handle *)dmabuf;
	nvmap_foreign_dmabuf_get(dmabuf);
	return h;
#else
	WARN_ON(1);
#endif
}

int nvhost_dmabuf_get_param(struct mem_mgr *memmgr, struct mem_handle *handle,
			    u32 param, u64 *result)
{
	struct dma_buf_attachment *attach = to_dmabuf_att(handle);

	switch(param)
	{
		case NVMAP_HANDLE_PARAM_SIZE:
			*result = attach->dmabuf->size;
			return 0;
		case NVMAP_HANDLE_PARAM_ALIGNMENT:
			if (attach->dmabuf->size & (128 * 1024 - 1))
				*result = 4096;
			else
				*result = 128 * 1024;
			return 0;
		case NVMAP_HANDLE_PARAM_BASE:
			*result = 0;
			return 0;
		case NVMAP_HANDLE_PARAM_HEAP:
			*result = NVMAP_HEAP_IOVMM;
			return 0;
		case NVMAP_HANDLE_PARAM_KIND:
			/*
			 * We probably need to change user space to set kind with
			 * NVMAP_IOC_ALLOC_KIND ioctl before we call this.
			 */
			*result = 0xfe;
			return 0;
	}

	printk(KERN_ERR"Unimplemented parameter %d handle %p\n",param,handle);
	return -EINVAL;
}

void nvhost_dmabuf_get_comptags(struct mem_handle *handle,
				struct nvhost_comptags *comptags)
{
	struct dma_buf_attachment *attach = to_dmabuf_att(handle);
	struct dma_buf *dmabuf = attach->dmabuf;
	struct device *dev = attach->priv;
	struct nvhost_dmabuf_data *drvdata;

	if (WARN_ON(!dev))
		return;

	drvdata = dma_buf_get_drvdata(dmabuf, dev);
	if (WARN_ON(IS_ERR(drvdata)) || WARN_ON(!drvdata))
		return;

	*comptags = drvdata->comptags;
}

int nvhost_dmabuf_alloc_comptags(struct mem_handle *handle,
				 struct nvhost_allocator *allocator,
				 int lines)
{
	struct dma_buf_attachment *attach = to_dmabuf_att(handle);
	struct dma_buf *dmabuf = attach->dmabuf;
	struct device *dev = attach->priv;
	struct nvhost_dmabuf_data *drvdata;
	u32 offset = 0;
	int err;

	if (WARN_ON(!lines))
		return -EINVAL;
	if (WARN_ON(!dev))
		return -EINVAL;

	drvdata = dma_buf_get_drvdata(dmabuf, dev);
	if (WARN_ON(IS_ERR(drvdata)) || WARN_ON(!drvdata))
		return -EINVAL;

	/* store the allocator so we can use it when we free the ctags */
	drvdata->comptag_allocator = allocator;
	err = allocator->alloc(allocator, &offset, lines);
	if (!err) {
		drvdata->comptags.lines = lines;
		drvdata->comptags.offset = offset;
	}

	return err;
}
