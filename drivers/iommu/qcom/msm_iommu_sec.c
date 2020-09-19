/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/kmemleak.h>
#include <linux/dma-mapping.h>
#include <linux/dma-iommu.h>
#include <soc/qcom/scm.h>

#include <asm/cacheflush.h>
#include <asm/sizes.h>

#include <soc/qcom/secure_buffer.h>


#include "msm_iommu_perfmon.h"
#include "msm_iommu_hw-v1.h"
#include "msm_iommu_priv.h"
#include "qcom_iommu.h"
#include <trace/events/kmem.h>

/* bitmap of the page sizes currently supported */
#define MSM_IOMMU_PGSIZES	(SZ_4K | SZ_64K | SZ_1M | SZ_16M)

/* commands for SCM_SVC_MP */
#define IOMMU_SECURE_CFG	2
#define IOMMU_SECURE_PTBL_SIZE  3
#define IOMMU_SECURE_PTBL_INIT  4
#define IOMMU_SET_CP_POOL_SIZE	5
#define IOMMU_SECURE_MAP	6
#define IOMMU_SECURE_UNMAP      7
#define IOMMU_SECURE_MAP2 0x0B
#define IOMMU_SECURE_MAP2_FLAT 0x12
#define IOMMU_SECURE_UNMAP2 0x0C
#define IOMMU_SECURE_UNMAP2_FLAT 0x13
#define IOMMU_TLBINVAL_FLAG 0x00000001

/* commands for SCM_SVC_UTIL */
#define IOMMU_DUMP_SMMU_FAULT_REGS 0X0C
//#define SCM_SVC_MP		0xc

static struct iommu_access_ops *iommu_access_ops;
static int is_secure;

static const struct of_device_id msm_smmu_list[] = {
	{ .compatible = "qcom,msm-smmu-v1", },
	{ .compatible = "qcom,msm-smmu-v2", },
	{ }
};

struct msm_scm_paddr_list {
	phys_addr_t list;
	unsigned int list_size;
	unsigned int size;
};

struct msm_scm_mapping_info {
	unsigned int id;
	unsigned int ctx_id;
	unsigned int va;
	unsigned int size;
};

struct msm_scm_map2_req {
	struct msm_scm_paddr_list plist;
	struct msm_scm_mapping_info info;
	unsigned int flags;
};

struct msm_scm_unmap2_req {
	struct msm_scm_mapping_info info;
	unsigned int flags;
};

struct msm_cp_pool_size {
	uint32_t size;
	uint32_t spare;
};

#define NUM_DUMP_REGS 14
/*
 * some space to allow the number of registers returned by the secure
 * environment to grow
 */
#define WIGGLE_ROOM (NUM_DUMP_REGS * 2)
/* Each entry is a (reg_addr, reg_val) pair, hence the * 2 */
#define SEC_DUMP_SIZE ((NUM_DUMP_REGS * 2) + WIGGLE_ROOM)

struct msm_scm_fault_regs_dump {
	uint32_t dump_size;
	uint32_t dump_data[SEC_DUMP_SIZE];
} __aligned(PAGE_SIZE);

void msm_iommu_sec_set_access_ops(struct iommu_access_ops *access_ops)
{
	iommu_access_ops = access_ops;
}

static int msm_iommu_dump_fault_regs(int smmu_id, int cb_num,
				struct msm_scm_fault_regs_dump *regs)
{
	int ret, resp;
	struct scm_desc desc = {0};

	struct msm_scm_fault_regs_dump_req {
		uint32_t id;
		uint32_t cb_num;
		uint32_t buff;
		uint32_t len;
	} req_info;

	desc.args[0] = req_info.id = smmu_id;
	desc.args[1] = req_info.cb_num = cb_num;
	/* virt_to_phys(regs) may be greater than 4GB */
	req_info.buff = virt_to_phys(regs);
	desc.args[2] =  virt_to_phys(regs);
	desc.args[3] = req_info.len = sizeof(*regs);
	desc.arginfo = SCM_ARGS(4, SCM_VAL, SCM_VAL, SCM_RW, SCM_VAL);

	//__dma_flush_area(regs, sizeof(*regs));
	dmac_clean_range(regs, regs + 1);
	if (!is_scm_armv8())
		ret = scm_call(SCM_SVC_UTIL, IOMMU_DUMP_SMMU_FAULT_REGS,
			&req_info, sizeof(req_info), &resp, 1);
	else
		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_UTIL,
			IOMMU_DUMP_SMMU_FAULT_REGS), &desc);
	dmac_inv_range(regs, regs + 1);
	//__dma_flush_area(regs, sizeof(*regs));

	return ret;
}

static int msm_iommu_reg_dump_to_regs(
	struct msm_iommu_context_reg ctx_regs[],
	struct msm_scm_fault_regs_dump *dump, struct msm_iommu_drvdata *drvdata,
	struct msm_iommu_ctx_drvdata *ctx_drvdata)
{
	int i, j, ret = 0;
	const uint32_t nvals = (dump->dump_size / sizeof(uint32_t));
	uint32_t *it = (uint32_t *) dump->dump_data;
	const uint32_t * const end = ((uint32_t *) dump) + nvals;
	phys_addr_t phys_base = drvdata->phys_base;
	int ctx = ctx_drvdata->num;

	if (!nvals)
		return -EINVAL;

	for (i = 1; it < end; it += 2, i += 2) {
		unsigned int reg_offset;
		uint32_t addr	= *it;
		uint32_t val	= *(it + 1);
		struct msm_iommu_context_reg *reg = NULL;
		if (addr < phys_base) {
			pr_err("Bogus-looking register (0x%x) for Iommu with base at %pa. Skipping.\n",
				addr, &phys_base);
			continue;
		}
		reg_offset = addr - phys_base;

		for (j = 0; j < MAX_DUMP_REGS; ++j) {
			struct dump_regs_tbl_entry dump_reg = dump_regs_tbl[j];
			void *test_reg;
			unsigned int test_offset;
			switch (dump_reg.dump_reg_type) {
			case DRT_CTX_REG:
				test_reg = CTX_REG(dump_reg.reg_offset,
					drvdata->cb_base, ctx);
				break;
			case DRT_GLOBAL_REG:
				test_reg = GLB_REG(
					dump_reg.reg_offset, drvdata->glb_base);
				break;
			case DRT_GLOBAL_REG_N:
				test_reg = GLB_REG_N(
					drvdata->glb_base, ctx,
					dump_reg.reg_offset);
				break;
			default:
				pr_err("Unknown dump_reg_type: 0x%x\n",
					dump_reg.dump_reg_type);
				BUG();
				break;
			}
			test_offset = test_reg - drvdata->glb_base;
			if (test_offset == reg_offset) {
				reg = &ctx_regs[j];
				break;
			}
		}

		if (reg == NULL) {
			pr_debug("Unknown register in secure CB dump: %x\n",
				addr);
			continue;
		}

		if (reg->valid) {
			WARN(1, "Invalid (repeated?) register in CB dump: %x\n",
				addr);
			continue;
		}

		reg->val = val;
		reg->valid = true;
	}

	if (i != nvals) {
		pr_err("Invalid dump! %d != %d\n", i, nvals);
		ret = 1;
	}

	for (i = 0; i < MAX_DUMP_REGS; ++i) {
		if (!ctx_regs[i].valid) {
			if (dump_regs_tbl[i].must_be_present) {
				pr_err("Register missing from dump for ctx %d: %s, 0x%x\n",
					ctx,
					dump_regs_tbl[i].name,
					dump_regs_tbl[i].reg_offset);
				ret = 1;
			}
			ctx_regs[i].val = 0xd00dfeed;
		}
	}

	return ret;
}

irqreturn_t msm_iommu_secure_fault_handler_v2(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct msm_iommu_drvdata *drvdata;
	struct msm_iommu_ctx_drvdata *ctx_drvdata;
	struct msm_scm_fault_regs_dump *regs;
	int tmp, ret = IRQ_HANDLED;

	iommu_access_ops->iommu_lock_acquire(0);

	BUG_ON(!pdev);

	drvdata = dev_get_drvdata(pdev->dev.parent);
	BUG_ON(!drvdata);

	ctx_drvdata = dev_get_drvdata(&pdev->dev);
	BUG_ON(!ctx_drvdata);

	regs = kzalloc(sizeof(*regs), GFP_ATOMIC);
	if (!regs) {
		pr_err("%s: Couldn't allocate memory\n", __func__);
		goto lock_release;
	}

	if (!drvdata->ctx_attach_count) {
		pr_err("Unexpected IOMMU page fault from secure context bank!\n");
		pr_err("name = %s\n", drvdata->name);
		pr_err("Power is OFF. Unable to read page fault information\n");
		/*
		 * We cannot determine which context bank caused the issue so
		 * we just return handled here to ensure IRQ handler code is
		 * happy
		 */
		goto free_regs;
	}

	tmp = msm_iommu_dump_fault_regs(drvdata->sec_id,
					ctx_drvdata->num, regs);

	if (tmp) {
		pr_err("%s: Couldn't dump fault registers (%d) %s, ctx: %d\n",
			__func__, tmp, drvdata->name, ctx_drvdata->num);
		goto free_regs;
	} else {
		struct msm_iommu_context_reg ctx_regs[MAX_DUMP_REGS];
		memset(ctx_regs, 0, sizeof(ctx_regs));
		tmp = msm_iommu_reg_dump_to_regs(
			ctx_regs, regs, drvdata, ctx_drvdata);
		if (tmp < 0) {
			ret = IRQ_NONE;
			pr_err("Incorrect response from secure environment\n");
			goto free_regs;
		}

		if (ctx_regs[DUMP_REG_FSR].val) {
			if (tmp)
				pr_err("Incomplete fault register dump. Printout will be incomplete.\n");
			if (!ctx_drvdata->attached_domain) {
				pr_err("Bad domain in interrupt handler\n");
				tmp = -ENOSYS;
			} else {
				tmp = report_iommu_fault(
					ctx_drvdata->attached_domain,
					&ctx_drvdata->pdev->dev,
					COMBINE_DUMP_REG(
						ctx_regs[DUMP_REG_FAR1].val,
						ctx_regs[DUMP_REG_FAR0].val),
					0);
			}

			/* if the fault wasn't handled by someone else: */
			if (tmp == -ENOSYS) {
				pr_err("Unexpected IOMMU page fault from secure context bank!\n");
				pr_err("name = %s\n", drvdata->name);
				pr_err("context = %s (%d)\n", ctx_drvdata->name,
					ctx_drvdata->num);
				pr_err("Interesting registers:\n");
				print_ctx_regs(ctx_regs);
			}
		} else {
			ret = IRQ_NONE;
		}
	}
free_regs:
	kfree(regs);
lock_release:
	iommu_access_ops->iommu_lock_release(0);
	return ret;
}

int msm_iommu_sec_program_iommu(struct msm_iommu_drvdata *drvdata,
			struct msm_iommu_ctx_drvdata *ctx_drvdata)
{
	int ret, scm_ret;

	if (drvdata->smmu_local_base) {
		writel_relaxed(0xFFFFFFFF,
			       drvdata->smmu_local_base + SMMU_INTR_SEL_NS);
		mb();
	}

	ret = scm_restore_sec_cfg(drvdata->sec_id, ctx_drvdata->num, &scm_ret);
	if (ret || scm_ret) {
		pr_err("scm call IOMMU_SECURE_CFG failed\n");
		return ret ? ret : -EINVAL;
	}

	return ret;
}

static int msm_iommu_sec_map2(struct msm_scm_map2_req *map)
{
	struct scm_desc desc = {0};
	u32 resp, flags;
	int ret;

	flags = 0;

	desc.args[0] = map->plist.list;
	desc.args[1] = map->plist.list_size;
	desc.args[2] = map->plist.size;
	desc.args[3] = map->info.id;
	desc.args[4] = map->info.ctx_id;
	desc.args[5] = map->info.va;
	desc.args[6] = map->info.size;
	desc.args[7] = flags;
	desc.arginfo = SCM_ARGS(8, SCM_RW, SCM_VAL, SCM_VAL, SCM_VAL, SCM_VAL,
				SCM_VAL, SCM_VAL, SCM_VAL);
	if (!is_scm_armv8()) {
		ret = scm_call(SCM_SVC_MP, IOMMU_SECURE_MAP2, map, sizeof(*map),
				&resp, sizeof(resp));
	} else {
		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_MP,
				IOMMU_SECURE_MAP2_FLAT), &desc);
		resp = desc.ret[0];
	}
	if (ret || resp)
		return -EINVAL;

	return 0;
}

static int msm_iommu_sec_ptbl_map(struct msm_iommu_drvdata *iommu_drvdata,
			struct msm_iommu_ctx_drvdata *ctx_drvdata,
			unsigned long va, phys_addr_t pa, size_t len)
{
	struct msm_scm_map2_req map;
	void *flush_va, *flush_va_end;
	int ret = 0;

	if (!IS_ALIGNED(va, SZ_1M) || !IS_ALIGNED(len, SZ_1M) ||
		!IS_ALIGNED(pa, SZ_1M))
		return -EINVAL;
	map.plist.list = virt_to_phys(&pa);
	map.plist.list_size = 1;
	map.plist.size = len;
	map.info.id = iommu_drvdata->sec_id;
	map.info.ctx_id = ctx_drvdata->num;
	map.info.va = va;
	map.info.size = len;

	flush_va = &pa;
	flush_va_end = (void *)
		(((unsigned long) flush_va) + sizeof(phys_addr_t));

	/*
	 * Ensure that the buffer is in RAM by the time it gets to TZ
	 */
	//__dma_flush_area(flush_va, sizeof(phys_addr_t));
	dmac_clean_range(flush_va, flush_va_end);

	ret = msm_iommu_sec_map2(&map);
	if (ret)
		return -EINVAL;

	return 0;
}

#if 0
static unsigned int get_phys_addr(struct scatterlist *sg)
{
	/*
	 * Try sg_dma_address first so that we can
	 * map carveout regions that do not have a
	 * struct page associated with them.
	 */
	unsigned int pa = sg_dma_address(sg);
	if (pa == 0)
		pa = sg_phys(sg);
	return pa;
}

static int msm_iommu_sec_ptbl_map_range(struct msm_iommu_drvdata *iommu_drvdata,
			struct msm_iommu_ctx_drvdata *ctx_drvdata,
			unsigned long va, struct scatterlist *sg, size_t len)
{
	struct scatterlist *sgiter;
	struct msm_scm_map2_req map;
	unsigned int *pa_list = 0;
	unsigned int pa, cnt;
	void *flush_va, *flush_va_end;
	unsigned int offset = 0, chunk_offset = 0;
	int ret;

	if (!IS_ALIGNED(va, SZ_1M) || !IS_ALIGNED(len, SZ_1M))
		return -EINVAL;

	map.info.id = iommu_drvdata->sec_id;
	map.info.ctx_id = ctx_drvdata->num;
	map.info.va = va;
	map.info.size = len;

	if (sg->length == len) {
		/*
		 * physical address for secure mapping needs
		 * to be 1MB aligned
		 */
		pa = get_phys_addr(sg);
		if (!IS_ALIGNED(pa, SZ_1M))
			return -EINVAL;
		map.plist.list = virt_to_phys(&pa);
		map.plist.list_size = 1;
		map.plist.size = len;
		flush_va = &pa;
	} else {
		sgiter = sg;
		if (!IS_ALIGNED(sgiter->length, SZ_1M))
			return -EINVAL;
		cnt = sg->length / SZ_1M;
		while ((sgiter = sg_next(sgiter))) {
			if (!IS_ALIGNED(sgiter->length, SZ_1M))
				return -EINVAL;
			cnt += sgiter->length / SZ_1M;
		}

		pa_list = kmalloc(cnt * sizeof(*pa_list), GFP_KERNEL);
		if (!pa_list)
			return -ENOMEM;

		sgiter = sg;
		cnt = 0;
		pa = get_phys_addr(sgiter);
		if (!IS_ALIGNED(pa, SZ_1M)) {
			kfree(pa_list);
			return -EINVAL;
		}
		while (offset < len) {
			pa += chunk_offset;
			pa_list[cnt] = pa;
			chunk_offset += SZ_1M;
			offset += SZ_1M;
			cnt++;

			if (chunk_offset >= sgiter->length && offset < len) {
				chunk_offset = 0;
				sgiter = sg_next(sgiter);
				pa = get_phys_addr(sgiter);
			}
		}

		map.plist.list = virt_to_phys(pa_list);
		map.plist.list_size = cnt;
		map.plist.size = SZ_1M;
		flush_va = pa_list;
	}

	/*
	 * Ensure that the buffer is in RAM by the time it gets to TZ
	 */
	flush_va_end = (void *) (((unsigned long) flush_va) +
			(map.plist.list_size * sizeof(*pa_list)));
	//__dma_flush_area(flush_va, (map.plist.list_size * sizeof(*pa_list)));
	dmac_clean_range(flush_va, flush_va_end);

	ret = msm_iommu_sec_map2(&map);
	kfree(pa_list);

	return ret;
}
#endif
static int msm_iommu_sec_ptbl_unmap(struct msm_iommu_drvdata *iommu_drvdata,
				    struct msm_iommu_ctx_drvdata *ctx_drvdata,
				    unsigned long va, size_t len)
{
	struct msm_scm_unmap2_req unmap;
	int ret, scm_ret;
	struct scm_desc desc = {0};

	if (!IS_ALIGNED(va, SZ_1M) || !IS_ALIGNED(len, SZ_1M))
		return -EINVAL;

	desc.args[0] = unmap.info.id = iommu_drvdata->sec_id;
	desc.args[1] = unmap.info.ctx_id = ctx_drvdata->num;
	desc.args[2] = unmap.info.va = va;
	desc.args[3] = unmap.info.size = len;
	desc.args[4] = unmap.flags = IOMMU_TLBINVAL_FLAG;
	desc.arginfo = SCM_ARGS(5);

	if (!is_scm_armv8())
		ret = scm_call(SCM_SVC_MP, IOMMU_SECURE_UNMAP2, &unmap,
				sizeof(unmap), &scm_ret, sizeof(scm_ret));
	else
		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_MP,
				IOMMU_SECURE_UNMAP2_FLAT), &desc);

	return ret;
}

static struct iommu_domain *  msm_iommu_domain_alloc(unsigned type)
{
	struct msm_iommu_priv *priv;
	struct iommu_domain *domain;

	if (type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	INIT_LIST_HEAD(&priv->list_attached);
	domain = &priv->domain;
	return domain;
}

static void msm_iommu_domain_free(struct iommu_domain *domain)
{
	struct msm_iommu_priv *priv;

	iommu_access_ops->iommu_lock_acquire(0);
	priv = to_msm_priv(domain);

	kfree(priv);
	iommu_access_ops->iommu_lock_release(0);
}

static int msm_iommu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	struct msm_iommu_priv *priv;
	struct msm_iommu_drvdata *iommu_drvdata;
	struct msm_iommu_ctx_drvdata *ctx_drvdata;
	struct msm_iommu_ctx_drvdata *tmp_drvdata;
	int ret = 0;

	iommu_access_ops->iommu_lock_acquire(0);

	priv = to_msm_priv(domain);
	if (!priv || !dev) {
		ret = -EINVAL;
		goto fail;
	}

	if (!(priv->client_name))
		priv->client_name = dev_name(dev);

	iommu_drvdata = dev_get_drvdata(dev->parent);
	ctx_drvdata = dev_get_drvdata(dev);
	if (!iommu_drvdata || !ctx_drvdata) {
		ret = -EINVAL;
		goto fail;
	}

	if (!list_empty(&ctx_drvdata->attached_elm)) {
		ret = -EBUSY;
		goto fail;
	}

	list_for_each_entry(tmp_drvdata, &priv->list_attached, attached_elm)
		if (tmp_drvdata == ctx_drvdata) {
			ret = -EBUSY;
			goto fail;
		}

	ret = iommu_access_ops->iommu_power_on(iommu_drvdata);
	if (ret)
		goto fail;

	/* We can only do this once */
	if (!iommu_drvdata->ctx_attach_count) {
		ret = msm_iommu_sec_program_iommu(iommu_drvdata,
						ctx_drvdata);

		/* bfb settings are always programmed by HLOS */
		program_iommu_bfb_settings(iommu_drvdata->base,
					   iommu_drvdata->bfb_settings);
	}

	list_add(&(ctx_drvdata->attached_elm), &priv->list_attached);
	ctx_drvdata->attached_domain = domain;
	++iommu_drvdata->ctx_attach_count;

	iommu_access_ops->iommu_lock_release(0);

	msm_iommu_attached(dev->parent);
	return ret;
fail:
	iommu_access_ops->iommu_lock_release(0);
	return ret;
}

static void msm_iommu_detach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	struct msm_iommu_drvdata *iommu_drvdata;
	struct msm_iommu_ctx_drvdata *ctx_drvdata;

	if (!dev)
		return;

	msm_iommu_detached(dev->parent);

	iommu_access_ops->iommu_lock_acquire(0);

	iommu_drvdata = dev_get_drvdata(dev->parent);
	ctx_drvdata = dev_get_drvdata(dev);
	if (!iommu_drvdata || !ctx_drvdata || !ctx_drvdata->attached_domain)
		goto fail;

	list_del_init(&ctx_drvdata->attached_elm);
	ctx_drvdata->attached_domain = NULL;

	iommu_access_ops->iommu_power_off(iommu_drvdata);
	BUG_ON(iommu_drvdata->ctx_attach_count == 0);
	--iommu_drvdata->ctx_attach_count;
fail:
	iommu_access_ops->iommu_lock_release(0);
}

static int get_drvdata(struct iommu_domain *domain,
			struct msm_iommu_drvdata **iommu_drvdata,
			struct msm_iommu_ctx_drvdata **ctx_drvdata)
{
	struct msm_iommu_priv *priv = to_msm_priv(domain);
	struct msm_iommu_ctx_drvdata *ctx;

	list_for_each_entry(ctx, &priv->list_attached, attached_elm) {
		if (ctx->attached_domain == domain)
			break;
	}

	if (ctx->attached_domain != domain)
		return -EINVAL;

	*ctx_drvdata = ctx;
	*iommu_drvdata = dev_get_drvdata(ctx->pdev->dev.parent);
	return 0;
}

static int msm_iommu_map(struct iommu_domain *domain, unsigned long va,
			 phys_addr_t pa, size_t len, int prot)
{
	struct msm_iommu_drvdata *iommu_drvdata;
	struct msm_iommu_ctx_drvdata *ctx_drvdata;
	int ret = 0;

	iommu_access_ops->iommu_lock_acquire(0);

	ret = get_drvdata(domain, &iommu_drvdata, &ctx_drvdata);
	if (ret)
		goto fail;

	ret = msm_iommu_sec_ptbl_map(iommu_drvdata, ctx_drvdata,
					va, pa, len);
fail:
	iommu_access_ops->iommu_lock_release(0);
	return ret;
}

static size_t msm_iommu_unmap(struct iommu_domain *domain, unsigned long va,
			    size_t len)
{
	struct msm_iommu_drvdata *iommu_drvdata;
	struct msm_iommu_ctx_drvdata *ctx_drvdata;
	int ret = -ENODEV;

	iommu_access_ops->iommu_lock_acquire(0);

	ret = get_drvdata(domain, &iommu_drvdata, &ctx_drvdata);
	if (ret)
		goto fail;

	ret = msm_iommu_sec_ptbl_unmap(iommu_drvdata, ctx_drvdata,
					va, len);
fail:
	iommu_access_ops->iommu_lock_release(0);

	/* the IOMMU API requires us to return how many bytes were unmapped */
	len = ret ? 0 : len;
	return len;
}

#if 0
static int msm_iommu_map_range(struct iommu_domain *domain, unsigned int va,
			       struct scatterlist *sg, unsigned int len,
			       int prot)
{
	int ret;
	struct msm_iommu_drvdata *iommu_drvdata;
	struct msm_iommu_ctx_drvdata *ctx_drvdata;

	iommu_access_ops->iommu_lock_acquire(0);

	ret = get_drvdata(domain, &iommu_drvdata, &ctx_drvdata);
	if (ret)
		goto fail;
	ret = msm_iommu_sec_ptbl_map_range(iommu_drvdata, ctx_drvdata,
						va, sg, len);
fail:
	iommu_access_ops->iommu_lock_release(0);
	return ret;
}


static int msm_iommu_unmap_range(struct iommu_domain *domain, unsigned int va,
				 unsigned int len)
{
	struct msm_iommu_drvdata *iommu_drvdata;
	struct msm_iommu_ctx_drvdata *ctx_drvdata;
	int ret = -EINVAL;

	if (!IS_ALIGNED(va, SZ_1M) || !IS_ALIGNED(len, SZ_1M))
		return -EINVAL;

	iommu_access_ops->iommu_lock_acquire(0);

	ret = get_drvdata(domain, &iommu_drvdata, &ctx_drvdata);
	if (ret)
		goto fail;

	ret = msm_iommu_sec_ptbl_unmap(iommu_drvdata, ctx_drvdata, va, len);

fail:
	iommu_access_ops->iommu_lock_release(0);
	return ret ? ret : 0;
}
#endif
static phys_addr_t msm_iommu_iova_to_phys(struct iommu_domain *domain,
					  phys_addr_t va)
{
	return 0;
}

void msm_iommu_check_scm_call_avail(void)
{
	is_secure = scm_is_call_available(SCM_SVC_MP, IOMMU_SECURE_CFG);
}

int msm_iommu_get_scm_call_avail(void)
{
	return is_secure;
}

static int msm_iommu_domain_set_attr(struct iommu_domain *domain,
				enum iommu_attr attr, void *data)
{
	switch (attr) {
	case DOMAIN_ATTR_SECURE_VMID:
		/*
		 * Not supported on MMU-500 driver as we are on preconfigured
		 * secure context banks where the secure VMID is already set
		 * from bootloader MMU initialization.
		 * Also, the TZ in MSM SoC using this driver will not accept
		 * hypervisor SCM calls which would be needed to change the
		 * secure VMID mapping in the IOMMU!
		 *
		 * Note: This is valid for both secure and non-secure IOMMU.
		 */
		break;
	case DOMAIN_ATTR_ATOMIC:
		/* 
		 * Map / unmap in legacy driver are by default atomic. So
		 * we don't need to do anything here.
		 */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int msm_iommu_domain_get_attr(struct iommu_domain *domain,
				enum iommu_attr attr, void *data)
{
	struct msm_iommu_priv *priv = to_msm_priv(domain);
	struct msm_iommu_ctx_drvdata *ctx_drvdata = NULL;

	if (!list_empty(&priv->list_attached))
		ctx_drvdata = list_first_entry(&priv->list_attached,
			struct msm_iommu_ctx_drvdata, attached_elm);

	switch (attr) {
	case DOMAIN_ATTR_SECURE_VMID:
		*((int *) data) = -VMID_INVAL;
		break;
	case DOMAIN_ATTR_CONTEXT_BANK:
		if (!ctx_drvdata)
			return -ENODEV;

		*((unsigned int *) data) = ctx_drvdata->num;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct iommu_ops msm_iommu_ops = {
	.domain_alloc = msm_iommu_domain_alloc,
	.domain_free = msm_iommu_domain_free,
	.attach_dev = msm_iommu_attach_dev,
	.detach_dev = msm_iommu_detach_dev,
	.map = msm_iommu_map,
	.unmap = msm_iommu_unmap,
/*	.map_range = msm_iommu_map_range,*/
	.map_sg = default_iommu_map_sg,
/*	.unmap_range = msm_iommu_unmap_range,*/
	.iova_to_phys = msm_iommu_iova_to_phys,
	.domain_set_attr = msm_iommu_domain_set_attr,
	.domain_get_attr = msm_iommu_domain_get_attr,
	.pgsize_bitmap = MSM_IOMMU_PGSIZES,
	.dma_supported = msm_iommu_dma_supported,
};

static int __init msm_iommu_sec_init(void)
{
	int ret;

	ret = bus_register(&msm_iommu_sec_bus_type);
	if (ret)
		return ret;

	ret = bus_set_iommu(&msm_iommu_sec_bus_type, &msm_iommu_ops);
	if (ret) {
		bus_unregister(&msm_iommu_sec_bus_type);
		return ret;
	}

	return 0;
}

subsys_initcall(msm_iommu_sec_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM SMMU Secure Driver");
