/*
 * IOMMU API for SMMU in Tegra30
 *
 * Copyright (c) 2011-2013, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define pr_fmt(fmt)	"%s(): " fmt, __func__

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_iommu.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/tegra-ahb.h>
#include <linux/syscore_ops.h>

#include <asm/page.h>
#include <asm/cacheflush.h>
#include <asm/dma-iommu.h>

#include <dt-bindings/memory/tegra-swgroup.h>

/* bitmap of the page sizes currently supported */
#define SMMU_IOMMU_PGSIZES	(SZ_4K)

#define MC_INTSTATUS			0x0
#define MC_INTMASK			0x4

#define MC_INT_ERR_SHIFT		6
#define MC_INT_ERR_MASK			(0x1f << MC_INT_ERR_SHIFT)
#define MC_INT_INVALID_SMMU_PAGE	BIT(MC_INT_ERR_SHIFT + 4)

#define MC_ERR_STATUS			0x8
#define MC_ERR_ADR			0xc

#define MC_ERR_TYPE_SHIFT		28
#define MC_ERR_TYPE_MASK		(0x7 << MC_ERR_TYPE_SHIFT)
#define MC_ERR_TYPE_INVALID_SMMU_PAGE	6

#define MC_ERR_RW_SHIFT			16
#define MC_ERR_RW			BIT(MC_ERR_RW_SHIFT)

#define MC_CLIENT_ID_MASK		0x7f

#define SMMU_CONFIG				0x10
#define SMMU_CONFIG_DISABLE			0
#define SMMU_CONFIG_ENABLE			1

/* REVISIT: To support multiple MCs */
enum {
	_MC = 0,
};

enum {
	_TLB = 0,
	_PTC,
};

#define SMMU_CACHE_CONFIG_BASE			0x14
#define __SMMU_CACHE_CONFIG(mc, cache)		(SMMU_CACHE_CONFIG_BASE + 4 * cache)
#define SMMU_CACHE_CONFIG(cache)		__SMMU_CACHE_CONFIG(_MC, cache)

#define SMMU_CACHE_CONFIG_STATS_SHIFT		31
#define SMMU_CACHE_CONFIG_STATS_ENABLE		(1 << SMMU_CACHE_CONFIG_STATS_SHIFT)
#define SMMU_CACHE_CONFIG_STATS_TEST_SHIFT	30
#define SMMU_CACHE_CONFIG_STATS_TEST		(1 << SMMU_CACHE_CONFIG_STATS_TEST_SHIFT)

#define SMMU_TLB_CONFIG_HIT_UNDER_MISS__ENABLE	(1 << 29)
#define SMMU_TLB_CONFIG_RESET_VAL		0x20000000
#define SMMU_TLB_RR_ARB				(1 << 28)

#define SMMU_PTC_CONFIG_CACHE__ENABLE		(1 << 29)
#define SMMU_PTC_CONFIG_INDEX_MAP__PATTERN	0x3f
#define SMMU_PTC_CONFIG_RESET_VAL		0x2000003f
#define SMMU_PTC_REQ_LIMIT			(8 << 24)

#define SMMU_PTB_ASID				0x1c
#define SMMU_PTB_ASID_CURRENT_SHIFT		0

#define SMMU_PTB_DATA				0x20
#define SMMU_PTB_DATA_RESET_VAL			0
#define SMMU_PTB_DATA_ASID_NONSECURE_SHIFT	29
#define SMMU_PTB_DATA_ASID_WRITABLE_SHIFT	30
#define SMMU_PTB_DATA_ASID_READABLE_SHIFT	31

#define SMMU_TLB_FLUSH				0x30
#define SMMU_TLB_FLUSH_VA_MATCH_ALL		0
#define SMMU_TLB_FLUSH_VA_MATCH_SECTION		2
#define SMMU_TLB_FLUSH_VA_MATCH_GROUP		3
#define SMMU_TLB_FLUSH_ASID_SHIFT_BASE		31
#define SMMU_TLB_FLUSH_ASID_MATCH_DISABLE	0
#define SMMU_TLB_FLUSH_ASID_MATCH_ENABLE	1
#define SMMU_TLB_FLUSH_ASID_MATCH_SHIFT		31

#define SMMU_TLB_FLUSH_ASID_SHIFT(as)					\
	(SMMU_TLB_FLUSH_ASID_SHIFT_BASE - __ffs((as)->smmu->num_as))

#define SMMU_PTC_FLUSH				0x34
#define SMMU_PTC_FLUSH_TYPE_ALL			0
#define SMMU_PTC_FLUSH_TYPE_ADR			1
#define SMMU_PTC_FLUSH_ADR_SHIFT		4

#define SMMU_PTC_FLUSH_1			0x9b8

#define SMMU_ASID_SECURITY			0x38
#define SMMU_ASID_SECURITY_1			0x3c
#define SMMU_ASID_SECURITY_2			0x9e0
#define SMMU_ASID_SECURITY_3			0x9e4
#define SMMU_ASID_SECURITY_4			0x9e8
#define SMMU_ASID_SECURITY_5			0x9ec
#define SMMU_ASID_SECURITY_6			0x9f0
#define SMMU_ASID_SECURITY_7			0x9f4

#define SMMU_STATS_CACHE_COUNT_BASE		0x1f0

#define SMMU_STATS_CACHE_COUNT(mc, cache, hitmiss)		\
	(SMMU_STATS_CACHE_COUNT_BASE + 8 * cache + 4 * hitmiss)

#define SMMU_TRANSLATION_ENABLE_0		0x228

#define SMMU_AFI_ASID	0x238   /* PCIE */
#define SMMU_ASID_BASE	SMMU_AFI_ASID

#define SMMU_PDE_NEXT_SHIFT		28

#define SMMU_TLB_FLUSH_VA_SECTION__MASK		0xffc00000
#define SMMU_TLB_FLUSH_VA_SECTION__SHIFT	12 /* right shift */
#define SMMU_TLB_FLUSH_VA_GROUP__MASK		0xffffc000
#define SMMU_TLB_FLUSH_VA_GROUP__SHIFT		12 /* right shift */
#define SMMU_TLB_FLUSH_VA(iova, which)	\
	((((iova) & SMMU_TLB_FLUSH_VA_##which##__MASK) >> \
		SMMU_TLB_FLUSH_VA_##which##__SHIFT) |	\
	SMMU_TLB_FLUSH_VA_MATCH_##which)
#define SMMU_PTB_ASID_CUR(n)	\
		((n) << SMMU_PTB_ASID_CURRENT_SHIFT)
#define SMMU_TLB_FLUSH_ASID_MATCH_disable		\
		(SMMU_TLB_FLUSH_ASID_MATCH_DISABLE <<	\
			SMMU_TLB_FLUSH_ASID_MATCH_SHIFT)
#define SMMU_TLB_FLUSH_ASID_MATCH__ENABLE		\
		(SMMU_TLB_FLUSH_ASID_MATCH_ENABLE <<	\
			SMMU_TLB_FLUSH_ASID_MATCH_SHIFT)

#define SMMU_PAGE_SHIFT 12
#define SMMU_PAGE_SIZE	(1 << SMMU_PAGE_SHIFT)
#define SMMU_PAGE_MASK	((1 << SMMU_PAGE_SHIFT) - 1)

#define SMMU_PDIR_COUNT	1024
#define SMMU_PDIR_SIZE	(sizeof(unsigned long) * SMMU_PDIR_COUNT)
#define SMMU_PTBL_COUNT	1024
#define SMMU_PTBL_SIZE	(sizeof(unsigned long) * SMMU_PTBL_COUNT)
#define SMMU_PDIR_SHIFT	12
#define SMMU_PDE_SHIFT	12
#define SMMU_PTE_SHIFT	12
#define SMMU_PFN_MASK	0x0fffffff

#define SMMU_ADDR_TO_PFN(addr)	((addr) >> 12)
#define SMMU_ADDR_TO_PDN(addr)	((addr) >> 22)
#define SMMU_PDN_TO_ADDR(pdn)	((pdn) << 22)

#define _READABLE	(1 << SMMU_PTB_DATA_ASID_READABLE_SHIFT)
#define _WRITABLE	(1 << SMMU_PTB_DATA_ASID_WRITABLE_SHIFT)
#define _NONSECURE	(1 << SMMU_PTB_DATA_ASID_NONSECURE_SHIFT)
#define _PDE_NEXT	(1 << SMMU_PDE_NEXT_SHIFT)
#define _MASK_ATTR	(_READABLE | _WRITABLE | _NONSECURE)

#define _PDIR_ATTR	(_READABLE | _WRITABLE | _NONSECURE)

#define _PDE_ATTR	(_READABLE | _WRITABLE | _NONSECURE)
#define _PDE_ATTR_N	(_PDE_ATTR | _PDE_NEXT)
#define _PDE_VACANT(pdn)	(0)

#define _PTE_ATTR	(_READABLE | _WRITABLE | _NONSECURE)
#define _PTE_VACANT(addr)	(0)

#define SMMU_MK_PDIR(page, attr)	\
		((page_to_phys(page) >> SMMU_PDIR_SHIFT) | (attr))
#define SMMU_MK_PDE(page, attr)		\
		(unsigned long)((page_to_phys(page) >> SMMU_PDE_SHIFT) | (attr))
#define SMMU_EX_PTBL_PAGE(pde)		\
		pfn_to_page((unsigned long)(pde) & SMMU_PFN_MASK)
#define SMMU_PFN_TO_PTE(pfn, attr)	(unsigned long)((pfn) | (attr))

#define SMMU_ASID_ENABLE(asid)	((asid) | (1 << 31))
#define SMMU_ASID_DISABLE	0
#define SMMU_ASID_ASID(n)	((n) & ~SMMU_ASID_ENABLE(0))

#define NUM_SMMU_REG_BANKS	3

#define smmu_client_enable_swgroups(c, m) smmu_client_set_swgroups(c, m, 1)
#define smmu_client_disable_swgroups(c) smmu_client_set_swgroups(c, 0, 0)
#define __smmu_client_enable_swgroups(c, m) __smmu_client_set_swgroups(c, m, 1)
#define __smmu_client_disable_swgroups(c) __smmu_client_set_swgroups(c, 0, 0)

/*
 * Per client for address space
 */
struct smmu_client {
	struct device_node	*of_node;
	struct rb_node		node;
	struct device		*dev;
	struct list_head	list;
	struct smmu_as		*as;
	unsigned long		swgroups[2];
};

/*
 * Per address space
 */
struct smmu_as {
	struct smmu_device	*smmu;	/* back pointer to container */
	unsigned int		asid;
	spinlock_t		lock;	/* for pagetable */
	struct page		*pdir_page;
	unsigned long		pdir_attr;
	unsigned long		pde_attr;
	unsigned long		pte_attr;
	unsigned int		*pte_count;

	struct list_head	client;
	spinlock_t		client_lock; /* for client list */
};

struct smmu_debugfs_info {
	struct smmu_device *smmu;
	int mc;
	int cache;
};

/*
 * Per SMMU device - IOMMU device
 */
struct smmu_device {
	struct iommu	iommu;

	void __iomem	*regbase;	/* register offset base */
	void __iomem	**regs;		/* register block start address array */
	void __iomem	**rege;		/* register block end address array */
	int		nregs;		/* number of register blocks */

	unsigned long	iovmm_base;	/* remappable base address */
	unsigned long	page_count;	/* total remappable size */
	spinlock_t	lock;
	char		*name;
	struct rb_root	clients;
	struct page *avp_vector_page;	/* dummy page shared by all AS's */

	int nr_xlats;		/* number of translation_enable registers */
	int nr_asid_secs;	/* number of asid_security registers */
	u32 tlb_reset;		/* TLB config reset value */
	u32 ptc_reset;		/* PTC config reset value */
	bool extended_pa;	/* supports > 32bit physical addresses */

	/*
	 * Register image savers for suspend/resume
	 */
	u32 *xlat;
	u32 *asid_sec;

	struct dentry *debugfs_root;
	struct smmu_debugfs_info *debugfs_info;

	struct device_node *ahb;

	struct dma_iommu_mapping **map;

	int		num_as;
	struct smmu_as	as[0];		/* Run-time allocated array */
};

struct smmu_platform_data {
	int asids;		/* number of asids */
	int nr_xlats;		/* number of translation_enable registers */
	int nr_asid_secs;	/* number of asid_security registers */
	u32 tlb_reset;		/* TLB config reset value */
	u32 ptc_reset;		/* PTC config reset value */
	bool extended_pa;	/* supports > 32bit physical addresses */
};

static struct smmu_device *smmu_handle; /* unique for a system */

/*
 *	SMMU register accessors
 */
static bool inline smmu_valid_reg(struct smmu_device *smmu,
				  void __iomem *addr)
{
	int i;

	for (i = 0; i < smmu->nregs; i++) {
		if (addr < smmu->regs[i])
			break;
		if (addr <= smmu->rege[i])
			return true;
	}

	return false;
}

static inline u32 smmu_read(struct smmu_device *smmu, size_t offs)
{
	void __iomem *addr = smmu->regbase + offs;

	BUG_ON(!smmu_valid_reg(smmu, addr));

	return readl(addr);
}

static inline void smmu_write(struct smmu_device *smmu, u32 val, size_t offs)
{
	void __iomem *addr = smmu->regbase + offs;

	BUG_ON(!smmu_valid_reg(smmu, addr));

	writel(val, addr);
}

#define VA_PAGE_TO_PA(va, page)	\
	(page_to_phys(page) + ((unsigned long)(va) & ~PAGE_MASK))

#define VA_PAGE_TO_PA_HI(va, page) (u32)((u64)page_to_phys(page) >> 32)

#define FLUSH_CPU_DCACHE(va, page, size)	\
	do {	\
		unsigned long _pa_ = VA_PAGE_TO_PA(va, page);		\
		__cpuc_flush_dcache_area((void *)(va), (size_t)(size));	\
		outer_flush_range(_pa_, _pa_+(size_t)(size));		\
	} while (0)

/*
 * Any interaction between any block on PPSB and a block on APB or AHB
 * must have these read-back barriers to ensure the APB/AHB bus
 * transaction is complete before initiating activity on the PPSB
 * block.
 */
#define FLUSH_SMMU_REGS(smmu)	smmu_read(smmu, SMMU_CONFIG)

static const u32 smmu_asid_sec_ofs[] = {
	SMMU_ASID_SECURITY,
	SMMU_ASID_SECURITY_1,
	SMMU_ASID_SECURITY_2,
	SMMU_ASID_SECURITY_3,
	SMMU_ASID_SECURITY_4,
	SMMU_ASID_SECURITY_5,
	SMMU_ASID_SECURITY_6,
	SMMU_ASID_SECURITY_7,
};

static size_t smmu_get_asid_offset(int id)
{
	switch (id) {
	case TEGRA_SWGROUP_DC14:
		return 0x490;
	case TEGRA_SWGROUP_DC12:
		return 0xa88;
	case TEGRA_SWGROUP_AFI...TEGRA_SWGROUP_ISP:
	case TEGRA_SWGROUP_MPE...TEGRA_SWGROUP_PPCS1:
		return (id - TEGRA_SWGROUP_AFI) * sizeof(u32) + SMMU_ASID_BASE;
	case TEGRA_SWGROUP_SDMMC1A...63:
		return (id - TEGRA_SWGROUP_SDMMC1A) * sizeof(u32) + 0xa94;
	};

	BUG();
}

static struct smmu_client *find_smmu_client(struct smmu_device *smmu,
					    struct device_node *dev_node)
{
	struct rb_node *node = smmu->clients.rb_node;

	while (node) {
		struct smmu_client *client;

		client = container_of(node, struct smmu_client, node);
		if (dev_node < client->of_node)
			node = node->rb_left;
		else if (dev_node > client->of_node)
			node = node->rb_right;
		else
			return client;
	}

	return NULL;
}

static int insert_smmu_client(struct smmu_device *smmu,
			      struct smmu_client *client)
{
	struct rb_node **new, *parent;

	new = &smmu->clients.rb_node;
	parent = NULL;
	while (*new) {
		struct smmu_client *this;
		this = container_of(*new, struct smmu_client, node);

		parent = *new;
		if (client->of_node < this->of_node)
			new = &((*new)->rb_left);
		else if (client->of_node > this->of_node)
			new = &((*new)->rb_right);
		else
			return -EEXIST;
	}

	rb_link_node(&client->node, parent, new);
	rb_insert_color(&client->node, &smmu->clients);
	return 0;
}

static int register_smmu_client(struct smmu_device *smmu,
				struct device *dev, unsigned long *swgroups)
{
	struct smmu_client *client;

	client = find_smmu_client(smmu, dev->of_node);
	if (client) {
		dev_err(dev,
			"rejecting multiple registrations for client device %s\n",
			dev->of_node->full_name);
		return -EBUSY;
	}

	client = devm_kzalloc(smmu->iommu.dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->dev = dev;
	client->of_node = dev->of_node;
	memcpy(client->swgroups, swgroups, sizeof(u64));
	return insert_smmu_client(smmu, client);
}

static int smmu_of_get_swgroups(struct device *dev, unsigned long *swgroups)
{
	struct of_phandle_args args;
	const __be32 *cur, *end;

	of_property_for_each_phandle_with_args(dev->of_node, "iommus",
					       "#iommu-cells", 0, args, cur, end) {
		if (args.np != smmu_handle->iommu.dev->of_node)
			continue;

		BUG_ON(args.args_count != 2);

		memcpy(swgroups, args.args, sizeof(u64));
		pr_debug("swgroups=%08lx %08lx ops=%p %s\n",
			 swgroups[0], swgroups[1],
			 dev->bus->iommu_ops, dev_name(dev));
		return 0;
	}

	return -ENODEV;
}

static int __smmu_client_set_swgroups(struct smmu_client *c,
				   unsigned long *map, int on)
{
	int i;
	struct smmu_as *as = c->as;
	u32 val, offs, mask = SMMU_ASID_ENABLE(as->asid);
	struct smmu_device *smmu = as->smmu;

	if (!on)
		map = c->swgroups;

	for_each_set_bit(i, map, TEGRA_SWGROUP_MAX) {
		offs = smmu_get_asid_offset(i);
		val = smmu_read(smmu, offs);
		if (on) {
			if (val) {
				if (WARN_ON(val != mask))
					return -EINVAL;
				continue;
			}

			val = mask;
			memcpy(c->swgroups, map, sizeof(u64));
		} else {
			WARN_ON((val & mask) == mask);
			val &= ~mask;
		}
		smmu_write(smmu, val, offs);
	}

	FLUSH_SMMU_REGS(smmu);
	return 0;
}

static int smmu_client_set_swgroups(struct smmu_client *c,
				 unsigned long *map, int on)
{
	int err;
	unsigned long flags;
	struct smmu_as *as = c->as;
	struct smmu_device *smmu = as->smmu;

	spin_lock_irqsave(&smmu->lock, flags);
	err = __smmu_client_set_swgroups(c, map, on);
	spin_unlock_irqrestore(&smmu->lock, flags);
	return err;
}

/*
 * Flush all TLB entries and all PTC entries
 * Caller must lock smmu
 */
static void smmu_flush_regs(struct smmu_device *smmu, int enable)
{
	u32 val;

	smmu_write(smmu, SMMU_PTC_FLUSH_TYPE_ALL, SMMU_PTC_FLUSH);
	FLUSH_SMMU_REGS(smmu);
	val = SMMU_TLB_FLUSH_VA_MATCH_ALL |
		SMMU_TLB_FLUSH_ASID_MATCH_disable;
	smmu_write(smmu, val, SMMU_TLB_FLUSH);

	if (enable)
		smmu_write(smmu, SMMU_CONFIG_ENABLE, SMMU_CONFIG);
	FLUSH_SMMU_REGS(smmu);
}

static int smmu_setup_regs(struct smmu_device *smmu)
{
	int i;
	u32 val;

	for (i = 0; i < smmu->num_as; i++) {
		struct smmu_as *as = &smmu->as[i];
		struct smmu_client *c;

		smmu_write(smmu, SMMU_PTB_ASID_CUR(as->asid), SMMU_PTB_ASID);
		val = as->pdir_page ?
			SMMU_MK_PDIR(as->pdir_page, as->pdir_attr) :
			SMMU_PTB_DATA_RESET_VAL;
		smmu_write(smmu, val, SMMU_PTB_DATA);

		list_for_each_entry(c, &as->client, list)
			__smmu_client_set_swgroups(c, c->swgroups, 1);
	}

	for (i = 0; i < smmu->nr_xlats; i++)
		smmu_write(smmu, smmu->xlat[i],
			   SMMU_TRANSLATION_ENABLE_0 + i * sizeof(u32));

	for (i = 0; i < smmu->nr_asid_secs; i++)
		smmu_write(smmu, smmu->asid_sec[i], smmu_asid_sec_ofs[i]);

	smmu_write(smmu, smmu->ptc_reset, SMMU_CACHE_CONFIG(_PTC));
	smmu_write(smmu, smmu->tlb_reset, SMMU_CACHE_CONFIG(_TLB));

	smmu_flush_regs(smmu, 1);

	if (smmu->ahb)
		return tegra_ahb_enable_smmu(smmu->ahb);

	return 0;
}

static void flush_ptc_by_addr(struct smmu_device *smmu, unsigned long *pte,
			      struct page *page)
{
	u32 val;

	if (smmu->extended_pa) {
		val = VA_PAGE_TO_PA_HI(pte, page);
		smmu_write(smmu, val, SMMU_PTC_FLUSH_1);
	}

	val = SMMU_PTC_FLUSH_TYPE_ADR | VA_PAGE_TO_PA(pte, page);
	smmu_write(smmu, val, SMMU_PTC_FLUSH);

	FLUSH_SMMU_REGS(smmu);
}

static void flush_ptc_and_tlb(struct smmu_device *smmu,
		      struct smmu_as *as, dma_addr_t iova,
		      unsigned long *pte, struct page *page, int is_pde)
{
	u32 val;
	unsigned long tlb_flush_va = is_pde
		?  SMMU_TLB_FLUSH_VA(iova, SECTION)
		:  SMMU_TLB_FLUSH_VA(iova, GROUP);

	flush_ptc_by_addr(smmu, pte, page);

	val = tlb_flush_va |
		SMMU_TLB_FLUSH_ASID_MATCH__ENABLE |
		(as->asid << SMMU_TLB_FLUSH_ASID_SHIFT(as));
	smmu_write(smmu, val, SMMU_TLB_FLUSH);
	FLUSH_SMMU_REGS(smmu);
}

static void free_ptbl(struct smmu_as *as, dma_addr_t iova)
{
	unsigned long pdn = SMMU_ADDR_TO_PDN(iova);
	unsigned long *pdir = (unsigned long *)page_address(as->pdir_page);

	if (pdir[pdn] != _PDE_VACANT(pdn)) {
		dev_dbg(as->smmu->iommu.dev, "pdn: %lx\n", pdn);

		ClearPageReserved(SMMU_EX_PTBL_PAGE(pdir[pdn]));
		__free_page(SMMU_EX_PTBL_PAGE(pdir[pdn]));
		pdir[pdn] = _PDE_VACANT(pdn);
		FLUSH_CPU_DCACHE(&pdir[pdn], as->pdir_page, sizeof pdir[pdn]);
		flush_ptc_and_tlb(as->smmu, as, iova, &pdir[pdn],
				  as->pdir_page, 1);
	}
}

static void free_pdir(struct smmu_as *as)
{
	unsigned addr;
	int count;
	struct device *dev = as->smmu->iommu.dev;

	if (!as->pdir_page)
		return;

	addr = as->smmu->iovmm_base;
	count = as->smmu->page_count;
	while (count-- > 0) {
		free_ptbl(as, addr);
		addr += SMMU_PAGE_SIZE * SMMU_PTBL_COUNT;
	}
	ClearPageReserved(as->pdir_page);
	__free_page(as->pdir_page);
	as->pdir_page = NULL;
	devm_kfree(dev, as->pte_count);
	as->pte_count = NULL;
}

/*
 * Maps PTBL for given iova and returns the PTE address
 * Caller must unmap the mapped PTBL returned in *ptbl_page_p
 */
static unsigned long *locate_pte(struct smmu_as *as,
				 dma_addr_t iova, bool allocate,
				 struct page **ptbl_page_p,
				 unsigned int **count)
{
	unsigned long ptn = SMMU_ADDR_TO_PFN(iova);
	unsigned long pdn = SMMU_ADDR_TO_PDN(iova);
	unsigned long *pdir = page_address(as->pdir_page);
	unsigned long *ptbl;

	if (pdir[pdn] != _PDE_VACANT(pdn)) {
		/* Mapped entry table already exists */
		*ptbl_page_p = SMMU_EX_PTBL_PAGE(pdir[pdn]);
		ptbl = page_address(*ptbl_page_p);
	} else if (!allocate) {
		return NULL;
	} else {
		int pn;
		unsigned long addr = SMMU_PDN_TO_ADDR(pdn);

		/* Vacant - allocate a new page table */
		dev_dbg(as->smmu->iommu.dev, "New PTBL pdn: %lx\n", pdn);

		*ptbl_page_p = alloc_page(GFP_ATOMIC);
		if (!*ptbl_page_p) {
			dev_err(as->smmu->iommu.dev,
				"failed to allocate smmu_device page table\n");
			return NULL;
		}
		SetPageReserved(*ptbl_page_p);
		ptbl = (unsigned long *)page_address(*ptbl_page_p);
		for (pn = 0; pn < SMMU_PTBL_COUNT;
		     pn++, addr += SMMU_PAGE_SIZE) {
			ptbl[pn] = _PTE_VACANT(addr);
		}
		FLUSH_CPU_DCACHE(ptbl, *ptbl_page_p, SMMU_PTBL_SIZE);
		pdir[pdn] = SMMU_MK_PDE(*ptbl_page_p,
					as->pde_attr | _PDE_NEXT);
		FLUSH_CPU_DCACHE(&pdir[pdn], as->pdir_page, sizeof pdir[pdn]);
		flush_ptc_and_tlb(as->smmu, as, iova, &pdir[pdn],
				  as->pdir_page, 1);
	}
	*count = &as->pte_count[pdn];

	return &ptbl[ptn % SMMU_PTBL_COUNT];
}

#ifdef CONFIG_SMMU_SIG_DEBUG
static void put_signature(struct smmu_as *as,
			  dma_addr_t iova, unsigned long pfn)
{
	struct page *page;
	unsigned long *vaddr;

	page = pfn_to_page(pfn);
	vaddr = page_address(page);
	if (!vaddr)
		return;

	vaddr[0] = iova;
	vaddr[1] = pfn << PAGE_SHIFT;
	FLUSH_CPU_DCACHE(vaddr, page, sizeof(vaddr[0]) * 2);
}
#else
static inline void put_signature(struct smmu_as *as,
				 unsigned long addr, unsigned long pfn)
{
}
#endif

/*
 * Caller must not hold as->lock
 */
static int alloc_pdir(struct smmu_as *as)
{
	unsigned long *pdir, flags;
	int pdn, err = 0;
	u32 val;
	struct smmu_device *smmu = as->smmu;
	struct page *page;
	unsigned int *cnt;

	/*
	 * do the allocation, then grab as->lock
	 */
	cnt = devm_kzalloc(smmu->iommu.dev,
			   sizeof(cnt[0]) * SMMU_PDIR_COUNT,
			   GFP_KERNEL);
	page = alloc_page(GFP_KERNEL | __GFP_DMA);

	spin_lock_irqsave(&as->lock, flags);

	if (as->pdir_page) {
		/* We raced, free the redundant */
		err = -EAGAIN;
		goto err_out;
	}

	if (!page || !cnt) {
		dev_err(smmu->iommu.dev,
			"failed to allocate at %s\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	as->pdir_page = page;
	as->pte_count = cnt;

	SetPageReserved(as->pdir_page);
	pdir = page_address(as->pdir_page);

	for (pdn = 0; pdn < SMMU_PDIR_COUNT; pdn++)
		pdir[pdn] = _PDE_VACANT(pdn);
	FLUSH_CPU_DCACHE(pdir, as->pdir_page, SMMU_PDIR_SIZE);

	flush_ptc_by_addr(as->smmu, pdir, page);

	val = SMMU_TLB_FLUSH_VA_MATCH_ALL |
		SMMU_TLB_FLUSH_ASID_MATCH__ENABLE |
		(as->asid << SMMU_TLB_FLUSH_ASID_SHIFT(as));
	smmu_write(smmu, val, SMMU_TLB_FLUSH);
	FLUSH_SMMU_REGS(as->smmu);

	spin_unlock_irqrestore(&as->lock, flags);

	return 0;

err_out:
	spin_unlock_irqrestore(&as->lock, flags);

	devm_kfree(smmu->iommu.dev, cnt);
	if (page)
		__free_page(page);
	return err;
}

static void __smmu_iommu_unmap(struct smmu_as *as, dma_addr_t iova)
{
	unsigned long *pte;
	struct page *page;
	unsigned int *count;

	pte = locate_pte(as, iova, false, &page, &count);
	if (WARN_ON(!pte))
		return;

	if (WARN_ON(*pte == _PTE_VACANT(iova)))
		return;

	*pte = _PTE_VACANT(iova);
	FLUSH_CPU_DCACHE(pte, page, sizeof(*pte));
	flush_ptc_and_tlb(as->smmu, as, iova, pte, page, 0);
	if (!--(*count))
		free_ptbl(as, iova);
}

static void __smmu_iommu_map_pfn(struct smmu_as *as, dma_addr_t iova,
				 unsigned long pfn)
{
	struct smmu_device *smmu = as->smmu;
	unsigned long *pte;
	unsigned int *count;
	struct page *page;

	pte = locate_pte(as, iova, true, &page, &count);
	if (WARN_ON(!pte))
		return;

	if (*pte == _PTE_VACANT(iova))
		(*count)++;
	*pte = SMMU_PFN_TO_PTE(pfn, as->pte_attr);
	FLUSH_CPU_DCACHE(pte, page, sizeof(*pte));
	flush_ptc_and_tlb(smmu, as, iova, pte, page, 0);
	put_signature(as, iova, pfn);
}

static int smmu_iommu_map(struct iommu_domain *domain, unsigned long iova,
			  phys_addr_t pa, size_t bytes, int prot)
{
	struct smmu_as *as = domain->priv;
	unsigned long pfn = __phys_to_pfn(pa);
	unsigned long flags;

	dev_dbg(as->smmu->iommu.dev, "[%d] %08lx:%pa\n", as->asid, iova, &pa);

	if (!pfn_valid(pfn))
		return -ENOMEM;

	spin_lock_irqsave(&as->lock, flags);
	__smmu_iommu_map_pfn(as, iova, pfn);
	spin_unlock_irqrestore(&as->lock, flags);
	return 0;
}

static size_t smmu_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
			       size_t bytes)
{
	struct smmu_as *as = domain->priv;
	unsigned long flags;

	dev_dbg(as->smmu->iommu.dev, "[%d] %08lx\n", as->asid, iova);

	spin_lock_irqsave(&as->lock, flags);
	__smmu_iommu_unmap(as, iova);
	spin_unlock_irqrestore(&as->lock, flags);
	return SMMU_PAGE_SIZE;
}

static phys_addr_t smmu_iommu_iova_to_phys(struct iommu_domain *domain,
					   dma_addr_t iova)
{
	struct smmu_as *as = domain->priv;
	unsigned long *pte;
	unsigned int *count;
	struct page *page;
	unsigned long pfn;
	unsigned long flags;

	spin_lock_irqsave(&as->lock, flags);

	pte = locate_pte(as, iova, true, &page, &count);
	pfn = *pte & SMMU_PFN_MASK;
	WARN_ON(!pfn_valid(pfn));
	dev_dbg(as->smmu->iommu.dev,
		"iova:%08llx pfn:%08lx asid:%d\n", (unsigned long long)iova,
		 pfn, as->asid);

	spin_unlock_irqrestore(&as->lock, flags);
	return PFN_PHYS(pfn);
}

static int smmu_iommu_domain_has_cap(struct iommu_domain *domain,
				     unsigned long cap)
{
	return 0;
}

static int smmu_iommu_attach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	struct smmu_as *as = domain->priv;
	struct smmu_device *smmu = as->smmu;
	struct smmu_client *client, *c;
	int err;

	client = find_smmu_client(smmu, dev->of_node);
	if (!client)
		return -ENOMEM;

	client->as = as;
	err = smmu_client_enable_swgroups(client, client->swgroups);
	if (err)
		return -EINVAL;

	spin_lock(&as->client_lock);
	list_for_each_entry(c, &as->client, list) {
		if (c->dev == dev) {
			dev_err(smmu->iommu.dev,
				"%s is already attached\n", dev_name(c->dev));
			err = -EINVAL;
			goto err_client;
		}
	}
	list_add(&client->list, &as->client);
	spin_unlock(&as->client_lock);

	/*
	 * Reserve "page zero" for AVP vectors using a common dummy
	 * page.
	 */
	if (test_bit(TEGRA_SWGROUP_AVPC, client->swgroups)) {
		struct page *page;

		page = as->smmu->avp_vector_page;
		__smmu_iommu_map_pfn(as, 0, page_to_pfn(page));

		pr_info("Reserve \"page zero\" for AVP vectors using a common dummy\n");
	}

	dev_dbg(smmu->iommu.dev, "%s is attached\n", dev_name(dev));
	return 0;

err_client:
	smmu_client_disable_swgroups(client);
	spin_unlock(&as->client_lock);
	return err;
}

static void smmu_iommu_detach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	struct smmu_as *as = domain->priv;
	struct smmu_device *smmu = as->smmu;
	struct smmu_client *c;

	spin_lock(&as->client_lock);

	list_for_each_entry(c, &as->client, list) {
		if (c->dev == dev) {
			smmu_client_disable_swgroups(c);
			list_del(&c->list);
			c->as = NULL;
			dev_dbg(smmu->iommu.dev,
				"%s is detached\n", dev_name(c->dev));
			goto out;
		}
	}
	dev_err(smmu->iommu.dev, "Couldn't find %s\n", dev_name(dev));
out:
	spin_unlock(&as->client_lock);
}

static int smmu_iommu_domain_init(struct iommu_domain *domain)
{
	int i, err = -EAGAIN;
	unsigned long flags;
	struct smmu_as *as;
	struct smmu_device *smmu = smmu_handle;

	/* Look for a free AS with lock held */
	for  (i = 0; i < smmu->num_as; i++) {
		as = &smmu->as[i];

		if (as->pdir_page)
			continue;

		err = alloc_pdir(as);
		if (!err)
			goto found;

		if (err != -EAGAIN)
			break;
	}
	if (i == smmu->num_as)
		dev_err(smmu->iommu.dev,  "no free AS\n");
	return err;

found:
	spin_lock_irqsave(&smmu->lock, flags);

	/* Update PDIR register */
	smmu_write(smmu, SMMU_PTB_ASID_CUR(as->asid), SMMU_PTB_ASID);
	smmu_write(smmu,
		   SMMU_MK_PDIR(as->pdir_page, as->pdir_attr), SMMU_PTB_DATA);
	FLUSH_SMMU_REGS(smmu);

	spin_unlock_irqrestore(&smmu->lock, flags);

	domain->priv = as;

	domain->geometry.aperture_start = smmu->iovmm_base;
	domain->geometry.aperture_end   = smmu->iovmm_base +
		smmu->page_count * SMMU_PAGE_SIZE - 1;
	domain->geometry.force_aperture = true;

	dev_dbg(smmu->iommu.dev, "smmu_as@%p\n", as);

	return 0;
}

static void smmu_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct smmu_as *as = domain->priv;
	struct smmu_device *smmu = as->smmu;
	unsigned long flags;

	spin_lock_irqsave(&as->lock, flags);

	if (as->pdir_page) {
		spin_lock(&smmu->lock);
		smmu_write(smmu, SMMU_PTB_ASID_CUR(as->asid), SMMU_PTB_ASID);
		smmu_write(smmu, SMMU_PTB_DATA_RESET_VAL, SMMU_PTB_DATA);
		FLUSH_SMMU_REGS(smmu);
		spin_unlock(&smmu->lock);

		free_pdir(as);
	}

	if (!list_empty(&as->client)) {
		struct smmu_client *c;

		list_for_each_entry(c, &as->client, list)
			smmu_iommu_detach_dev(domain, c->dev);
	}

	spin_unlock_irqrestore(&as->lock, flags);

	domain->priv = NULL;
	dev_dbg(smmu->iommu.dev, "smmu_as@%p\n", as);
}

/*
 * ASID[0] for the system default
 * ASID[1] for PPCS("AHB bus children"), which has SDMMC
 * ASID[2][3].. open for drivers, first come, first served.
 */
enum {
	SYSTEM_DEFAULT,
	SYSTEM_PROTECTED,
	SYSTEM_AVPC,
	NUM_OF_STATIC_MAPS,
};

int tegra_smmu_get_asid(struct device *dev)
{
	int err;
	unsigned long swgroups[2];

	err = smmu_of_get_swgroups(dev, swgroups);
	if (err) {
		WARN(1, "Get asid for device: %s failed.\n", dev_name(dev));
		return -ENODEV;
	}

	if (test_bit(TEGRA_SWGROUP_PPCS, swgroups))
		return SYSTEM_PROTECTED;
	else if (test_bit(TEGRA_SWGROUP_AVPC, swgroups))
		return SYSTEM_AVPC;

	return SYSTEM_DEFAULT;
}
EXPORT_SYMBOL(tegra_smmu_get_asid);

static int smmu_iommu_bound_driver(struct device *dev)
{
	int err;
	unsigned long swgroups[2];
	struct dma_iommu_mapping *map = NULL;

	err = smmu_of_get_swgroups(dev, swgroups);
	if (err)
		return -ENODEV;

	if (!find_smmu_client(smmu_handle, dev->of_node)) {
		err = register_smmu_client(smmu_handle, dev, swgroups);
		if (err) {
			dev_err(dev, "failed to add client %s\n",
				dev_name(dev));
			return -EINVAL;
		}
	}

	if (test_bit(TEGRA_SWGROUP_PPCS, swgroups))
		map = smmu_handle->map[SYSTEM_PROTECTED];
	else if (test_bit(TEGRA_SWGROUP_AVPC, swgroups))
		map = smmu_handle->map[SYSTEM_AVPC];
	else
		map = smmu_handle->map[SYSTEM_DEFAULT];

	if (map)
		err = arm_iommu_attach_device(dev, map);
	else
		return -ENODEV;

	pr_debug("swgroups=%08lx %08lx map=%p err=%d %s\n",
		 swgroups[0], swgroups[1], map, err, dev_name(dev));
	return err;
}

static void smmu_iommu_unbind_driver(struct device *dev)
{
	dev_dbg(dev, "Detaching from map %p\n", to_dma_iommu_mapping(dev));
	arm_iommu_detach_device(dev);
}

static struct iommu_ops smmu_iommu_ops = {
	.domain_init	= smmu_iommu_domain_init,
	.domain_destroy	= smmu_iommu_domain_destroy,
	.attach_dev	= smmu_iommu_attach_dev,
	.detach_dev	= smmu_iommu_detach_dev,
	.map		= smmu_iommu_map,
	.unmap		= smmu_iommu_unmap,
	.iova_to_phys	= smmu_iommu_iova_to_phys,
	.domain_has_cap	= smmu_iommu_domain_has_cap,
	.bound_driver	= smmu_iommu_bound_driver,
	.unbind_driver	= smmu_iommu_unbind_driver,
	.pgsize_bitmap	= SMMU_IOMMU_PGSIZES,
};

static const char * const tegra_mc_clients[] = {
	"csr_ptcr",
	"csr_display0a",
	"csr_display0ab",
	"csr_display0b",
	"csr_display0bb",
	"csr_display0c",
	"csr_display0cb",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"csr_afir",
	"csr_avpcarm7r",
	"csr_displayhc",
	"csr_displayhcb",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"csr_hdar",
	"csr_host1xdmar",
	"csr_host1xr",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"csr_msencsrd",
	"csr_ppcsahbdmar",
	"csr_ppcsahbslvr",
	"csr_satar",
	"dummy_client",
	"dummy_client",
	"csr_vdebsevr",
	"csr_vdember",
	"csr_vdemcer",
	"csr_vdetper",
	"csr_mpcorelpr",
	"csr_mpcorer",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"csw_msencswr",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"csw_afiw",
	"csw_avpcarm7w",
	"dummy_client",
	"dummy_client",
	"csw_hdaw",
	"csw_host1xw",
	"dummy_client",
	"csw_mpcorelpw",
	"csw_mpcorew",
	"dummy_client",
	"csw_ppcsahbdmaw",
	"csw_ppcsahbslvw",
	"csw_sataw",
	"csw_vdebsevw",
	"csw_vdedbgw",
	"csw_vdembew",
	"csw_vdetpmw",
	"dummy_client",
	"dummy_client",
	"csr_ispra",
	"dummy_client",
	"csw_ispwa",
	"csw_ispwb",
	"dummy_client",
	"dummy_client",
	"csr_xusb_hostr",
	"csw_xusb_hostw",
	"csr_xusb_devr",
	"csw_xusb_devw",
	"csr_isprab",
	"dummy_client",
	"csw_ispwab",
	"csw_ispwbb",
	"dummy_client",
	"dummy_client",
	"csr_tsecsrd",
	"csw_tsecswr",
	"csr_a9avpscr",
	"csw_a9avpscw",
	"csr_gpusrd",
	"csw_gpuswr",
	"csr_displayt",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"csr_sdmmcra",
	"csr_sdmmcraa",
	"csr_sdmmcr",
	"csr_sdmmcrab",
	"csw_sdmmcwa",
	"csw_sdmmcwaa",
	"csw_sdmmcw",
	"csw_sdmmcwab",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"csr_vicsrd",
	"csw_vicswr",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"dummy_client",
	"csw_viw",
	"csr_displayd",
	"dummy_client",
};

static irqreturn_t tegra_smmu_irq(int irq, void *data)
{
	struct device *dev = data;
	struct smmu_device *smmu = dev_get_drvdata(dev);
	u32 stat, mask, err, type;

	stat = smmu_read(smmu, MC_INTSTATUS);
	mask = smmu_read(smmu, MC_INTMASK);
	mask &= stat;
	if (!mask)
		return IRQ_NONE;

	err = smmu_read(smmu, MC_ERR_STATUS);
	type = (err & MC_ERR_TYPE_MASK) >> MC_ERR_TYPE_SHIFT;
	if (type == MC_ERR_TYPE_INVALID_SMMU_PAGE) {
		u32 cid, addr;

		cid = err & MC_CLIENT_ID_MASK;
		cid = min(cid, ARRAY_SIZE(tegra_mc_clients) - 1);
		addr = smmu_read(smmu, MC_ERR_ADR);

		dev_err_ratelimited(dev,
				    "SMMU %s fault at 0x%08x (client: %s, error: 0x%08x)\n",
				    (err & MC_ERR_RW) ? "write" : "read", addr,
				    tegra_mc_clients[cid], err);
	}

	smmu_write(smmu, stat, MC_INTSTATUS);
	return IRQ_HANDLED;
}

/* Should be in the order of enum */
static const char * const smmu_debugfs_mc[] = { "mc", };
static const char * const smmu_debugfs_cache[] = {  "tlb", "ptc", };

static ssize_t smmu_debugfs_stats_write(struct file *file,
					const char __user *buffer,
					size_t count, loff_t *pos)
{
	struct smmu_debugfs_info *info;
	struct smmu_device *smmu;
	int i;
	enum {
		_OFF = 0,
		_ON,
		_RESET,
	};
	const char * const command[] = {
		[_OFF]		= "off",
		[_ON]		= "on",
		[_RESET]	= "reset",
	};
	char str[] = "reset";
	u32 val;
	size_t offs;

	count = min_t(size_t, count, sizeof(str));
	if (copy_from_user(str, buffer, count))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(command); i++)
		if (strncmp(str, command[i],
			    strlen(command[i])) == 0)
			break;

	if (i == ARRAY_SIZE(command))
		return -EINVAL;

	info = file_inode(file)->i_private;
	smmu = info->smmu;

	offs = SMMU_CACHE_CONFIG(info->cache);
	val = smmu_read(smmu, offs);
	switch (i) {
	case _OFF:
		val &= ~SMMU_CACHE_CONFIG_STATS_ENABLE;
		val &= ~SMMU_CACHE_CONFIG_STATS_TEST;
		smmu_write(smmu, val, offs);
		break;
	case _ON:
		val |= SMMU_CACHE_CONFIG_STATS_ENABLE;
		val &= ~SMMU_CACHE_CONFIG_STATS_TEST;
		smmu_write(smmu, val, offs);
		break;
	case _RESET:
		val |= SMMU_CACHE_CONFIG_STATS_TEST;
		smmu_write(smmu, val, offs);
		val &= ~SMMU_CACHE_CONFIG_STATS_TEST;
		smmu_write(smmu, val, offs);
		break;
	default:
		BUG();
		break;
	}

	dev_dbg(smmu->iommu.dev, "%s() %08x, %08x @%08x\n", __func__,
		val, smmu_read(smmu, offs), offs);

	return count;
}

static int smmu_debugfs_stats_show(struct seq_file *s, void *v)
{
	struct smmu_debugfs_info *info = s->private;
	struct smmu_device *smmu = info->smmu;
	int i;
	const char * const stats[] = { "hit", "miss", };


	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		u32 val;
		size_t offs;

		offs = SMMU_STATS_CACHE_COUNT(info->mc, info->cache, i);
		val = smmu_read(smmu, offs);
		seq_printf(s, "%s:%08x ", stats[i], val);

		dev_dbg(smmu->iommu.dev, "%s() %s %08x @%08x\n", __func__,
			stats[i], val, offs);
	}
	seq_printf(s, "\n");
	return 0;
}

static int smmu_debugfs_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, smmu_debugfs_stats_show, inode->i_private);
}

static const struct file_operations smmu_debugfs_stats_fops = {
	.open		= smmu_debugfs_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= smmu_debugfs_stats_write,
};

static void smmu_debugfs_delete(struct smmu_device *smmu)
{
	debugfs_remove_recursive(smmu->debugfs_root);
	kfree(smmu->debugfs_info);
}

static void smmu_debugfs_create(struct smmu_device *smmu)
{
	int i;
	size_t bytes;
	struct dentry *root;

	bytes = ARRAY_SIZE(smmu_debugfs_mc) * ARRAY_SIZE(smmu_debugfs_cache) *
		sizeof(*smmu->debugfs_info);
	smmu->debugfs_info = kmalloc(bytes, GFP_KERNEL);
	if (!smmu->debugfs_info)
		return;

	root = debugfs_create_dir(dev_name(smmu->iommu.dev), NULL);
	if (!root)
		goto err_out;
	smmu->debugfs_root = root;

	for (i = 0; i < ARRAY_SIZE(smmu_debugfs_mc); i++) {
		int j;
		struct dentry *mc;

		mc = debugfs_create_dir(smmu_debugfs_mc[i], root);
		if (!mc)
			goto err_out;

		for (j = 0; j < ARRAY_SIZE(smmu_debugfs_cache); j++) {
			struct dentry *cache;
			struct smmu_debugfs_info *info;

			info = smmu->debugfs_info;
			info += i * ARRAY_SIZE(smmu_debugfs_mc) + j;
			info->smmu = smmu;
			info->mc = i;
			info->cache = j;

			cache = debugfs_create_file(smmu_debugfs_cache[j],
						    S_IWUGO | S_IRUGO, mc,
						    (void *)info,
						    &smmu_debugfs_stats_fops);
			if (!cache)
				goto err_out;
		}
	}

	return;

err_out:
	smmu_debugfs_delete(smmu);
}

static int tegra_smmu_suspend(void)
{
	int i;
	struct smmu_device *smmu = smmu_handle;

	for (i = 0; i < smmu->nr_xlats; i++)
		smmu->xlat[i] = smmu_read(smmu,
				SMMU_TRANSLATION_ENABLE_0 + i * sizeof(u32));

	for (i = 0; i < smmu->nr_asid_secs; i++)
		smmu->asid_sec[i] =
			smmu_read(smmu, smmu_asid_sec_ofs[i]);
	return 0;
}

static void tegra_smmu_resume(void)
{
	struct smmu_device *smmu = smmu_handle;
	unsigned long flags;

	spin_lock_irqsave(&smmu->lock, flags);
	smmu_setup_regs(smmu);
	spin_unlock_irqrestore(&smmu->lock, flags);
}

static struct syscore_ops tegra_smmu_syscore_ops = {
	.suspend	= tegra_smmu_suspend,
	.resume		= tegra_smmu_resume,
};

static void tegra_smmu_create_default_map(struct smmu_device *smmu)
{
	int i;

	for (i = 0; i < smmu->num_as; i++) {
		dma_addr_t base = smmu->iovmm_base;
		size_t size = smmu->page_count << PAGE_SHIFT;

		smmu->map[i] = arm_iommu_create_mapping(&platform_bus_type,
							base, size, 0);
		if (IS_ERR(smmu->map[i]))
			dev_err(smmu->iommu.dev,
				"Couldn't create: asid=%d map=%p %pa-%pa\n",
				i, smmu->map[i], &base, &base + size - 1);
	}
}

static const struct smmu_platform_data tegra124_smmu_pdata = {
	.asids = 128,
	.nr_xlats = 4,
	.nr_asid_secs = 8,
	.tlb_reset = SMMU_TLB_CONFIG_RESET_VAL | SMMU_TLB_RR_ARB | 0x20,
	.ptc_reset = SMMU_PTC_CONFIG_RESET_VAL | SMMU_PTC_REQ_LIMIT,
	.extended_pa = true,
};

static struct of_device_id tegra_smmu_of_match[] = {
	{ .compatible = "nvidia,tegra124-smmu", .data = &tegra124_smmu_pdata, },
	{ .compatible = "nvidia,tegra30-smmu", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_smmu_of_match);

static int tegra_smmu_probe(struct platform_device *pdev)
{
	struct smmu_device *smmu;
	struct device *dev = &pdev->dev;
	int i, asids, irq, err = 0;
	dma_addr_t uninitialized_var(base);
	size_t bytes, uninitialized_var(size);
	const struct of_device_id *match;
	const struct smmu_platform_data *pdata;
	int nr_xlats, nr_asid_secs;

	if (smmu_handle)
		return -EIO;

	BUILD_BUG_ON(PAGE_SHIFT != SMMU_PAGE_SHIFT);

	match = of_match_device(tegra_smmu_of_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	pdata = match->data;
	nr_xlats = (pdata && pdata->nr_xlats) ?	pdata->nr_xlats : 3;
	nr_asid_secs = (pdata && pdata->nr_asid_secs) ?	pdata->nr_asid_secs : 1;

	if (of_property_read_u32(dev->of_node, "nvidia,#asids", &asids))
		asids = (pdata && pdata->asids) ? pdata->asids : 4;
	if (asids < NUM_OF_STATIC_MAPS)
		return -EINVAL;

	bytes = sizeof(*smmu) + asids * (sizeof(*smmu->as) +
					 sizeof(struct dma_iommu_mapping *));
	bytes += sizeof(u32) * (nr_asid_secs + nr_xlats);
	smmu = devm_kzalloc(dev, bytes, GFP_KERNEL);
	if (!smmu) {
		dev_err(dev, "failed to allocate smmu_device\n");
		return -ENOMEM;
	}

	smmu->clients = RB_ROOT;
	smmu->map = (struct dma_iommu_mapping **)(smmu->as + asids);
	smmu->xlat = (u32 *)(smmu->map + smmu->num_as);
	smmu->asid_sec = smmu->xlat + smmu->nr_xlats;
	smmu->nregs = pdev->num_resources - 1;
	smmu->tlb_reset = (pdata && pdata->tlb_reset) ? pdata->tlb_reset :
		(SMMU_TLB_CONFIG_RESET_VAL | 0x10);
	smmu->ptc_reset = (pdata && pdata->ptc_reset) ? pdata->ptc_reset :
		(SMMU_PTC_CONFIG_RESET_VAL | SMMU_PTC_REQ_LIMIT);
	smmu->extended_pa = pdata->extended_pa;
	smmu->regs = devm_kzalloc(dev, 2 * smmu->nregs * sizeof(*smmu->regs),
				  GFP_KERNEL);
	smmu->rege = smmu->regs + smmu->nregs;
	if (!smmu->regs)
		return -ENOMEM;
	for (i = 0; i < smmu->nregs; i++) {
		struct resource *res;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			return -ENODEV;
		smmu->regs[i] = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(smmu->regs[i]))
			return PTR_ERR(smmu->regs[i]);
		smmu->rege[i] = smmu->regs[i] + resource_size(res) - 1;
	}
	/* Same as "mc" 1st regiter block start address */
	smmu->regbase = (void __iomem *)((u32)smmu->regs[0] & PAGE_MASK);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;
	err = devm_request_irq(&pdev->dev, irq, tegra_smmu_irq, 0,
			       "tegra-smmu", &pdev->dev);
	if (err)
		return err;
	smmu_write(smmu, MC_INT_INVALID_SMMU_PAGE, MC_INTMASK);

	err = of_get_dma_window(dev->of_node, NULL, 0, NULL, &base, &size);
	if (err)
		return -ENODEV;

	if (size & SMMU_PAGE_MASK)
		return -EINVAL;

	size >>= SMMU_PAGE_SHIFT;
	if (!size)
		return -EINVAL;

	smmu->ahb = of_parse_phandle(dev->of_node, "nvidia,ahb", 0);
	smmu->iommu.dev = dev;
	smmu->num_as = asids;
	smmu->nr_xlats = nr_xlats;
	smmu->iovmm_base = base;
	smmu->page_count = size;
	smmu->nr_asid_secs = nr_asid_secs;
	for (i = 0; i < smmu->nr_xlats; i++)
		smmu->xlat[i] = ~0;

	for (i = 0; i < smmu->num_as; i++) {
		struct smmu_as *as = &smmu->as[i];

		as->smmu = smmu;
		as->asid = i;
		as->pdir_attr = _PDIR_ATTR;
		as->pde_attr = _PDE_ATTR;
		as->pte_attr = _PTE_ATTR;

		spin_lock_init(&as->lock);
		spin_lock_init(&as->client_lock);
		INIT_LIST_HEAD(&as->client);
	}
	spin_lock_init(&smmu->lock);
	err = smmu_setup_regs(smmu);
	if (err)
		return err;
	platform_set_drvdata(pdev, smmu);

	smmu->avp_vector_page = alloc_page(GFP_KERNEL);
	if (!smmu->avp_vector_page)
		return -ENOMEM;

	smmu_debugfs_create(smmu);
	smmu_handle = smmu;
	bus_set_iommu(&platform_bus_type, &smmu_iommu_ops);
	tegra_smmu_create_default_map(smmu);
	register_syscore_ops(&tegra_smmu_syscore_ops);

	iommu_add(&smmu->iommu);
	return 0;
}

static int tegra_smmu_remove(struct platform_device *pdev)
{
	struct smmu_device *smmu = platform_get_drvdata(pdev);
	int i;

	smmu_debugfs_delete(smmu);

	unregister_syscore_ops(&tegra_smmu_syscore_ops);
	smmu_write(smmu, SMMU_CONFIG_DISABLE, SMMU_CONFIG);
	for (i = 0; i < smmu->num_as; i++)
		free_pdir(&smmu->as[i]);
	__free_page(smmu->avp_vector_page);
	smmu_handle = NULL;
	iommu_del(&smmu->iommu);
	return 0;
}

static struct platform_driver tegra_smmu_driver = {
	.probe		= tegra_smmu_probe,
	.remove		= tegra_smmu_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra-smmu",
		.of_match_table = tegra_smmu_of_match,
	},
};

static int tegra_smmu_init(void)
{
	return platform_driver_register(&tegra_smmu_driver);
}

static void __exit tegra_smmu_exit(void)
{
	platform_driver_unregister(&tegra_smmu_driver);
}

subsys_initcall(tegra_smmu_init);
module_exit(tegra_smmu_exit);

MODULE_DESCRIPTION("IOMMU API for SMMU in Tegra30");
MODULE_AUTHOR("Hiroshi DOYU <hdoyu@nvidia.com>");
MODULE_ALIAS("platform:tegra-smmu");
MODULE_LICENSE("GPL v2");
