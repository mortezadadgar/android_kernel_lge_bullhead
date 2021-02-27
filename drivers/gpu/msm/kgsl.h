/* Copyright (c) 2008-2015,2017,2021. The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __KGSL_H
#define __KGSL_H

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/msm_kgsl.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/regulator/consumer.h>
#include <linux/mm.h>
#include <linux/dma-attrs.h>
#include <linux/uaccess.h>

/* The number of memstore arrays limits the number of contexts allowed.
 * If more contexts are needed, update multiple for MEMSTORE_SIZE
 */
#define KGSL_MEMSTORE_SIZE	((int)(PAGE_SIZE * 2))
#define KGSL_MEMSTORE_GLOBAL	(0)
#define KGSL_PRIORITY_MAX_RB_LEVELS 4
#define KGSL_MEMSTORE_MAX	(KGSL_MEMSTORE_SIZE / \
	sizeof(struct kgsl_devmemstore) - 1 - KGSL_PRIORITY_MAX_RB_LEVELS)

/* Timestamp window used to detect rollovers (half of integer range) */
#define KGSL_TIMESTAMP_WINDOW 0x80000000

/* The SVM upper bound is the same as the TASK_SIZE in arm32 */
#define KGSL_SVM_UPPER_BOUND (0xC0000000 - SZ_16M)

/*
 * Defines the lowest possible addresses for SVM map. The VA space below
 * has been reserved for GMEM and SP Memory regions.
 */
#define KGSL_SVM_LOWER_BOUND 0x300000

/* A macro for memory statistics - add the new size to the stat and if
   the statisic is greater then _max, set _max
*/

#define KGSL_STATS_ADD(_size, _stat, _max) \
	do { _stat += (_size); if (_stat > _max) _max = _stat; } while (0)

#define KGSL_MAX_NUMIBS 100000

struct kgsl_device;
struct kgsl_context;

struct kgsl_driver {
	struct cdev cdev;
	dev_t major;
	struct class *class;
	/* Virtual device for managing the core */
	struct device virtdev;
	/* Kobjects for storing pagetable and process statistics */
	struct kobject *ptkobj;
	struct kobject *prockobj;
	struct kgsl_device *devp[KGSL_DEVICE_MAX];

	/* Global lilst of open processes */
	struct list_head process_list;
	/* Global list of pagetables */
	struct list_head pagetable_list;
	/* Spinlock for accessing the pagetable list */
	spinlock_t ptlock;
	/* Mutex for accessing the process list */
	struct mutex process_mutex;

	/* Mutex for protecting the device list */
	struct mutex devlock;

	struct {
		uint64_t vmalloc;
		uint64_t vmalloc_max;
		uint64_t page_alloc;
		uint64_t page_alloc_max;
		uint64_t coherent;
		uint64_t coherent_max;
		uint64_t secure;
		uint64_t secure_max;
		uint64_t mapped;
		uint64_t mapped_max;
	} stats;
	unsigned int full_cache_threshold;
};

extern struct kgsl_driver kgsl_driver;

struct kgsl_pagetable;
struct kgsl_memdesc;

struct kgsl_memdesc_ops {
	unsigned int vmflags;
	int (*vmfault)(struct kgsl_memdesc *, struct vm_area_struct *,
		       struct vm_fault *);
	void (*free)(struct kgsl_memdesc *memdesc);
	int (*map_kernel)(struct kgsl_memdesc *);
	void (*unmap_kernel)(struct kgsl_memdesc *);
};

/* Internal definitions for memdesc->priv */
#define KGSL_MEMDESC_GUARD_PAGE BIT(0)
/* Set if the memdesc is mapped into all pagetables */
#define KGSL_MEMDESC_GLOBAL BIT(1)
/* The memdesc is frozen during a snapshot */
#define KGSL_MEMDESC_FROZEN BIT(2)
/* The memdesc is mapped into a pagetable */
#define KGSL_MEMDESC_MAPPED BIT(3)
/* The memdesc is secured for content protection */
#define KGSL_MEMDESC_SECURE BIT(4)
/* The memdesc is private for use during pagetable switch only */
#define KGSL_MEMDESC_PRIVATE BIT(5)
/* Memory is accessible in privileged mode */
#define KGSL_MEMDESC_PRIVILEGED BIT(6)
/* The memdesc is TZ locked content protection */
#define KGSL_MEMDESC_TZ_LOCKED BIT(7)

/**
 * struct kgsl_memdesc - GPU memory object descriptor
 * @pagetable: Pointer to the pagetable that the object is mapped in
 * @hostptr: Kernel virtual address
 * @hostptr_count: Number of threads using hostptr
 * @gpuaddr: GPU virtual address
 * @physaddr: Physical address of the memory object
 * @size: Size of the memory object
 * @mmapsize: Total size of the object in VM (including guard)
 * @priv: Internal flags and settings
 * @sgt: Scatter gather table for allocated pages
 * @ops: Function hooks for the memdesc memory type
 * @flags: Flags set from userspace
 * @dev: Pointer to the struct device that owns this memory
 * @memmap: bitmap of pages for mmapsize
 * @memmap_len: Number of bits for memmap
 */
struct kgsl_memdesc {
	struct kgsl_pagetable *pagetable;
	void *hostptr;
	unsigned int hostptr_count;
	uint64_t gpuaddr;
	phys_addr_t physaddr;
	uint64_t size;
	uint64_t mmapsize;
	unsigned int priv;
	struct sg_table *sgt;
	struct kgsl_memdesc_ops *ops;
	uint64_t flags;
	struct device *dev;
	struct dma_attrs attrs;
};

/*
 * List of different memory entry types. The usermem enum
 * starts at 0, which we use for allocated memory, so 1 is
 * added to the enum values.
 */
#define KGSL_MEM_ENTRY_KERNEL 0
#define KGSL_MEM_ENTRY_USER (KGSL_USER_MEM_TYPE_ADDR + 1)
#define KGSL_MEM_ENTRY_ION (KGSL_USER_MEM_TYPE_ION + 1)
#define KGSL_MEM_ENTRY_MAX (KGSL_USER_MEM_TYPE_MAX + 1)

/* symbolic table for trace and debugfs */
#define KGSL_MEM_TYPES \
	{ KGSL_MEM_ENTRY_KERNEL, "gpumem" }, \
	{ KGSL_MEM_ENTRY_USER, "usermem" }, \
	{ KGSL_MEM_ENTRY_ION, "ion" }

/*
 * struct kgsl_mem_entry - a userspace memory allocation
 * @refcount: reference count. Currently userspace can only
 *  hold a single reference count, but the kernel may hold more.
 * @memdesc: description of the memory
 * @priv_data: type-specific data, such as the dma-buf attachment pointer.
 * @node: rb_node for the gpu address lookup rb tree
 * @id: idr index for this entry, can be used to find memory that does not have
 *  a valid GPU address.
 * @priv: back pointer to the process that owns this memory
 * @pending_free: if !0, userspace requested that his memory be freed, but there
 *  are still references to it.
 * @dev_priv: back pointer to the device file that created this entry.
 */
struct kgsl_mem_entry {
	struct kref refcount;
	struct kgsl_memdesc memdesc;
	void *priv_data;
	struct rb_node node;
	unsigned int id;
	struct kgsl_process_private *priv;
	int pending_free;
	/*
	 * @map_count: Count how many vmas this object is mapped in - used for
	 * debugfs accounting
	 */
	atomic_t map_count;
};

struct kgsl_device_private;
struct kgsl_event_group;

typedef void (*kgsl_event_func)(struct kgsl_device *, struct kgsl_event_group *,
		void *, int);

/**
 * struct kgsl_event - KGSL GPU timestamp event
 * @device: Pointer to the KGSL device that owns the event
 * @context: Pointer to the context that owns the event
 * @timestamp: Timestamp for the event to expire
 * @func: Callback function for for the event when it expires
 * @priv: Private data passed to the callback function
 * @node: List node for the kgsl_event_group list
 * @created: Jiffies when the event was created
 * @work: Work struct for dispatching the callback
 * @result: KGSL event result type to pass to the callback
 * group: The event group this event belongs to
 */
struct kgsl_event {
	struct kgsl_device *device;
	struct kgsl_context *context;
	unsigned int timestamp;
	kgsl_event_func func;
	void *priv;
	struct list_head node;
	unsigned int created;
	struct work_struct work;
	int result;
	struct kgsl_event_group *group;
};

typedef int (*readtimestamp_func)(struct kgsl_device *, void *,
	enum kgsl_timestamp_type, unsigned int *);

/**
 * struct event_group - A list of GPU events
 * @context: Pointer to the active context for the events
 * @lock: Spinlock for protecting the list
 * @events: List of active GPU events
 * @group: Node for the master group list
 * @processed: Last processed timestamp
 * @name: String name for the group (for the debugfs file)
 * @readtimestamp: Function pointer to read a timestamp
 * @priv: Priv member to pass to the readtimestamp function
 */
struct kgsl_event_group {
	struct kgsl_context *context;
	spinlock_t lock;
	struct list_head events;
	struct list_head group;
	unsigned int processed;
	char name[64];
	readtimestamp_func readtimestamp;
	void *priv;
};

long kgsl_ioctl_device_getproperty(struct kgsl_device_private *dev_priv,
					  unsigned int cmd, void *data);
long kgsl_ioctl_device_setproperty(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_device_waittimestamp_ctxtid(struct kgsl_device_private
				*dev_priv, unsigned int cmd, void *data);
long kgsl_ioctl_rb_issueibcmds(struct kgsl_device_private *dev_priv,
				      unsigned int cmd, void *data);
long kgsl_ioctl_submit_commands(struct kgsl_device_private *dev_priv,
				unsigned int cmd, void *data);
long kgsl_ioctl_cmdstream_readtimestamp_ctxtid(struct kgsl_device_private
					*dev_priv, unsigned int cmd,
					void *data);
long kgsl_ioctl_cmdstream_freememontimestamp_ctxtid(
						struct kgsl_device_private
						*dev_priv, unsigned int cmd,
						void *data);
long kgsl_ioctl_drawctxt_create(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_drawctxt_destroy(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_sharedmem_free(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_free_id(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_map_user_mem(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_sync_cache(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_sync_cache_bulk(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_sharedmem_flush_cache(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_alloc(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_alloc_id(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_get_info(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_cff_syncmem(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_cff_user_event(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_timestamp_event(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_cff_sync_gpuobj(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_alloc(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_free(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_info(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_import(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_sync(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpu_command(struct kgsl_device_private *dev_priv,
				unsigned int cmd, void *data);

void kgsl_mem_entry_destroy(struct kref *kref);

struct kgsl_mem_entry *kgsl_sharedmem_find_region(
	struct kgsl_process_private *private, uint64_t gpuaddr,
	uint64_t size);
void kgsl_get_egl_counts(struct kgsl_mem_entry *entry,
			int *egl_surface_count, int *egl_image_count);

struct kgsl_mem_entry * __must_check
kgsl_sharedmem_find_id(struct kgsl_process_private *process, unsigned int id);

void kgsl_get_memory_usage(char *str, size_t len, uint64_t memflags);

extern const struct dev_pm_ops kgsl_pm_ops;

int kgsl_suspend_driver(struct platform_device *pdev, pm_message_t state);
int kgsl_resume_driver(struct platform_device *pdev);

static inline int kgsl_gpuaddr_in_memdesc(const struct kgsl_memdesc *memdesc,
				uint64_t gpuaddr, uint64_t size)
{
	/* set a minimum size to search for */
	if (!size)
		size = 1;

	/* don't overflow */
	if (size > UINT_MAX - gpuaddr)
		return 0;

	if (gpuaddr >= memdesc->gpuaddr &&
	    ((gpuaddr + size) <= (memdesc->gpuaddr + memdesc->size))) {
		return 1;
	}
	return 0;
}

static inline void *kgsl_memdesc_map(struct kgsl_memdesc *memdesc)
{
	if (memdesc->ops && memdesc->ops->map_kernel)
		memdesc->ops->map_kernel(memdesc);

	return memdesc->hostptr;
}

static inline void kgsl_memdesc_unmap(struct kgsl_memdesc *memdesc)
{
	if (memdesc->ops && memdesc->ops->unmap_kernel)
		memdesc->ops->unmap_kernel(memdesc);
}

static inline void *kgsl_gpuaddr_to_vaddr(struct kgsl_memdesc *memdesc,
					     uint64_t gpuaddr)
{
	void *hostptr = NULL;

	if ((gpuaddr >= memdesc->gpuaddr) &&
		(gpuaddr < (memdesc->gpuaddr + memdesc->size)))
		hostptr = kgsl_memdesc_map(memdesc);

	return hostptr != NULL ? hostptr + (gpuaddr - memdesc->gpuaddr) : NULL;
}

static inline int timestamp_cmp(unsigned int a, unsigned int b)
{
	/* check for equal */
	if (a == b)
		return 0;

	/* check for greater-than for non-rollover case */
	if ((a > b) && (a - b < KGSL_TIMESTAMP_WINDOW))
		return 1;

	/* check for greater-than for rollover case
	 * note that <= is required to ensure that consistent
	 * results are returned for values whose difference is
	 * equal to the window size
	 */
	a += KGSL_TIMESTAMP_WINDOW;
	b += KGSL_TIMESTAMP_WINDOW;
	return ((a > b) && (a - b <= KGSL_TIMESTAMP_WINDOW)) ? 1 : -1;
}

static inline int
kgsl_mem_entry_get(struct kgsl_mem_entry *entry)
{
	if (entry)
		return kref_get_unless_zero(&entry->refcount);
	return 0;
}

static inline void
kgsl_mem_entry_put(struct kgsl_mem_entry *entry)
{
	if (entry)
		kref_put(&entry->refcount, kgsl_mem_entry_destroy);
}

/*
 * kgsl_addr_range_overlap() - Checks if 2 ranges overlap
 * @gpuaddr1: Start of first address range
 * @size1: Size of first address range
 * @gpuaddr2: Start of second address range
 * @size2: Size of second address range
 *
 * Function returns true if the 2 given address ranges overlap
 * else false
 */
static inline bool kgsl_addr_range_overlap(uint64_t gpuaddr1,
		uint64_t size1, uint64_t gpuaddr2, uint64_t size2)
{
	if ((size1 > (UINT_MAX - gpuaddr1)) || (size2 > (UINT_MAX - gpuaddr2)))
		return false;
	return !(((gpuaddr1 + size1) <= gpuaddr2) ||
		(gpuaddr1 >= (gpuaddr2 + size2)));
}

/**
 * kgsl_malloc() - Use either kzalloc or vmalloc to allocate memory
 * @size: Size of the desired allocation
 *
 * Allocate a block of memory for the driver - if it is small try to allocate it
 * from kmalloc (fast!) otherwise we need to go with vmalloc (safe!)
 */
static inline void *kgsl_malloc(size_t size)
{
	if (size <= PAGE_SIZE)
		return kzalloc(size, GFP_KERNEL);

	return vmalloc(size);
}

/**
 * kgsl_free() - Free memory allocated by kgsl_malloc()
 * @ptr: Pointer to the memory to free
 *
 * Free the memory be it in vmalloc or kmalloc space
 */
static inline void kgsl_free(void *ptr)
{
	if (ptr != NULL && is_vmalloc_addr(ptr))
		return vfree(ptr);

	kfree(ptr);
}

static inline int _copy_from_user(void *dest, void __user *src,
		unsigned int ksize, unsigned int usize)
{
	unsigned int copy = ksize < usize ? ksize : usize;

	if (copy == 0)
		return -EINVAL;

	return copy_from_user(dest, src, copy) ? -EFAULT : 0;
}

static inline void __user *to_user_ptr(uint64_t address)
{
	return (void __user *)(uintptr_t)address;
}

#endif /* __KGSL_H */
