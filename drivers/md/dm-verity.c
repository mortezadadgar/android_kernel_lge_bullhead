/*
 * Copyright (C) 2012 Red Hat, Inc.
 *
 * Author: Mikulas Patocka <mpatocka@redhat.com>
 *
 * Based on Chromium dm-verity driver (C) 2011 The Chromium OS Authors
 *
 * This file is released under the GPLv2.
 *
 * In the file "/sys/module/dm_verity/parameters/prefetch_cluster" you can set
 * default prefetch value. Data are read in "prefetch_cluster" chunks from the
 * hash device. Setting this greatly improves performance when data and hash
 * are on the same disk on different partitions on devices with poor random
 * access behavior.
 */

#include "dm-bufio.h"

#include <linux/module.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/device-mapper.h>
#include <linux/mount.h>
#include <crypto/hash.h>
#include "dm-verity.h"

#define DM_MSG_PREFIX			"verity"

#define DM_VERITY_IO_VEC_INLINE		16
#define DM_VERITY_MEMPOOL_SIZE		4
#define DM_VERITY_DEFAULT_PREFETCH_SIZE	262144

#define DM_VERITY_MAX_LEVELS		63
#define DM_VERITY_NUM_POSITIONAL_ARGS	10

static unsigned dm_verity_prefetch_cluster = DM_VERITY_DEFAULT_PREFETCH_SIZE;

module_param_named(prefetch_cluster, dm_verity_prefetch_cluster, uint, S_IRUGO | S_IWUSR);

struct dm_verity {
	struct dm_dev *data_dev;
	struct dm_dev *hash_dev;
	struct dm_target *ti;
	struct dm_bufio_client *bufio;
	char *alg_name;
	struct crypto_shash *tfm;
	u8 *root_digest;	/* digest of the root block */
	u8 *salt;		/* salt: its size is salt_size */
	unsigned salt_size;
	sector_t data_start;	/* data offset in 512-byte sectors */
	sector_t hash_start;	/* hash start in blocks */
	sector_t data_blocks;	/* the number of data blocks */
	sector_t hash_blocks;	/* the number of hash blocks */
	unsigned char data_dev_block_bits;	/* log2(data blocksize) */
	unsigned char hash_dev_block_bits;	/* log2(hash blocksize) */
	unsigned char hash_per_block_bits;	/* log2(hashes in hash block) */
	unsigned char levels;	/* the number of tree levels */
	unsigned char version;
	unsigned digest_size;	/* digest size for the current hash algorithm */
	unsigned shash_descsize;/* the size of temporary space for crypto */
	int hash_failed;	/* set to 1 if hash of any block failed */
	int error_behavior;	/* selects error behavior on io erros */

	mempool_t *vec_mempool;	/* mempool of bio vector */

	struct workqueue_struct *verify_wq;

	/* starting blocks for each tree level. 0 is the lowest level. */
	sector_t hash_level_block[DM_VERITY_MAX_LEVELS];
};

struct dm_verity_io {
	struct dm_verity *v;

	/* original values of bio->bi_end_io and bio->bi_private */
	bio_end_io_t *orig_bi_end_io;
	void *orig_bi_private;

	sector_t block;
	unsigned n_blocks;

	/* saved bio vector */
	struct bio_vec *io_vec;
	unsigned io_vec_size;

	struct work_struct work;

	/* A space for short vectors; longer vectors are allocated separately. */
	struct bio_vec io_vec_inline[DM_VERITY_IO_VEC_INLINE];

	/*
	 * Three variably-size fields follow this struct:
	 *
	 * u8 hash_desc[v->shash_descsize];
	 * u8 real_digest[v->digest_size];
	 * u8 want_digest[v->digest_size];
	 *
	 * To access them use: io_hash_desc(), io_real_digest() and io_want_digest().
	 */
};

struct dm_verity_prefetch_work {
	struct work_struct work;
	struct dm_verity *v;
	sector_t block;
	unsigned n_blocks;
};

/* Provide a lightweight means of specifying the global default for
 * error behavior: eio, reboot, or none
 * Legacy support for 0 = eio, 1 = reboot/panic, 2 = none, 3 = notify.
 * This is matched to the enum in dm-verity.h.
 */
static const char *allowed_error_behaviors[] = { "eio", "panic", "none",
						 "notify", NULL };
static char *error_behavior = "eio";
module_param(error_behavior, charp, 0644);
MODULE_PARM_DESC(error_behavior, "Behavior on error "
				 "(eio, panic, none, notify)");

/* Controls whether verity_get_device will wait forever for a device. */
static int dev_wait;
module_param(dev_wait, int, 0444);
MODULE_PARM_DESC(dev_wait, "Wait forever for a backing device");

static BLOCKING_NOTIFIER_HEAD(verity_error_notifier);

int dm_verity_register_error_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&verity_error_notifier, nb);
}
EXPORT_SYMBOL_GPL(dm_verity_register_error_notifier);

int dm_verity_unregister_error_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&verity_error_notifier, nb);
}
EXPORT_SYMBOL_GPL(dm_verity_unregister_error_notifier);

/* If the request is not successful, this handler takes action.
 * TODO make this call a registered handler.
 */
static void verity_error(struct dm_verity *v, struct dm_verity_io *io,
			 int error)
{
	const char *message = v->hash_failed ? "integrity" : "block";
	int error_behavior = DM_VERITY_ERROR_BEHAVIOR_PANIC;
	dev_t devt = 0;
	u64 block = ~0;
	struct dm_verity_error_state error_state;
	/* If the hash did not fail, then this is likely transient. */
	int transient = !v->hash_failed;

	devt = v->data_dev->bdev->bd_dev;
	error_behavior = v->error_behavior;
	if (io)
		block = io->block;

	DMERR_LIMIT("verification failure occurred: %s failure%s", message,
		    transient ? " (transient)" : "");

	if (error_behavior == DM_VERITY_ERROR_BEHAVIOR_NOTIFY) {
		error_state.code = error;
		error_state.transient = transient;
		error_state.block = block;
		error_state.message = message;
		error_state.dev_start = v->data_start;
		error_state.dev_len = v->data_blocks;
		error_state.dev = v->data_dev->bdev;
		error_state.hash_dev_start = v->hash_start;
		error_state.hash_dev_len = v->hash_blocks;
		error_state.hash_dev = v->hash_dev->bdev;

		/* Set default fallthrough behavior. */
		error_state.behavior = DM_VERITY_ERROR_BEHAVIOR_PANIC;
		error_behavior = DM_VERITY_ERROR_BEHAVIOR_PANIC;

		if (!blocking_notifier_call_chain(
		    &verity_error_notifier, transient, &error_state)) {
			error_behavior = error_state.behavior;
		}
	}

	switch (error_behavior) {
	case DM_VERITY_ERROR_BEHAVIOR_EIO:
		break;
	case DM_VERITY_ERROR_BEHAVIOR_NONE:
		break;
	default:
		goto do_panic;
	}
	return;

do_panic:
	panic("dm-verity failure: "
	      "device:%u:%u error:%d block:%llu message:%s",
	      MAJOR(devt), MINOR(devt), error, (u64)block, message);
}

/**
 * verity_parse_error_behavior - parse a behavior charp to the enum
 * @behavior:	NUL-terminated char array
 *
 * Checks if the behavior is valid either as text or as an index digit
 * and returns the proper enum value or -1 on error.
 */
static int verity_parse_error_behavior(const char *behavior)
{
	const char **allowed = allowed_error_behaviors;
	char index = '0';

	for (; *allowed; allowed++, index++)
		if (!strcmp(*allowed, behavior) || behavior[0] == index)
			break;

	if (!*allowed)
		return -1;

	/* Convert to the integer index matching the enum. */
	return allowed - allowed_error_behaviors;
}

static struct shash_desc *io_hash_desc(struct dm_verity *v, struct dm_verity_io *io)
{
	return (struct shash_desc *)(io + 1);
}

static u8 *io_real_digest(struct dm_verity *v, struct dm_verity_io *io)
{
	return (u8 *)(io + 1) + v->shash_descsize;
}

static u8 *io_want_digest(struct dm_verity *v, struct dm_verity_io *io)
{
	return (u8 *)(io + 1) + v->shash_descsize + v->digest_size;
}

/*
 * Auxiliary structure appended to each dm-bufio buffer. If the value
 * hash_verified is nonzero, hash of the block has been verified.
 *
 * The variable hash_verified is set to 0 when allocating the buffer, then
 * it can be changed to 1 and it is never reset to 0 again.
 *
 * There is no lock around this value, a race condition can at worst cause
 * that multiple processes verify the hash of the same buffer simultaneously
 * and write 1 to hash_verified simultaneously.
 * This condition is harmless, so we don't need locking.
 */
struct buffer_aux {
	int hash_verified;
};

/*
 * Initialize struct buffer_aux for a freshly created buffer.
 */
static void dm_bufio_alloc_callback(struct dm_buffer *buf)
{
	struct buffer_aux *aux = dm_bufio_get_aux_data(buf);

	aux->hash_verified = 0;
}

/*
 * Translate input sector number to the sector number on the target device.
 */
static sector_t verity_map_sector(struct dm_verity *v, sector_t bi_sector)
{
	return v->data_start + dm_target_offset(v->ti, bi_sector);
}

/*
 * Return hash position of a specified block at a specified tree level
 * (0 is the lowest level).
 * The lowest "hash_per_block_bits"-bits of the result denote hash position
 * inside a hash block. The remaining bits denote location of the hash block.
 */
static sector_t verity_position_at_level(struct dm_verity *v, sector_t block,
					 int level)
{
	return block >> (level * v->hash_per_block_bits);
}

static void verity_hash_at_level(struct dm_verity *v, sector_t block, int level,
				 sector_t *hash_block, unsigned *offset)
{
	sector_t position = verity_position_at_level(v, block, level);
	unsigned idx;

	*hash_block = v->hash_level_block[level] + (position >> v->hash_per_block_bits);

	if (!offset)
		return;

	idx = position & ((1 << v->hash_per_block_bits) - 1);
	if (!v->version)
		*offset = idx * v->digest_size;
	else
		*offset = idx << (v->hash_dev_block_bits - v->hash_per_block_bits);
}

/*
 * Verify hash of a metadata block pertaining to the specified data block
 * ("block" argument) at a specified level ("level" argument).
 *
 * On successful return, io_want_digest(v, io) contains the hash value for
 * a lower tree level or for the data block (if we're at the lowest leve).
 *
 * If "skip_unverified" is true, unverified buffer is skipped and 1 is returned.
 * If "skip_unverified" is false, unverified buffer is hashed and verified
 * against current value of io_want_digest(v, io).
 */
static int verity_verify_level(struct dm_verity_io *io, sector_t block,
			       int level, bool skip_unverified)
{
	struct dm_verity *v = io->v;
	struct dm_buffer *buf;
	struct buffer_aux *aux;
	u8 *data;
	int r;
	sector_t hash_block;
	unsigned offset;

	verity_hash_at_level(v, block, level, &hash_block, &offset);

	data = dm_bufio_read(v->bufio, hash_block, &buf);
	if (unlikely(IS_ERR(data)))
		return PTR_ERR(data);

	aux = dm_bufio_get_aux_data(buf);

	if (!aux->hash_verified) {
		struct shash_desc *desc;
		u8 *result;

		if (skip_unverified) {
			r = 1;
			goto release_ret_r;
		}

		desc = io_hash_desc(v, io);
		desc->tfm = v->tfm;
		desc->flags = CRYPTO_TFM_REQ_MAY_SLEEP;
		r = crypto_shash_init(desc);
		if (r < 0) {
			DMERR("crypto_shash_init failed: %d", r);
			goto release_ret_r;
		}

		if (likely(v->version >= 1)) {
			r = crypto_shash_update(desc, v->salt, v->salt_size);
			if (r < 0) {
				DMERR("crypto_shash_update failed: %d", r);
				goto release_ret_r;
			}
		}

		r = crypto_shash_update(desc, data, 1 << v->hash_dev_block_bits);
		if (r < 0) {
			DMERR("crypto_shash_update failed: %d", r);
			goto release_ret_r;
		}

		if (!v->version) {
			r = crypto_shash_update(desc, v->salt, v->salt_size);
			if (r < 0) {
				DMERR("crypto_shash_update failed: %d", r);
				goto release_ret_r;
			}
		}

		result = io_real_digest(v, io);
		r = crypto_shash_final(desc, result);
		if (r < 0) {
			DMERR("crypto_shash_final failed: %d", r);
			goto release_ret_r;
		}
		if (unlikely(memcmp(result, io_want_digest(v, io), v->digest_size))) {
			DMERR_LIMIT("metadata block %llu is corrupted",
				(unsigned long long)hash_block);
			v->hash_failed = 1;
			r = -EIO;
			goto release_ret_r;
		} else
			aux->hash_verified = 1;
	}

	data += offset;

	memcpy(io_want_digest(v, io), data, v->digest_size);

	dm_bufio_release(buf);
	return 0;

release_ret_r:
	dm_bufio_release(buf);

	return r;
}

/*
 * Verify one "dm_verity_io" structure.
 */
static int verity_verify_io(struct dm_verity_io *io)
{
	struct dm_verity *v = io->v;
	unsigned b;
	int i;
	unsigned vector = 0, offset = 0;

	for (b = 0; b < io->n_blocks; b++) {
		struct shash_desc *desc;
		u8 *result;
		int r;
		unsigned todo;

		if (likely(v->levels)) {
			/*
			 * First, we try to get the requested hash for
			 * the current block. If the hash block itself is
			 * verified, zero is returned. If it isn't, this
			 * function returns 0 and we fall back to whole
			 * chain verification.
			 */
			int r = verity_verify_level(io, io->block + b, 0, true);
			if (likely(!r))
				goto test_block_hash;
			if (r < 0)
				return r;
		}

		memcpy(io_want_digest(v, io), v->root_digest, v->digest_size);

		for (i = v->levels - 1; i >= 0; i--) {
			int r = verity_verify_level(io, io->block + b, i, false);
			if (unlikely(r))
				return r;
		}

test_block_hash:
		desc = io_hash_desc(v, io);
		desc->tfm = v->tfm;
		desc->flags = CRYPTO_TFM_REQ_MAY_SLEEP;
		r = crypto_shash_init(desc);
		if (r < 0) {
			DMERR("crypto_shash_init failed: %d", r);
			return r;
		}

		if (likely(v->version >= 1)) {
			r = crypto_shash_update(desc, v->salt, v->salt_size);
			if (r < 0) {
				DMERR("crypto_shash_update failed: %d", r);
				return r;
			}
		}

		todo = 1 << v->data_dev_block_bits;
		do {
			struct bio_vec *bv;
			u8 *page;
			unsigned len;

			BUG_ON(vector >= io->io_vec_size);
			bv = &io->io_vec[vector];
			page = kmap_atomic(bv->bv_page);
			len = bv->bv_len - offset;
			if (likely(len >= todo))
				len = todo;
			r = crypto_shash_update(desc,
					page + bv->bv_offset + offset, len);
			kunmap_atomic(page);
			if (r < 0) {
				DMERR("crypto_shash_update failed: %d", r);
				return r;
			}
			offset += len;
			if (likely(offset == bv->bv_len)) {
				offset = 0;
				vector++;
			}
			todo -= len;
		} while (todo);

		if (!v->version) {
			r = crypto_shash_update(desc, v->salt, v->salt_size);
			if (r < 0) {
				DMERR("crypto_shash_update failed: %d", r);
				return r;
			}
		}

		result = io_real_digest(v, io);
		r = crypto_shash_final(desc, result);
		if (r < 0) {
			DMERR("crypto_shash_final failed: %d", r);
			return r;
		}
		if (unlikely(memcmp(result, io_want_digest(v, io), v->digest_size))) {
			DMERR_LIMIT("data block %llu is corrupted",
				(unsigned long long)(io->block + b));
			v->hash_failed = 1;
			return -EIO;
		}
	}
	BUG_ON(vector != io->io_vec_size);
	BUG_ON(offset);

	return 0;
}

/*
 * End one "io" structure with a given error.
 */
static void verity_finish_io(struct dm_verity_io *io, int error)
{
	struct dm_verity *v = io->v;
	struct bio *bio = dm_bio_from_per_bio_data(io, v->ti->per_bio_data_size);

	if (error)
		verity_error(v, io, error);
	bio->bi_end_io = io->orig_bi_end_io;
	bio->bi_private = io->orig_bi_private;

	if (io->io_vec != io->io_vec_inline)
		mempool_free(io->io_vec, v->vec_mempool);

	bio_endio(bio, error);
}

static void verity_work(struct work_struct *w)
{
	struct dm_verity_io *io = container_of(w, struct dm_verity_io, work);

	verity_finish_io(io, verity_verify_io(io));
}

static void verity_end_io(struct bio *bio, int error)
{
	struct dm_verity_io *io = bio->bi_private;

	if (error) {
		verity_finish_io(io, error);
		return;
	}
	INIT_WORK(&io->work, verity_work);
	queue_work(io->v->verify_wq, &io->work);
}

/*
 * Prefetch buffers for the specified io.
 * The root buffer is not prefetched, it is assumed that it will be cached
 * all the time.
 */
static void verity_prefetch_io(struct work_struct *work)
{
	struct dm_verity_prefetch_work *pw =
		container_of(work, struct dm_verity_prefetch_work, work);
	struct dm_verity *v = pw->v;
	int i;

	for (i = v->levels - 2; i >= 0; i--) {
		sector_t hash_block_start;
		sector_t hash_block_end;
		verity_hash_at_level(v, pw->block, i, &hash_block_start, NULL);
		verity_hash_at_level(v, pw->block + pw->n_blocks - 1, i, &hash_block_end, NULL);
		if (!i) {
			unsigned cluster = ACCESS_ONCE(dm_verity_prefetch_cluster);

			cluster >>= v->data_dev_block_bits;
			if (unlikely(!cluster))
				goto no_prefetch_cluster;

			if (unlikely(cluster & (cluster - 1)))
				cluster = 1 << (fls(cluster) - 1);

			hash_block_start &= ~(sector_t)(cluster - 1);
			hash_block_end |= cluster - 1;
			if (unlikely(hash_block_end >= v->hash_blocks))
				hash_block_end = v->hash_blocks - 1;
		}
no_prefetch_cluster:
		dm_bufio_prefetch(v->bufio, hash_block_start,
				  hash_block_end - hash_block_start + 1);
	}

	kfree(pw);
}

static void verity_submit_prefetch(struct dm_verity *v, struct dm_verity_io *io)
{
	struct dm_verity_prefetch_work *pw;

	pw = kmalloc(sizeof(struct dm_verity_prefetch_work),
		GFP_NOIO | __GFP_NORETRY | __GFP_NOMEMALLOC | __GFP_NOWARN);

	if (!pw)
		return;

	INIT_WORK(&pw->work, verity_prefetch_io);
	pw->v = v;
	pw->block = io->block;
	pw->n_blocks = io->n_blocks;
	queue_work(v->verify_wq, &pw->work);
}

/*
 * Bio map function. It allocates dm_verity_io structure and bio vector and
 * fills them. Then it issues prefetches and the I/O.
 */
static int verity_map(struct dm_target *ti, struct bio *bio)
{
	struct dm_verity *v = ti->private;
	struct dm_verity_io *io;

	bio->bi_bdev = v->data_dev->bdev;
	bio->bi_sector = verity_map_sector(v, bio->bi_sector);

	if (((unsigned)bio->bi_sector | bio_sectors(bio)) &
	    ((1 << (v->data_dev_block_bits - SECTOR_SHIFT)) - 1)) {
		DMERR_LIMIT("unaligned io");
		return -EIO;
	}

	if (bio_end_sector(bio) >>
	    (v->data_dev_block_bits - SECTOR_SHIFT) > v->data_blocks) {
		DMERR_LIMIT("io out of range");
		return -EIO;
	}

	if (bio_data_dir(bio) == WRITE) {
		return -EIO;
	}

	io = dm_per_bio_data(bio, ti->per_bio_data_size);
	io->v = v;
	io->orig_bi_end_io = bio->bi_end_io;
	io->orig_bi_private = bio->bi_private;
	io->block = bio->bi_sector >> (v->data_dev_block_bits - SECTOR_SHIFT);
	io->n_blocks = bio->bi_size >> v->data_dev_block_bits;

	bio->bi_end_io = verity_end_io;
	bio->bi_private = io;
	io->io_vec_size = bio_segments(bio);
	if (io->io_vec_size < DM_VERITY_IO_VEC_INLINE)
		io->io_vec = io->io_vec_inline;
	else
		io->io_vec = mempool_alloc(v->vec_mempool, GFP_NOIO);
	memcpy(io->io_vec, bio_iovec(bio),
	       io->io_vec_size * sizeof(struct bio_vec));

	verity_submit_prefetch(v, io);

	generic_make_request(bio);

	return DM_MAPIO_SUBMITTED;
}

/*
 * Status: V (valid) or C (corruption found)
 */
static void verity_status(struct dm_target *ti, status_type_t type,
			  unsigned status_flags, char *result, unsigned maxlen)
{
	struct dm_verity *v = ti->private;
	unsigned sz = 0;
	unsigned x;

	switch (type) {
	case STATUSTYPE_INFO:
		DMEMIT("%c", v->hash_failed ? 'C' : 'V');
		break;
	case STATUSTYPE_TABLE:
		DMEMIT("%u %s %s %u %u %llu %llu %s ",
			v->version,
			v->data_dev->name,
			v->hash_dev->name,
			1 << v->data_dev_block_bits,
			1 << v->hash_dev_block_bits,
			(unsigned long long)v->data_blocks,
			(unsigned long long)v->hash_start,
			v->alg_name
			);
		for (x = 0; x < v->digest_size; x++)
			DMEMIT("%02x", v->root_digest[x]);
		DMEMIT(" ");
		if (!v->salt_size)
			DMEMIT("-");
		else
			for (x = 0; x < v->salt_size; x++)
				DMEMIT("%02x", v->salt[x]);
		break;
	}
}

static int verity_ioctl(struct dm_target *ti, unsigned cmd,
			unsigned long arg)
{
	struct dm_verity *v = ti->private;
	int r = 0;

	if (v->data_start ||
	    ti->len != i_size_read(v->data_dev->bdev->bd_inode) >> SECTOR_SHIFT)
		r = scsi_verify_blk_ioctl(NULL, cmd);

	return r ? : __blkdev_driver_ioctl(v->data_dev->bdev, v->data_dev->mode,
				     cmd, arg);
}

static int verity_merge(struct dm_target *ti, struct bvec_merge_data *bvm,
			struct bio_vec *biovec, int max_size)
{
	struct dm_verity *v = ti->private;
	struct request_queue *q = bdev_get_queue(v->data_dev->bdev);

	if (!q->merge_bvec_fn)
		return max_size;

	bvm->bi_bdev = v->data_dev->bdev;
	bvm->bi_sector = verity_map_sector(v, bvm->bi_sector);

	return min(max_size, q->merge_bvec_fn(q, bvm, biovec));
}

static int verity_iterate_devices(struct dm_target *ti,
				  iterate_devices_callout_fn fn, void *data)
{
	struct dm_verity *v = ti->private;

	return fn(ti, v->data_dev, v->data_start, ti->len, data);
}

static void verity_io_hints(struct dm_target *ti, struct queue_limits *limits)
{
	struct dm_verity *v = ti->private;

	if (limits->logical_block_size < 1 << v->data_dev_block_bits)
		limits->logical_block_size = 1 << v->data_dev_block_bits;

	if (limits->physical_block_size < 1 << v->data_dev_block_bits)
		limits->physical_block_size = 1 << v->data_dev_block_bits;

	blk_limits_io_min(limits, limits->logical_block_size);
}

static void verity_dtr(struct dm_target *ti)
{
	struct dm_verity *v = ti->private;

	if (v->verify_wq)
		destroy_workqueue(v->verify_wq);

	if (v->vec_mempool)
		mempool_destroy(v->vec_mempool);

	if (v->bufio)
		dm_bufio_client_destroy(v->bufio);

	kfree(v->salt);
	kfree(v->root_digest);

	if (v->tfm)
		crypto_free_shash(v->tfm);

	kfree(v->alg_name);

	if (v->hash_dev)
		dm_put_device(ti, v->hash_dev);

	if (v->data_dev)
		dm_put_device(ti, v->data_dev);

	kfree(v);
}

static int verity_get_device(struct dm_target *ti, char *devname,
			     struct dm_dev **dm_dev)
{
	do {
		/* Try the normal path first since if everything is ready, it
		 * will be the fastest.
		 */
		if (!dm_get_device(ti, devname, /*FMODE_READ*/
				   dm_table_get_mode(ti->table), dm_dev))
			return 0;

		/* No need to be too aggressive since this is a slow path. */
		msleep(50);
	} while (dev_wait && (driver_probe_done() != 0 || *dm_dev == NULL));
	async_synchronize_full();
	return -1;
}

struct verity_args {
	int version;
	char *data_device;
	char *hash_device;
	int data_block_size_bits;
	int hash_block_size_bits;
	u64 num_data_blocks;
	u64 hash_start_block;
	char *algorithm;
	char *digest;
	char *salt;
	char *error_behavior;
};

static void pr_args(struct verity_args *args)
{
	printk(KERN_INFO "VERITY args: version=%d data_device=%s hash_device=%s"
		" data_block_size_bits=%d hash_block_size_bits=%d"
		" num_data_blocks=%lld hash_start_block=%lld"
		" algorithm=%s digest=%s salt=%s error_behavior=%s\n",
		args->version,
		args->data_device,
		args->hash_device,
		args->data_block_size_bits,
		args->hash_block_size_bits,
		args->num_data_blocks,
		args->hash_start_block,
		args->algorithm,
		args->digest,
		args->salt,
		args->error_behavior);
}

/*
 * positional_args - collects the argments using the positional
 * parameters.
 * arg# - parameter
 *    0 - version
 *    1 - data device
 *    2 - hash device - may be same as data device
 *    3 - data block size log2
 *    4 - hash block size log2
 *    5 - number of data blocks
 *    6 - hash start block
 *    7 - algorithm
 *    8 - digest
 *    9 - salt
 */
static char *positional_args(unsigned argc, char **argv,
				struct verity_args *args)
{
	unsigned num;
	unsigned long long num_ll;
	char dummy;

	if (argc != DM_VERITY_NUM_POSITIONAL_ARGS)
		return "Invalid argument count: exactly 10 arguments required";

	if (sscanf(argv[0], "%d%c", &num, &dummy) != 1 ||
	    num < 0 || num > 1)
		return "Invalid version";
	args->version = num;

	args->data_device = argv[1];
	args->hash_device = argv[2];


	if (sscanf(argv[3], "%u%c", &num, &dummy) != 1 ||
	    !num || (num & (num - 1)) ||
	    num > PAGE_SIZE)
		return "Invalid data device block size";
	args->data_block_size_bits = ffs(num) - 1;

	if (sscanf(argv[4], "%u%c", &num, &dummy) != 1 ||
	    !num || (num & (num - 1)) ||
	    num > INT_MAX)
		return "Invalid hash device block size";
	args->hash_block_size_bits = ffs(num) - 1;

	if (sscanf(argv[5], "%llu%c", &num_ll, &dummy) != 1 ||
	    (sector_t)(num_ll << (args->data_block_size_bits - SECTOR_SHIFT))
	    >> (args->data_block_size_bits - SECTOR_SHIFT) != num_ll)
		return "Invalid data blocks";
	args->num_data_blocks = num_ll;


	if (sscanf(argv[6], "%llu%c", &num_ll, &dummy) != 1 ||
	    (sector_t)(num_ll << (args->hash_block_size_bits - SECTOR_SHIFT))
	    >> (args->hash_block_size_bits - SECTOR_SHIFT) != num_ll)
		return "Invalid hash start";
	args->hash_start_block = num_ll;


	args->algorithm = argv[7];
	args->digest = argv[8];
	args->salt = argv[9];

	return NULL;
}

static void splitarg(char *arg, char **key, char **val)
{
	*key = strsep(&arg, "=");
	*val = strsep(&arg, "");
}

static char *chromeos_args(unsigned argc, char **argv, struct verity_args *args)
{
	char *key, *val;
	unsigned long num;
	int i;

	args->version = 0;
	args->data_block_size_bits = 12;
	args->hash_block_size_bits = 12;
	for (i = 0; i < argc; ++i) {
		DMWARN("Argument %d: '%s'", i, argv[i]);
		splitarg(argv[i], &key, &val);
		if (!key) {
			DMWARN("Bad argument %d: missing key?", i);
			return "Bad argument: missing key";
		}
		if (!val) {
			DMWARN("Bad argument %d='%s': missing value", i, key);
			return "Bad argument: missing value";
		}
		if (!strcmp(key, "alg")) {
			args->algorithm = val;
		} else if (!strcmp(key, "payload")) {
			args->data_device = val;
		} else if (!strcmp(key, "hashtree")) {
			args->hash_device = val;
		} else if (!strcmp(key, "root_hexdigest")) {
			args->digest = val;
		} else if (!strcmp(key, "hashstart")) {
			if (strict_strtoul(val, 10, &num))
				return "Invalid hashstart";
			args->hash_start_block =
				num >> (args->hash_block_size_bits - SECTOR_SHIFT);
			args->num_data_blocks = args->hash_start_block;
		} else if (!strcmp(key, "error_behavior")) {
			args->error_behavior = val;
		} else if (!strcmp(key, "salt")) {
			args->salt = val;
		}
	}
	if (!args->salt)
		args->salt = "";

#define NEEDARG(n) \
	if (!(n)) { \
		return "Missing argument: " #n; \
	}

	NEEDARG(args->algorithm);
	NEEDARG(args->data_device);
	NEEDARG(args->hash_device);
	NEEDARG(args->digest);

#undef NEEDARG
	return NULL;
}

/*
 * Target parameters:
 *	<version>	The current format is version 1.
 *			Vsn 0 is compatible with original Chromium OS releases.
 *	<data device>
 *	<hash device>
 *	<data block size>
 *	<hash block size>
 *	<the number of data blocks>
 *	<hash start block>
 *	<algorithm>
 *	<digest>
 *	<salt>		Hex string or "-" if no salt.
 */
static int verity_ctr(struct dm_target *ti, unsigned argc, char **argv)
{
	struct verity_args args = { 0 };
	struct dm_verity *v;
	int r;
	int i;
	sector_t hash_position;

	args.error_behavior = error_behavior;
	if (argc == DM_VERITY_NUM_POSITIONAL_ARGS)
		ti->error = positional_args(argc, argv, &args);
	else
		ti->error = chromeos_args(argc, argv, &args);
	if (ti->error)
		return -EINVAL;
	if (0)
		pr_args(&args);


	v = kzalloc(sizeof(struct dm_verity), GFP_KERNEL);
	if (!v) {
		ti->error = "Cannot allocate verity structure";
		return -ENOMEM;
	}
	ti->private = v;
	v->ti = ti;

	v->version = args.version;

	r = verity_get_device(ti, args.data_device, &v->data_dev);
	if (r) {
		ti->error = "Data device lookup failed";
		goto bad;
	}

	r = verity_get_device(ti, args.hash_device, &v->hash_dev);
	if (r) {
		ti->error = "Data device lookup failed";
		goto bad;
	}

	v->data_dev_block_bits = args.data_block_size_bits;
	if ((1 << v->data_dev_block_bits) <
	    bdev_logical_block_size(v->data_dev->bdev)) {
		ti->error = "Invalid data device block size";
		r = -EINVAL;
		goto bad;
	}

	v->hash_dev_block_bits = args.hash_block_size_bits;
	if ((1 << v->data_dev_block_bits) <
	    bdev_logical_block_size(v->hash_dev->bdev)) {
		ti->error = "Invalid hash device block size";
		r = -EINVAL;
		goto bad;
	}

	v->data_blocks = args.num_data_blocks;
	if (ti->len > (v->data_blocks << (v->data_dev_block_bits - SECTOR_SHIFT))) {
		ti->error = "Data device is too small";
		r = -EINVAL;
		goto bad;
	}

	v->hash_start = args.hash_start_block;

	v->alg_name = kstrdup(args.algorithm, GFP_KERNEL);
	if (!v->alg_name) {
		ti->error = "Cannot allocate algorithm name";
		r = -ENOMEM;
		goto bad;
	}

	v->tfm = crypto_alloc_shash(v->alg_name, 0, 0);
	if (IS_ERR(v->tfm)) {
		ti->error = "Cannot initialize hash function";
		r = PTR_ERR(v->tfm);
		v->tfm = NULL;
		goto bad;
	}
	v->digest_size = crypto_shash_digestsize(v->tfm);
	if ((1 << v->hash_dev_block_bits) < v->digest_size * 2) {
		ti->error = "Digest size too big";
		r = -EINVAL;
		goto bad;
	}
	v->shash_descsize =
		sizeof(struct shash_desc) + crypto_shash_descsize(v->tfm);

	v->root_digest = kmalloc(v->digest_size, GFP_KERNEL);
	if (!v->root_digest) {
		ti->error = "Cannot allocate root digest";
		r = -ENOMEM;
		goto bad;
	}
	if (strlen(args.digest) != v->digest_size * 2 ||
	    hex2bin(v->root_digest, args.digest, v->digest_size)) {
		ti->error = "Invalid root digest";
		r = -EINVAL;
		goto bad;
	}

	if (strcmp(args.salt, "-")) {
		v->salt_size = strlen(args.salt) / 2;
		v->salt = kmalloc(v->salt_size, GFP_KERNEL);
		if (!v->salt) {
			ti->error = "Cannot allocate salt";
			r = -ENOMEM;
			goto bad;
		}
		if (strlen(args.salt) != v->salt_size * 2 ||
		    hex2bin(v->salt, args.salt, v->salt_size)) {
			ti->error = "Invalid salt";
			r = -EINVAL;
			goto bad;
		}
	}

	v->hash_per_block_bits =
		fls((1 << v->hash_dev_block_bits) / v->digest_size) - 1;

	v->levels = 0;
	if (v->data_blocks)
		while (v->hash_per_block_bits * v->levels < 64 &&
		       (unsigned long long)(v->data_blocks - 1) >>
		       (v->hash_per_block_bits * v->levels))
			v->levels++;

	if (v->levels > DM_VERITY_MAX_LEVELS) {
		ti->error = "Too many tree levels";
		r = -E2BIG;
		goto bad;
	}

	hash_position = v->hash_start;
	for (i = v->levels - 1; i >= 0; i--) {
		sector_t s;
		v->hash_level_block[i] = hash_position;
		s = (v->data_blocks + ((sector_t)1 << ((i + 1) * v->hash_per_block_bits)) - 1)
					>> ((i + 1) * v->hash_per_block_bits);
		if (hash_position + s < hash_position) {
			ti->error = "Hash device offset overflow";
			r = -E2BIG;
			goto bad;
		}
		hash_position += s;
	}
	v->hash_blocks = hash_position;

	v->bufio = dm_bufio_client_create(v->hash_dev->bdev,
		1 << v->hash_dev_block_bits, 1, sizeof(struct buffer_aux),
		dm_bufio_alloc_callback, NULL);
	if (IS_ERR(v->bufio)) {
		ti->error = "Cannot initialize dm-bufio";
		r = PTR_ERR(v->bufio);
		v->bufio = NULL;
		goto bad;
	}

	if (dm_bufio_get_device_size(v->bufio) < v->hash_blocks) {
		ti->error = "Hash device is too small";
		r = -E2BIG;
		goto bad;
	}

	ti->per_bio_data_size = roundup(sizeof(struct dm_verity_io) + v->shash_descsize + v->digest_size * 2, __alignof__(struct dm_verity_io));

	v->vec_mempool = mempool_create_kmalloc_pool(DM_VERITY_MEMPOOL_SIZE,
					BIO_MAX_PAGES * sizeof(struct bio_vec));
	if (!v->vec_mempool) {
		ti->error = "Cannot allocate vector mempool";
		r = -ENOMEM;
		goto bad;
	}

	/* WQ_UNBOUND greatly improves performance when running on ramdisk */
	v->verify_wq = alloc_workqueue("kverityd", WQ_CPU_INTENSIVE | WQ_MEM_RECLAIM | WQ_UNBOUND, num_online_cpus());
	if (!v->verify_wq) {
		ti->error = "Cannot allocate workqueue";
		r = -ENOMEM;
		goto bad;
	}

	/* chromeos allows setting error_behavior from both the module
	 * parameters and the device args.
	 */
	v->error_behavior = verity_parse_error_behavior(args.error_behavior);
	if (v->error_behavior == -1) {
		ti->error = "Bad error_behavior supplied";
		r = -EINVAL;
		goto bad;
	}

	return 0;

bad:
	verity_dtr(ti);

	return r;
}

static struct target_type verity_target = {
	.name		= "verity",
	.version	= {1, 2, 0},
	.module		= THIS_MODULE,
	.ctr		= verity_ctr,
	.dtr		= verity_dtr,
	.map		= verity_map,
	.status		= verity_status,
	.ioctl		= verity_ioctl,
	.merge		= verity_merge,
	.iterate_devices = verity_iterate_devices,
	.io_hints	= verity_io_hints,
};

static int __init dm_verity_init(void)
{
	int r;

	r = dm_register_target(&verity_target);
	if (r < 0)
		DMERR("register failed %d", r);

	return r;
}

static void __exit dm_verity_exit(void)
{
	dm_unregister_target(&verity_target);
}

module_init(dm_verity_init);
module_exit(dm_verity_exit);

MODULE_AUTHOR("Mikulas Patocka <mpatocka@redhat.com>");
MODULE_AUTHOR("Mandeep Baines <msb@chromium.org>");
MODULE_AUTHOR("Will Drewry <wad@chromium.org>");
MODULE_DESCRIPTION(DM_NAME " target for transparent disk integrity checking");
MODULE_LICENSE("GPL");
