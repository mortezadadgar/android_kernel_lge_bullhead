/*
 * Linux Security Module for Chromium OS
 *
 * Copyright 2011 Google Inc. All Rights Reserved
 *
 * Authors:
 *      Stephan Uphoff  <ups@google.com>
 *      Kees Cook       <keescook@chromium.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "Chromium OS LSM: " fmt

#include <asm/syscall.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/hashtable.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/path.h>
#include <linux/root_dev.h>
#include <linux/sched.h>	/* current and other task related stuff */
#include <linux/security.h>

#include "inode_mark.h"
#include "process_management.h"
#include "utils.h"

#define NUM_BITS 8 // 128 buckets in hash table

static DEFINE_HASHTABLE(process_setuid_policy_hashtable, NUM_BITS);

/*
 * Bool signifying whether to disable fixups for process management related
 * routines in the kernel (setuid, setgid, kill). Default value is false. Can
 * be overridden by 'disable_process_management_policies' flag. Static vars get
 * initialized to 0/false since in BSS.
 **/
static bool disable_process_management_policies;

/* Disable process management policies if flag passed */
static int set_disable_process_management_policies(char *str)
{
	disable_process_management_policies = true;
	return 1;
}
__setup("disable_process_management_policies=",
	set_disable_process_management_policies);

/*
 * Hash table entry to store process management policy signifying that 'parent'
 * user can use 'child' user for process management (for now that just means
 * 'parent' can set*uid() to 'child'). Will be adding exceptions for set*gid()
 * and kill() in the future.
 */
struct entry {
	struct hlist_node next;
	struct hlist_node dlist; /* for deletion cleanup */
	uint64_t parent_kuid;
	uint64_t child_kuid;
};

static DEFINE_HASHTABLE(sb_nosymfollow_hashtable, NUM_BITS);

struct sb_entry {
	struct hlist_node next;
	struct hlist_node dlist; /* for deletion cleanup */
	uintptr_t sb;
};

static int chromiumos_security_sb_mount(const char *dev_name, struct path *path,
					const char *type, unsigned long flags,
					void *data)
{
	int error = current->total_link_count ? -ELOOP : 0;

	if (error) {
		char *cmdline;

		cmdline = printable_cmdline(current);
		pr_notice("Mount path with symlinks prohibited - "
			"pid=%d cmdline=%s\n",
			task_pid_nr(current), cmdline);
		kfree(cmdline);
	}

	return error;
}

static void report_load_module(struct path *path, char *operation)
{
	char *alloced = NULL, *cmdline;
	char *pathname; /* Pointer to either static string or "alloced". */

	if (!path)
		pathname = "<unknown>";
	else {
		/* We will allow 11 spaces for ' (deleted)' to be appended */
		alloced = pathname = kmalloc(PATH_MAX+11, GFP_KERNEL);
		if (!pathname)
			pathname = "<no_memory>";
		else {
			pathname = d_path(path, pathname, PATH_MAX+11);
			if (IS_ERR(pathname))
				pathname = "<too_long>";
			else {
				pathname = printable(pathname);
				kfree(alloced);
				alloced = pathname;
			}
		}
	}

	cmdline = printable_cmdline(current);

	pr_notice("init_module %s module=%s pid=%d cmdline=%s\n",
		  operation, pathname, task_pid_nr(current), cmdline);

	kfree(cmdline);
	kfree(alloced);
}

static int module_locking = 1;
static struct dentry *locked_root;
static DEFINE_SPINLOCK(locked_root_spinlock);
static DEFINE_SPINLOCK(process_setuid_policy_hashtable_spinlock);
static DEFINE_SPINLOCK(sb_nosymfollow_hashtable_spinlock);

#ifdef CONFIG_SYSCTL
static int zero;
static int one = 1;

static struct ctl_path chromiumos_sysctl_path[] = {
	{ .procname = "kernel", },
	{ .procname = "chromiumos", },
	{ }
};

static struct ctl_table chromiumos_sysctl_table[] = {
	{
		.procname       = "module_locking",
		.data           = &module_locking,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.proc_handler   = proc_dointvec_minmax,
		.extra1         = &zero,
		.extra2         = &one,
	},
	{ }
};

/* Check if the root device is read-only (e.g. dm-verity is enabled).
 * This must be called after early kernel init, since then the rootdev
 * is available.
 */
static bool rootdev_readonly(void)
{
	bool rc;
	struct block_device *bdev;
	const fmode_t mode = FMODE_WRITE;

	bdev = blkdev_get_by_dev(ROOT_DEV, mode, NULL);
	if (IS_ERR(bdev)) {
		/* In this weird case, assume it is read-only. */
		pr_info("dev(%u,%u): FMODE_WRITE disallowed?!\n",
			MAJOR(ROOT_DEV), MINOR(ROOT_DEV));
		return true;
	}

	rc = bdev_read_only(bdev);
	blkdev_put(bdev, mode);

	pr_info("dev(%u,%u): %s\n", MAJOR(ROOT_DEV), MINOR(ROOT_DEV),
		rc ? "read-only" : "writable");

	return rc;
}

static void check_locking_enforcement(void)
{
	/* If module locking is not being enforced, allow sysctl to change
	 * modes for testing.
	 */
	if (!rootdev_readonly()) {
		if (!register_sysctl_paths(chromiumos_sysctl_path,
					   chromiumos_sysctl_table))
			pr_notice("sysctl registration failed!\n");
		else
			pr_info("module locking can be disabled.\n");
	} else
		pr_info("module locking engaged.\n");
}
#else
static void check_locking_enforcement(void) { }
#endif

/* Check for entry in hash table. */
static bool chromiumos_check_sb_nosymfollow_hashtable(struct super_block *sb)
{
	struct sb_entry *entry;
	uintptr_t sb_pointer = (uintptr_t)sb;
	bool found = false;

	rcu_read_lock();
	hash_for_each_possible_rcu(sb_nosymfollow_hashtable,
				   entry, next, sb_pointer) {
		if (entry->sb == sb_pointer) {
			found = true;
			break;
		}
	}
	rcu_read_unlock();

	/*
	 * Its possible that a policy gets added in between the time we check
	 * above and when we return false here. Such a race condition should
	 * not affect this check however, since it would only be relevant if
	 * userspace tried to traverse a symlink on a filesystem before that
	 * filesystem was done being mounted (or potentially while it was being
	 * remounted with new mount flags).
	 */
	return found;
}

/* Add entry to hash table. */
static int chromiumos_add_sb_nosymfollow_hashtable(struct super_block *sb)
{
	struct sb_entry *new;
	uintptr_t sb_pointer = (uintptr_t)sb;

	/* Return if entry already exists */
	if (chromiumos_check_sb_nosymfollow_hashtable(sb))
		return 0;

	new = kzalloc(sizeof(struct sb_entry), GFP_KERNEL);
	if (!new)
		return -ENOMEM;
	new->sb = sb_pointer;
	spin_lock(&sb_nosymfollow_hashtable_spinlock);
	hash_add_rcu(sb_nosymfollow_hashtable, &new->next, sb_pointer);
	spin_unlock(&sb_nosymfollow_hashtable_spinlock);
	return 0;
}

/* Flush all entries from hash table. */
void chromiumos_flush_sb_nosymfollow_hashtable(void)
{
	struct sb_entry *entry;
	struct hlist_node *hlist_node;
	unsigned int bkt_loop_cursor;
	HLIST_HEAD(free_list);

	/*
	 * Could probably use hash_for_each_rcu here instead, but this should
	 * be fine as well.
	 */
	spin_lock(&sb_nosymfollow_hashtable_spinlock);
	hash_for_each_safe(sb_nosymfollow_hashtable, bkt_loop_cursor,
			   hlist_node, entry, next) {
		hash_del_rcu(&entry->next);
		hlist_add_head(&entry->dlist, &free_list);
	}
	spin_unlock(&sb_nosymfollow_hashtable_spinlock);
	synchronize_rcu();
	hlist_for_each_entry_safe(entry, hlist_node, &free_list, dlist)
		kfree(entry);
}

/* Remove entry from hash table. */
static void chromiumos_remove_sb_nosymfollow_hashtable(struct super_block *sb)
{
	struct sb_entry *entry;
	struct hlist_node *hlist_node;
	uintptr_t sb_pointer = (uintptr_t)sb;
	bool free_entry = false;

	/*
	 * Could probably use hash_for_each_rcu here instead, but this should
	 * be fine as well.
	 */
	spin_lock(&sb_nosymfollow_hashtable_spinlock);
	hash_for_each_possible_safe(sb_nosymfollow_hashtable, entry,
			   hlist_node, next, sb_pointer) {
		if (entry->sb == sb_pointer) {
			hash_del_rcu(&entry->next);
			free_entry = true;
			break;
		}
	}
	spin_unlock(&sb_nosymfollow_hashtable_spinlock);
	if (free_entry) {
		synchronize_rcu();
		kfree(entry);
	}
}

int chromiumos_security_sb_umount(struct vfsmount *mnt, int flags)
{
	/* If mnt->mnt_sb is in nosymfollow hashtable, remove it. */
	chromiumos_remove_sb_nosymfollow_hashtable(mnt->mnt_sb);

	return 0;
}

static int chromiumos_security_load_module(struct file *file)
{
	struct dentry *module_root;

	if (!file) {
		if (!module_locking) {
			report_load_module(NULL, "old-api-locking-ignored");
			return 0;
		}

		report_load_module(NULL, "old-api-denied");
		return -EPERM;
	}

	module_root = file->f_path.mnt->mnt_root;

	/* First loaded module defines the root for all others. */
	spin_lock(&locked_root_spinlock);
	if (!locked_root) {
		locked_root = dget(module_root);
		/*
		 * Unlock now since it's only locked_root we care about.
		 * In the worst case, we will (correctly) report locking
		 * failures before we have announced that locking is
		 * enabled. This would be purely cosmetic.
		 */
		spin_unlock(&locked_root_spinlock);
		report_load_module(&file->f_path, "locked");
		check_locking_enforcement();
	} else {
		spin_unlock(&locked_root_spinlock);
	}

	if (module_root != locked_root) {
		if (unlikely(!module_locking)) {
			report_load_module(&file->f_path, "locking-ignored");
			return 0;
		}

		report_load_module(&file->f_path, "denied");
		return -EPERM;
	}

	return 0;
}

/*
 * NOTE: The WARN() calls will emit a warning in cases of blocked symlink
 * traversal attempts. These will show up in kernel warning reports
 * collected by the crash reporter, so we have some insight on spurious
 * failures that need addressing.
 */
static int chromiumos_security_inode_follow_link(struct dentry *dentry,
						 struct nameidata *nd)
{
	static char accessed_path[PATH_MAX];
	enum chromiumos_inode_security_policy policy;

	/* Deny if symlinks have been disabled on this superblock. */
	if (chromiumos_check_sb_nosymfollow_hashtable(dentry->d_sb)) {
		WARN(1,
		     "Blocked symlink traversal for path %x:%x:%s (symlinks were disabled on this FS through the 'nosymfollow' mount option)\n",
		     MAJOR(dentry->d_sb->s_dev),
		     MINOR(dentry->d_sb->s_dev),
		     dentry_path(dentry, accessed_path, PATH_MAX));
		return -EACCES;
	}

	policy = chromiumos_get_inode_security_policy(
		dentry,
		CHROMIUMOS_SYMLINK_TRAVERSAL);

	WARN(policy == CHROMIUMOS_INODE_POLICY_BLOCK,
	     "Blocked symlink traversal for path %x:%x:%s (see https://goo.gl/8xICW6 for context and rationale)\n",
	     MAJOR(dentry->d_sb->s_dev), MINOR(dentry->d_sb->s_dev),
	     dentry_path(dentry, accessed_path, PATH_MAX));

	return policy == CHROMIUMOS_INODE_POLICY_BLOCK ? -EACCES : 0;
}

int chromiumos_security_file_open(
	struct file *file,
	const struct cred *cred)
{
	static char accessed_path[PATH_MAX];
	enum chromiumos_inode_security_policy policy;
	struct dentry *dentry = file->f_path.dentry;

	/* Returns 0 if file is not a FIFO */
	if (!S_ISFIFO(file->f_inode->i_mode))
		return 0;

	policy = chromiumos_get_inode_security_policy(
		dentry,
		CHROMIUMOS_FIFO_ACCESS);

	/*
	 * Emit a warning in cases of blocked fifo access attempts. These will
	 * show up in kernel warning reports collected by the crash reporter,
	 * so we have some insight on spurious failures that need addressing.
	 */
	WARN(policy == CHROMIUMOS_INODE_POLICY_BLOCK,
	     "Blocked fifo access for path %x:%x:%s\n (see https://goo.gl/8xICW6 for context and rationale)\n",
	     MAJOR(dentry->d_sb->s_dev), MINOR(dentry->d_sb->s_dev),
	     dentry_path(dentry, accessed_path, PATH_MAX));

	return policy == CHROMIUMOS_INODE_POLICY_BLOCK ? -EACCES : 0;
}

bool chromiumos_check_setuid_policy_hashtable_key(kuid_t parent)
{
	struct entry *entry;

	rcu_read_lock();
	hash_for_each_possible_rcu(process_setuid_policy_hashtable,
				   entry, next, __kuid_val(parent)) {
		if (entry->parent_kuid == __kuid_val(parent)) {
			rcu_read_unlock();
			return true;
		}
	}
	rcu_read_unlock();

	/*
	 * Using RCU, its possible that a policy gets added in between the time
	 * we check above and when we return false here. This is fine, since
	 * policy updates only happen during system startup, well before
	 * sandboxed system services start running and the policies need to be
	 * queried.
	 */
	return false;
}

bool chromiumos_check_setuid_policy_hashtable_key_value(kuid_t parent,
							kuid_t child)
{
	struct entry *entry;

	rcu_read_lock();
	hash_for_each_possible_rcu(process_setuid_policy_hashtable,
				   entry, next, __kuid_val(parent)) {
		if (entry->parent_kuid == __kuid_val(parent) &&
		    entry->child_kuid == __kuid_val(child)) {
			rcu_read_unlock();
			return true;
		}
	}
	rcu_read_unlock();

	/*
	 * Using RCU, its possible that a policy gets added in between the time
	 * we check above and when we return false here. This is fine, since
	 * policy updates only happen during system startup, well before
	 * sandboxed system services start running and the policies need to be
	 * queried.
	 */
	return false;
}

bool setuid_syscall(int num)
{
#ifdef CONFIG_X86_64
	if (!(num == __NR_setreuid ||
	      num == __NR_setuid ||
	      num == __NR_setresuid ||
	      num == __NR_setfsuid))
		return false;
#elif defined CONFIG_ARM64
	if (!(num == __NR_compat_setuid ||
	      num == __NR_compat_setreuid ||
	      num == __NR_compat_setfsuid ||
	      num == __NR_compat_setresuid ||
	      num == __NR_compat_setreuid32 ||
	      num == __NR_compat_setresuid32 ||
	      num == __NR_compat_setuid32 ||
	      num == __NR_compat_setfsuid32))
		return false;
#else /* CONFIG_ARM */
	if (!(num == __NR_setreuid32 ||
	      num == __NR_setuid32 ||
	      num == __NR_setresuid32 ||
	      num == __NR_setfsuid32))
		return false;
#endif
	return true;
}

int chromiumos_security_capable(const struct cred *cred,
				struct user_namespace *ns,
				int cap)
{
	/* The current->mm check will fail if this is a kernel thread. */
	if (!disable_process_management_policies &&
	    cap == CAP_SETUID &&
	    current->mm &&
	    chromiumos_check_setuid_policy_hashtable_key(cred->uid)) {
		// syscall_get_nr can theoretically return 0 or -1, but that
		// would signify that the syscall is being aborted due to a
		// signal, so we don't need to check for this case here.
		if (!(setuid_syscall(syscall_get_nr(current,
						    current_pt_regs())))) {
			// Deny if we're not in a set*uid() syscall to avoid
			// giving powers gated by CAP_SETUID that are related
			// to functionality other than calling set*uid() (e.g.
			// allowing user to set up userns uid mappings).
			WARN(1,
			     "Operation requires CAP_SETUID, which is not available to UID %u for operations besides approved set*uid transitions\n",
			     __kuid_val(cred->uid));
			return -1;
		}
	}
	return 0;
}

/*
 * This hook inspects the string pointed to by the first parameter, looking for
 * the "nosymfollow" mount option. The second parameter points to an empty
 * page-sized buffer that is used for holding LSM-specific mount options that
 * are grabbed (after this function executes, in security_sb_copy_data) from
 * the mount string in the first parameter. Since the chromiumos LSM is stacked
 * ahead of SELinux for ChromeOS, the page-sized buffer is empty when this
 * function is called. If the "nosymfollow" mount option is encountered in this
 * function, we write "nosymflw" to the empty page-sized buffer which lets us
 * transmit information which will be visible in chromiumos_sb_kern_mount
 * signifying that symlinks should be disabled for the sb. We store this token
 * at a spot in the buffer that is at a greater offset than the bytes needed to
 * record the rest of the LSM-specific mount options (e.g. those for SELinux).
 * The "nosymfollow" option will be stripped from the mount string if it is
 * encountered.
 */
int chromiumos_sb_copy_data(char *orig, char *copy)
{
	char *orig_copy;
	char *orig_copy_cur;
	char *option;
	size_t offset = 0;
	bool found = false;

	if (!orig || *orig == 0)
		return 0;

	orig_copy = alloc_secdata();
	if (!orig_copy)
		return -ENOMEM;
	strncpy(orig_copy, orig, PAGE_SIZE);

	memset(orig, 0, strlen(orig));

	orig_copy_cur = orig_copy;
	while (orig_copy_cur) {
		option = strsep(&orig_copy_cur, ",");
		if (strcmp(option, "nosymfollow") == 0) {
			if (found) /* Found multiple times. */
				return -EINVAL;
			found = true;
		} else {
			if (offset > 0) {
				orig[offset] = ',';
				offset++;
			}
			strcpy(orig + offset, option);
			offset += strlen(option);
		}
	}

	if (found)
		strcpy(copy + offset + 1, "nosymflw");

	free_secdata(orig_copy);
	return 0;
}

/*
 * Emit a warning when no entry found in whitelist. These will show up in
 * kernel warning reports collected by the crash reporter, so we have some
 * insight regarding failures that need addressing.
 */
void chromiumos_setuid_policy_warning(kuid_t parent, kuid_t child)
{
	WARN(1,
	     "UID %u is restricted to using certain whitelisted UIDs for process management, and %u is not in the whitelist.\n",
	     __kuid_val(parent),
	     __kuid_val(child));
}

int chromiumos_check_uid_transition(kuid_t parent, kuid_t child)
{
	if (chromiumos_check_setuid_policy_hashtable_key_value(parent, child))
		return 0;
	chromiumos_setuid_policy_warning(parent, child);
	return -1;
}

/*
 * Check whether there is either an exception for user under old cred struct to
 * use user under new cred struct, or the UID transition is allowed (by Linux
 * set*uid rules) even without CAP_SETUID.
 */
int chromiumos_security_task_fix_setuid(struct cred *new,
					const struct cred *old, int flags)
{
	/*
	 * Do nothing if feature is turned off by kernel compile flag or there
	 * are no setuid restrictions for this UID.
	 */
	if (disable_process_management_policies ||
	    !chromiumos_check_setuid_policy_hashtable_key(old->uid))
		return 0;

	switch (flags) {
	case LSM_SETID_RE:
		/*
		 * Users for which setuid restrictions exist can only set the
		 * real UID to the real UID or the effective UID, unless an
		 * explicit whitelist policy allows the transition.
		 */
		if (!uid_eq(old->uid, new->uid) &&
			!uid_eq(old->euid, new->uid)) {
			return chromiumos_check_uid_transition(old->uid,
								new->uid);
		}
		/*
		 * Users for which setuid restrictions exist can only set the
		 * effective UID to the real UID, the effective UID, or the
		 * saved set-UID, unless an explicit whitelist policy allows
		 * the transition.
		 */
		if (!uid_eq(old->uid, new->euid) &&
			!uid_eq(old->euid, new->euid) &&
			!uid_eq(old->suid, new->euid)) {
			return chromiumos_check_uid_transition(old->euid,
								new->euid);
		}
		break;
	case LSM_SETID_ID:
		/*
		 * Users for which setuid restrictions exist cannot change the
		 * real UID or saved set-UID unless an explicit whitelist
		 * policy allows the transition.
		 */
		if (!uid_eq(old->uid, new->uid)) {
			return chromiumos_check_uid_transition(old->uid,
								new->uid);
		}
		if (!uid_eq(old->suid, new->suid)) {
			return chromiumos_check_uid_transition(old->suid,
								new->suid);
		}
		break;
	case LSM_SETID_RES:
		/*
		 * Users for which setuid restrictions exist cannot change the
		 * real UID, effective UID, or saved set-UID to anything but
		 * one of: the current real UID, the current effective UID or
		 * the current saved set-user-ID unless an explicit whitelist
		 * policy allows the transition.
		 */
		if (!uid_eq(new->uid, old->uid) &&
			!uid_eq(new->uid, old->euid) &&
			!uid_eq(new->uid, old->suid)) {
			return chromiumos_check_uid_transition(old->uid,
								new->uid);
		}
		if (!uid_eq(new->euid, old->uid) &&
			!uid_eq(new->euid, old->euid) &&
			!uid_eq(new->euid, old->suid)) {
			return chromiumos_check_uid_transition(old->euid,
								new->euid);
		}
		if (!uid_eq(new->suid, old->uid) &&
			!uid_eq(new->suid, old->euid) &&
			!uid_eq(new->suid, old->suid)) {
			return chromiumos_check_uid_transition(old->suid,
								new->suid);
		}
		break;
	case LSM_SETID_FS:
		/*
		 * Users for which setuid restrictions exist cannot change the
		 * filesystem UID to anything but one of: the current real UID,
		 * the current effective UID or the current saved set-UID
		 * unless an explicit whitelist policy allows the transition.
		 */
		if (!uid_eq(new->fsuid, old->uid)  &&
			!uid_eq(new->fsuid, old->euid)  &&
			!uid_eq(new->fsuid, old->suid) &&
			!uid_eq(new->fsuid, old->fsuid)) {
			return chromiumos_check_uid_transition(old->fsuid,
								new->fsuid);
		}
		break;
	}
	return 0;
}

/* Add process management policy to hash table */
int chromiumos_add_process_management_entry(kuid_t parent, kuid_t child)
{
	struct entry *new;

	/* Return if entry already exists */
	if (chromiumos_check_setuid_policy_hashtable_key_value(parent,
							       child))
		return 0;

	new = kzalloc(sizeof(struct entry), GFP_KERNEL);
	if (!new)
		return -ENOMEM;
	new->parent_kuid = __kuid_val(parent);
	new->child_kuid = __kuid_val(child);
	spin_lock(&process_setuid_policy_hashtable_spinlock);
	hash_add_rcu(process_setuid_policy_hashtable,
		     &new->next,
		     __kuid_val(parent));
	spin_unlock(&process_setuid_policy_hashtable_spinlock);
	return 0;
}

void chromiumos_flush_process_management_entries(void)
{
	struct entry *entry;
	struct hlist_node *hlist_node;
	unsigned int bkt_loop_cursor;
	HLIST_HEAD(free_list);

	/*
	 * Could probably use hash_for_each_rcu here instead, but this should
	 * be fine as well.
	 */
	hash_for_each_safe(process_setuid_policy_hashtable, bkt_loop_cursor,
			   hlist_node, entry, next) {
		spin_lock(&process_setuid_policy_hashtable_spinlock);
		hash_del_rcu(&entry->next);
		spin_unlock(&process_setuid_policy_hashtable_spinlock);
		hlist_add_head(&entry->dlist, &free_list);
	}
	synchronize_rcu();
	hlist_for_each_entry_safe(entry, hlist_node, &free_list, dlist)
		kfree(entry);
}

static struct security_operations chromiumos_security_ops = {
	.name	= "chromiumos",
	.sb_mount = chromiumos_security_sb_mount,
	.kernel_module_from_file = chromiumos_security_load_module,
	.inode_follow_link = chromiumos_security_inode_follow_link,
	.file_open = chromiumos_security_file_open,
};

/* Unfortunately the kernel doesn't implement memmem function. */
static void *search_buffer(void *haystack, size_t haystacklen,
			   const void *needle, size_t needlelen)
{
	if (!needlelen)
		return (void *)haystack;
	while (haystacklen >= needlelen) {
		haystacklen--;
		if (!memcmp(haystack, needle, needlelen))
			return (void *)haystack;
		haystack++;
	}
	return NULL;
}

int chromiumos_sb_kern_mount(struct super_block *sb, int flags, void *data)
{
	int ret;
	char search_str[10] = "\0nosymflw";

	if (!data)
		return 0;

	if (search_buffer(data, PAGE_SIZE, search_str, 10)) {
		ret = chromiumos_add_sb_nosymfollow_hashtable(sb);
		if (ret)
			return ret;
	}

	return 0;
}

static int __init chromiumos_security_init(void)
{
	int error;

	error = register_security(&chromiumos_security_ops);

	if (error)
		panic("Could not register Chromium OS security module");

	return error;
}
security_initcall(chromiumos_security_init);

/* Should not be mutable after boot, so not listed in sysfs (perm == 0). */
module_param(module_locking, int, 0);
MODULE_PARM_DESC(module_locking, "Module loading restrictions (default: true)");
