/*
 * Copyright(c) 2015 Intel Deutschland GmbH
 *
 * Backport functionality introduced in Linux 4.4.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#ifdef CONFIG_DEBUG_FS
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
static ssize_t _debugfs_read_file_bool(struct file *file, char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	char buf[3];
	bool *val = file->private_data;

	if (*val)
		buf[0] = 'Y';
	else
		buf[0] = 'N';
	buf[1] = '\n';
	buf[2] = 0x00;
	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t _debugfs_write_file_bool(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[32];
	size_t buf_size;
	bool bv;
	bool *val = file->private_data;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	buf[buf_size] = '\0';
	if (strtobool(buf, &bv) == 0)
		*val = bv;

	return count;
}

static const struct file_operations fops_bool = {
	.read =		_debugfs_read_file_bool,
	.write =	_debugfs_write_file_bool,
	.open =		simple_open,
	.llseek =	default_llseek,
};

struct dentry *iwl_debugfs_create_bool(const char *name, umode_t mode,
				       struct dentry *parent, bool *value)
{
	return debugfs_create_file(name, mode, parent, value, &fops_bool);
}
EXPORT_SYMBOL_GPL(iwl_debugfs_create_bool);
#endif /* < 4.4.0 */
#endif /* CONFIG_DEBUG_FS */
