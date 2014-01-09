/*
 *  intel_sst_debug.c - Intel SST Driver debugfs support
 *
 *  Copyright (C) 2012	Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *		Omair Mohammed Abdullah <omair.m.abdullah@linux.intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This driver exposes the audio engine functionalities to the ALSA
 *	 and middleware.
 *
 *  This file contains all debugfs functions
 *  Support includes:
 *   - Disabling/Enabling runtime PM for SST
 *   - Reading/Writing LPE SHIM registers
 *   - Reading/Enabling Input OSC Clock
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": debugfs: " fmt

#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_ipcutil.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

/* FIXME: replace with simple_open from 3.4 kernel */
static int sst_debug_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

static ssize_t sst_debug_shim_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct intel_sst_drv *drv = file->private_data;
	unsigned long long val = 0;
	char buf[256];
	int pos = 0;

	if (drv->sst_state == SST_SUSPENDED) {
		pr_err("FW suspended, cannot read SHIM registers\n");
		return -EFAULT;
	}
	if (drv->pci_id == SST_MFLD_PCI_ID
			|| drv->pci_id == SST_CLV_PCI_ID) {
		val = sst_shim_read(drv->shim, SST_CSR);
		pos += sprintf(buf + pos, "CSR:  0x%llx\n", val);
		val = sst_shim_read(drv->shim, SST_PISR);
		pos += sprintf(buf + pos, "PISR: 0x%llx\n", val);
		val = sst_shim_read(drv->shim, SST_PIMR);
		pos += sprintf(buf + pos, "PIMR: 0x%llx\n", val);
		val = sst_shim_read(drv->shim, SST_ISRX);
		pos += sprintf(buf + pos, "ISRX: 0x%llx\n", val);
		val = sst_shim_read(drv->shim, SST_IMRX);
		pos += sprintf(buf + pos, "IMRX: 0x%llx\n", val);
		val = sst_shim_read(drv->shim, SST_IPCX);
		pos += sprintf(buf + pos, "IPCX: 0x%llx\n", val);
		val = sst_shim_read(drv->shim, SST_IPCD);
		pos += sprintf(buf + pos, "IPCD: 0x%llx\n", val);
		val = sst_shim_read(drv->shim, SST_ISRD);
		pos += sprintf(buf + pos, "ISRD: 0x%llx\n", val);
		val = sst_shim_read(drv->shim, SST_CSR2);
		pos += sprintf(buf + pos, "CSR2: 0x%llx\n", val);
	}
	else if(drv->pci_id == SST_BYT_PCI_ID)
	{
		val = sst_shim_read64(drv->shim, SST_CSR);
		pos += sprintf(buf + pos, "CSR:  0x%llx\n", val);
		val = sst_shim_read64(drv->shim, SST_PISR);
		pos += sprintf(buf + pos, "PISR: 0x%llx\n", val);
		val = sst_shim_read64(drv->shim, SST_PIMR);
		pos += sprintf(buf + pos, "PIMR: 0x%llx\n", val);
		val = sst_shim_read64(drv->shim, SST_ISRX);
		pos += sprintf(buf + pos, "ISRX: 0x%llx\n", val);
		val = sst_shim_read64(drv->shim, SST_IMRX);
		pos += sprintf(buf + pos, "IMRX: 0x%llx\n", val);
		val = sst_shim_read64(drv->shim, SST_IPCX);
		pos += sprintf(buf + pos, "IPCX: 0x%llx\n", val);
		val = sst_shim_read64(drv->shim, SST_IPCD);
		pos += sprintf(buf + pos, "IPCD: 0x%llx\n", val);
		val = sst_shim_read64(drv->shim, SST_ISRD);
		pos += sprintf(buf + pos, "ISRD: 0x%llx\n", val);
		val = sst_shim_read64(drv->shim, SST_CSR2);
		pos += sprintf(buf + pos, "CSR2: 0x%llx\n", val);
	}
	else{}

	return simple_read_from_buffer(user_buf, count, ppos,
			buf, strlen(buf));
}

static ssize_t sst_debug_shim_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct intel_sst_drv *drv = file->private_data;
	char buf[32];
	char regname[32];
	char *start = buf, *end;
	unsigned long value;
	unsigned long long wreg;
	size_t buf_size = min(count, sizeof(buf)-1);
	int ret_val;

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	if (drv->sst_state == SST_SUSPENDED) {
		pr_err("FW suspended, cannot write SHIM registers\n");
		return -EFAULT;
	}
	if (drv->pci_id == SST_MFLD_PCI_ID
			|| drv->pci_id == SST_CLV_PCI_ID) {
		while (*start == ' ')
			start++;
		end = start;
		while (isalnum(*end))
			end++;
		strncpy(regname, start, end - start);
		regname[end - start] = 0;

		start = end;
		while (*start == ' ')
			start++;

		ret_val = kstrtoul(start, 16, &value);
		if (ret_val) {
			pr_err("kstrtoul failed, ret_val = %d\n", ret_val);
			return ret_val;
		}

		if (!strncmp(regname, "CSR", 4))
			reg = SST_CSR;
		else if (!strncmp(regname, "PISR", 4))
			reg = SST_PISR;
		else if (!strncmp(regname, "PIMR", 4))
			reg = SST_PIMR;
		else if (!strncmp(regname, "ISRX", 4))
			reg = SST_ISRX;
		else if (!strncmp(regname, "IMRX", 4))
			reg = SST_IMRX;
		else if (!strncmp(regname, "IPCX", 4))
			reg = SST_IPCX;
		else if (!strncmp(regname, "IPCD", 4))
			reg = SST_IPCD;
		else if (!strncmp(regname, "ISRD", 4))
			reg = SST_ISRD;
		else if (!strncmp(regname, "CSR2", 4))
			reg = SST_CSR2;
		else
			return -EINVAL;

		pr_debug("writing shim: %s(0x%llx)=0x%lx", regname, reg, value);
		sst_shim_write(drv->shim, reg, (u32) value);
	}

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER);

	return buf_size;
}

static const struct file_operations sst_debug_shim_ops = {
	.open = sst_debug_open,
	.read = sst_debug_shim_read,
	.write = sst_debug_shim_write,
	.llseek = default_llseek,
};

static ssize_t sst_debug_rtpm_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct intel_sst_drv *drv = file->private_data;
	char *status;
	int usage = atomic_read(&drv->pci->dev.power.usage_count);

	pr_debug("RTPM usage: %d\n", usage);
	status = drv->debugfs.runtime_pm_status ? "enabled\n" : "disabled\n";

	return simple_read_from_buffer(user_buf, count, ppos,
			status, strlen(status));
}

static ssize_t sst_debug_rtpm_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct intel_sst_drv *drv = file->private_data;
	char buf[16];
	int sz = min(count, sizeof(buf)-1);
	int usage = atomic_read(&drv->pci->dev.power.usage_count);

	if (copy_from_user(buf, user_buf, sz))
		return -EFAULT;
	buf[sz] = 0;

	pr_debug("RTPM Usage: %d\n", usage);

	if (!strncmp(buf, "enable\n", sz)) {
		/* already enabled? */
		if (drv->debugfs.runtime_pm_status)
			return -EINVAL;
		drv->debugfs.runtime_pm_status = 1;
		pm_runtime_allow(&sst_drv_ctx->pci->dev);
		sz = 6; /* strlen("enable") */
	} else if (!strncmp(buf, "disable\n", sz)) {
		if (!drv->debugfs.runtime_pm_status)
			return -EINVAL;
		drv->debugfs.runtime_pm_status = 0;
		pm_runtime_forbid(&sst_drv_ctx->pci->dev);
		sz = 7; /* strlen("disable") */
	} else
		return -EINVAL;

	return sz;
}

static const struct file_operations sst_debug_rtpm_ops = {
	.open = sst_debug_open,
	.read = sst_debug_rtpm_read,
	.write = sst_debug_rtpm_write,
	.llseek = default_llseek,
};


static ssize_t sst_debug_readme_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	const char *buf =
		"All files can be read using 'cat'\n"
		"1. 'echo disable > runtime_pm' disables runtime PM and will prevent SST from suspending.\n"
		"To enable runtime PM, echo 'enable' to runtime_pm.\n"
		"2. Write to shim register using 'echo CSR 0x192 > shim_dump'.\n"
		"3. Enable input clock by 'echo enable > osc_clk0'.\n";

	return simple_read_from_buffer(user_buf, count, ppos,
			buf, strlen(buf));
}

static const struct file_operations sst_debug_readme_ops = {
	.open = sst_debug_open,
	.read = sst_debug_readme_read,
};

static ssize_t sst_debug_osc_clk0_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
/*	char status[16];
	int mode = intel_scu_ipc_set_osc_clk0(0, CLK0_QUERY);

	snprintf(status, 16, "0x%x\n", mode);
	return simple_read_from_buffer(user_buf, count, ppos,
			status, strlen(status));
*/
}

static ssize_t sst_debug_osc_clk0_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[16];
	int sz = min(count, sizeof(buf)-1);

	if (copy_from_user(buf, user_buf, sz))
		return -EFAULT;
	buf[sz] = 0;

/*Currently scu ipc is not used by BYT */
	if (!strncmp(buf, "enable\n", sz)) {
		/*intel_scu_ipc_set_osc_clk0(true, CLK0_DEBUG);*/
		sz = 6; /* strlen("enable") */
	} else if (!strncmp(buf, "disable\n", sz)) {
 		/*intel_scu_ipc_set_osc_clk0(false, CLK0_DEBUG);*/
		sz = 7; /* strlen("disable") */
	} else
		return -EINVAL;

	return sz;
}

static const struct file_operations sst_debug_osc_clk0_ops = {
	.open = sst_debug_open,
	.read = sst_debug_osc_clk0_read,
	.write = sst_debug_osc_clk0_write,
};

void sst_debugfs_init(struct intel_sst_drv *sst)
{
	sst->debugfs.root = debugfs_create_dir("sst", NULL);
	if (IS_ERR(sst->debugfs.root) || !sst->debugfs.root) {
		pr_err("Failed to create debugfs directory\n");
		return;
	}
	/* Runtime PM enable/disable */
	if (!debugfs_create_file("runtime_pm", 0600, sst->debugfs.root,
				sst, &sst_debug_rtpm_ops)) {
		pr_err("Failed to create runtime_pm file\n");
		return;
	}
	/* Initial status is enabled */
	sst->debugfs.runtime_pm_status = 1;

	/* For dumping shim registers */
	if (!debugfs_create_file("shim_dump", 0600, sst->debugfs.root,
				sst, &sst_debug_shim_ops)) {
		pr_err("Failed to create shim_dump file\n");
		return;
	}

	/* For Reading/Enabling OSC Clock */
	if (!debugfs_create_file("osc_clk0", 0600, sst->debugfs.root,
				sst, &sst_debug_osc_clk0_ops)) {
		pr_err("Failed to create osc_clk0 file\n");
		return;
	}

	/* README file for user help */
	if (!debugfs_create_file("README", 0400, sst->debugfs.root,
				sst, &sst_debug_readme_ops)) {
		pr_err("Failed to create README file\n");
		return;
	}
}

void sst_debugfs_exit(struct intel_sst_drv *sst)
{
	if (sst->debugfs.runtime_pm_status)
		pm_runtime_allow(&sst->pci->dev);
	debugfs_remove_recursive(sst->debugfs.root);
}
