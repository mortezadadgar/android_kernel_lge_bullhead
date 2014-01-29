/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
 *
 */

#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/stat.h>
#include <linux/tegra-soc.h>

#include "fuse.h"

int tegra_chip_id;
enum tegra_revision tegra_revision;

/*
 * The BCT to use at boot is specified by board straps that can be read
 * through a APB misc register and decoded. 2 bits, i.e. 4 possible BCTs.
 */
int tegra_bct_strapping;

#define TEGRA_STRAP_OPT		0x8
#define TEGRA_RAM_ID_SHIFT	4
#define TEGRA_RAM_ID_MASK	3

static u32 (*fuse_readl)(const unsigned int offset);
static u32 (*_fuse_write)(const unsigned int offset, const char *buf, u32 size);
static int fuse_size;
static void __iomem *fuse_base;
static void __iomem *apbmisc_base;
static void __iomem *car_base;

static const char *tegra_revision_name[TEGRA_REVISION_MAX] = {
	[TEGRA_REVISION_UNKNOWN] = "unknown",
	[TEGRA_REVISION_A01]     = "A01",
	[TEGRA_REVISION_A02]     = "A02",
	[TEGRA_REVISION_A03]     = "A03",
	[TEGRA_REVISION_A03p]    = "A03 prime",
	[TEGRA_REVISION_A04]     = "A04",
};

static u8 fuse_readb(const unsigned int offset)
{
	u32 val;

	val = fuse_readl(round_down(offset, 4));
	val >>= (offset % 4) * 8;
	val &= 0xff;

	return val;
}

static ssize_t fuse_read(struct file *fd, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t pos, size_t size)
{
	int i;

	if (pos < 0 || pos >= fuse_size)
		return 0;

	if (size > fuse_size - pos)
		size = fuse_size - pos;

	for (i = 0; i < size; i++)
		buf[i] = fuse_readb(pos + i);

	return i;
}

static ssize_t fuse_write(struct file *fd, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t pos, size_t size)
{
	int ret;

	if (!_fuse_write)
		return -EPERM;

	if (pos < 0 || pos >= fuse_size)
		return 0;

	if (size > fuse_size - pos)
		size = fuse_size - pos;

	ret = _fuse_write(pos, buf, size);

	if (IS_ERR_VALUE(ret))
		return -EPERM;

	return ret;
}

static struct bin_attribute fuse_bin_attr = {
	.attr = { .name = "fuse", .mode = S_IRUGO | S_IWUSR, },
	.read = fuse_read,
	.write = fuse_write,
};

static const struct of_device_id tegra_fuse_match[] __initconst = {
	{ .compatible = "nvidia,tegra20-efuse", },
	{ .compatible = "nvidia,tegra30-efuse", },
	{ .compatible = "nvidia,tegra114-efuse", },
	{ .compatible = "nvidia,tegra124-efuse", },
	{},
};

static const struct of_device_id car_match[] __initconst = {
	{ .compatible = "nvidia,tegra20-car", },
	{ .compatible = "nvidia,tegra30-car", },
	{ .compatible = "nvidia,tegra114-car", },
	{ .compatible = "nvidia,tegra124-car", },
	{},
};

static void tegra_read_bct_strapping(void)
{
	tegra_bct_strapping = readl_relaxed(apbmisc_base +
					    TEGRA_STRAP_OPT);
	tegra_bct_strapping >>= TEGRA_RAM_ID_SHIFT;
	tegra_bct_strapping &= TEGRA_RAM_ID_MASK;
}

static void tegra_get_revision(u32 id)
{
	u32 minor_rev = (id >> 16) & 0xf;

	switch (minor_rev) {
	case 1:
		tegra_revision = TEGRA_REVISION_A01;
		break;
	case 2:
		tegra_revision = TEGRA_REVISION_A02;
		break;
	case 3:
		if (tegra_chip_id == TEGRA20 &&
			(tegra20_spare_fuse_early(18, fuse_base) ||
			 tegra20_spare_fuse_early(19, fuse_base)))
			tegra_revision = TEGRA_REVISION_A03p;
		else
			tegra_revision = TEGRA_REVISION_A03;
		break;
	case 4:
		tegra_revision = TEGRA_REVISION_A04;
		break;
	default:
		tegra_revision = TEGRA_REVISION_UNKNOWN;
	}
}

u32 tegra_read_straps(void)
{
	return readl(apbmisc_base + TEGRA_STRAP_OPT);
}

u32 tegra_read_chipid(void)
{
	return readl_relaxed(apbmisc_base + 0x804);
}

int tegra_fuse_create_sysfs(struct device *dev, int size,
	 u32 (*readl)(const unsigned int offset),
	 u32 (*write)(const unsigned int offset, const char *buf, u32 size),
	 struct tegra_sku_info *sku_info)
{
	int err;

	if (fuse_size)
		return -ENODEV;

	fuse_bin_attr.size = size;
	fuse_bin_attr.read = fuse_read;
	fuse_bin_attr.write = fuse_write;

	fuse_size = size;
	fuse_readl = readl;
	_fuse_write = write;

	err = device_create_bin_file(dev, &fuse_bin_attr);
	if (err)
		return err;

	dev_info(dev,
		"Tegra Revision: %s SKU: %d CPU Process: %d Core Process: %d\n",
		tegra_revision_name[sku_info->revision],
		sku_info->sku_id, sku_info->cpu_process_id,
		sku_info->core_process_id);

	return 0;
}

void __init tegra_init_fuse(void)
{
	struct device_node *np;
	u32 id, reg;

	np = of_find_matching_node(NULL, tegra_fuse_match);
	fuse_base = of_iomap(np, 0);
	if (!fuse_base) {
		pr_err("ioremap tegra fuse failed\n");
		return;
	}

	apbmisc_base = of_iomap(np, 1);
	if (!apbmisc_base) {
		pr_err("ioremap tegra apbmisc failed\n");
		iounmap(fuse_base);
		return;
	}

	np = of_find_matching_node(NULL, car_match);
	car_base = of_iomap(np, 0);
	if (!car_base) {
		pr_err("ioremap tegra car failed\n");
		iounmap(fuse_base);
		iounmap(apbmisc_base);
		return;
	}

	reg = readl_relaxed(car_base + 0x48);
	reg |= 1 << 28;
	writel(reg, car_base + 0x48);

	/*
	 * Enable FUSE clock. This needs to be hardcoded because the clock
	 * subsystem is not active during early boot.
	 */
	reg = readl(car_base + 0x14);
	reg |= 1 << 7;
	writel(reg, car_base + 0x14);

	iounmap(car_base);

	id = tegra_read_chipid();
	tegra_chip_id = (id >> 8) & 0xff;

	tegra_read_bct_strapping();

	tegra_get_revision(id);
}
