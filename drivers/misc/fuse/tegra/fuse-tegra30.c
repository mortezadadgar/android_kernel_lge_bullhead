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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/tegra-soc.h>

#include "fuse.h"

#define FUSE_BEGIN	0x100

#define FUSE_SKU_INFO		0x10

/* Tegra30 and later */
#define FUSE_VENDOR_CODE	0x100
#define FUSE_FAB_CODE		0x104
#define FUSE_LOT_CODE_0		0x108
#define FUSE_LOT_CODE_1		0x10c
#define FUSE_WAFER_ID		0x110
#define FUSE_X_COORDINATE	0x114
#define FUSE_Y_COORDINATE	0x118

#define FUSE_HAS_REVISION_INFO	BIT(0)

struct tegra_fuse_info {
	int	size;
	int	spare_bit;
	void	(*init_speedo_data)(struct tegra_sku_info *sku_info,
				    struct device *dev);
};

static void __iomem *fuse_base;
static struct clk *fuse_clk;
static struct tegra_fuse_info *fuse_info;
static struct tegra_sku_info sku_info;

u32 tegra30_fuse_readl(const unsigned int offset)
{
	u32 val;

	clk_prepare_enable(fuse_clk);

	val = readl_relaxed(fuse_base + FUSE_BEGIN + offset);

	clk_disable_unprepare(fuse_clk);

	return val;
}

bool tegra30_spare_fuse(int spare_bit)
{
	u32 offset = fuse_info->spare_bit + spare_bit * 4;

	return tegra30_fuse_readl(offset) & 1;
}

static void tegra30_fuse_add_randomness(void)
{
	u32 randomness[12];

	randomness[0] = tegra30_fuse_readl(FUSE_SKU_INFO);
	randomness[1] = tegra_read_straps();
	randomness[2] = tegra_read_chipid();
	randomness[3] = sku_info.cpu_process_id << 16;
	randomness[3] |= sku_info.core_process_id;
	randomness[4] = sku_info.cpu_speedo_id << 16;
	randomness[4] |= sku_info.soc_speedo_id;
	randomness[5] = tegra30_fuse_readl(FUSE_VENDOR_CODE);
	randomness[6] = tegra30_fuse_readl(FUSE_FAB_CODE);
	randomness[7] = tegra30_fuse_readl(FUSE_LOT_CODE_0);
	randomness[8] = tegra30_fuse_readl(FUSE_LOT_CODE_1);
	randomness[9] = tegra30_fuse_readl(FUSE_WAFER_ID);
	randomness[10] = tegra30_fuse_readl(FUSE_X_COORDINATE);
	randomness[11] = tegra30_fuse_readl(FUSE_Y_COORDINATE);

	add_device_randomness(randomness, sizeof(randomness));
}

static struct tegra_fuse_info tegra30_info = {
	.size			= 0x2a4,
	.spare_bit		= 0x144,
	.init_speedo_data	= tegra30_init_speedo_data,
};

static struct tegra_fuse_info tegra114_info = {
	.size			= 0x2a0,
	.init_speedo_data	= tegra114_init_speedo_data,
};

static struct tegra_fuse_info tegra124_info = {
	.size			= 0x300,
	.init_speedo_data	= tegra124_init_speedo_data,
};

static const struct of_device_id tegra30_fuse_of_match[] = {
	{ .compatible = "nvidia,tegra30-efuse", .data = &tegra30_info },
	{ .compatible = "nvidia,tegra114-efuse", .data = &tegra114_info },
	{ .compatible = "nvidia,tegra124-efuse", .data = &tegra124_info },
	{},
};

static int tegra30_fuse_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_dev_id;
	struct resource *res;

	of_dev_id = of_match_device(tegra30_fuse_of_match, &pdev->dev);
	if (!of_dev_id)
		return -ENODEV;
	fuse_info = (struct tegra_fuse_info *)of_dev_id->data;

	fuse_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(fuse_clk)) {
		dev_err(&pdev->dev, "missing clock");
		return PTR_ERR(fuse_clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fuse_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fuse_base)) {
		dev_err(&pdev->dev, "unable to map base address");
		return PTR_ERR(fuse_base);
	}

	sku_info.revision = tegra_revision;
	fuse_info->init_speedo_data(&sku_info, &pdev->dev);
	dev_dbg(&pdev->dev, "CPU Speedo ID %d, Soc Speedo ID %d",
		sku_info.cpu_speedo_id, sku_info.soc_speedo_id);

	tegra30_fuse_add_randomness();

	platform_set_drvdata(pdev, NULL);

	if (tegra_fuse_create_sysfs(&pdev->dev, fuse_info->size,
				    tegra30_fuse_readl, &sku_info))
		return -ENODEV;

	dev_dbg(&pdev->dev, "loaded\n");

	return 0;
}

static struct platform_driver tegra30_fuse_driver = {
	.probe = tegra30_fuse_probe,
	.driver = {
		.name = "tegra_fuse",
		.owner = THIS_MODULE,
		.of_match_table = tegra30_fuse_of_match,
	}
};

static int __init tegra30_fuse_init(void)
{
	return platform_driver_register(&tegra30_fuse_driver);
}
postcore_initcall(tegra30_fuse_init);

