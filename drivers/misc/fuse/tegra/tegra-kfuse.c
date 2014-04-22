/*
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/tegra-soc.h>

/* register definition */
#define KFUSE_STATE			0x80
#define KFUSE_STATE_DONE		(1u << 16)
#define KFUSE_STATE_CRCPASS		(1u << 17)
#define KFUSE_KEYADDR			0x88
#define KFUSE_KEYADDR_AUTOINC		(1u << 16)
#define KFUSE_KEYS			0x8c

struct tegra_kfuse_info {
	struct platform_device *pdev;
	void __iomem *base;
	struct clk *clk;
};

static struct tegra_kfuse_info *kfuse_info;

static inline u32 tegra_kfuse_readl(unsigned long offset)
{
	return readl_relaxed(kfuse_info->base + offset);
}

static inline void tegra_kfuse_writel(u32 value, unsigned long offset)
{
	writel_relaxed(value, kfuse_info->base + offset);
}

static int tegra_kfuse_wait_for_done(void)
{
	u32 reg;
	int retries = 50;
	do {
		reg = tegra_kfuse_readl(KFUSE_STATE);
		if (reg & KFUSE_STATE_DONE)
			return 0;
		usleep_range(10000, 11000);
	} while (--retries);
	return -ETIMEDOUT;
}

/* read up to TEGRA_KFUSE_DATA_SZ bytes into dest.
 * always starts at the first kfuse.
 */
int tegra_kfuse_read(void *dest, size_t len)
{
	int err;
	u32 v;
	unsigned cnt;

	if (len > TEGRA_KFUSE_DATA_SZ)
		return -EINVAL;

	err = clk_prepare_enable(kfuse_info->clk);
	if (err)
		return err;

	tegra_kfuse_writel(KFUSE_KEYADDR_AUTOINC, KFUSE_KEYADDR);

	err = tegra_kfuse_wait_for_done();
	if (err) {
		dev_err(&kfuse_info->pdev->dev, "Read timeout");
		clk_disable_unprepare(kfuse_info->clk);
		return err;
	}

	if ((tegra_kfuse_readl(KFUSE_STATE) & KFUSE_STATE_CRCPASS) == 0) {
		dev_err(&kfuse_info->pdev->dev, "CRC failed\n");
		clk_disable_unprepare(kfuse_info->clk);
		return -EIO;
	}

	for (cnt = 0; cnt < len; cnt += 4) {
		v = tegra_kfuse_readl(KFUSE_KEYS);
		memcpy(dest + cnt, &v, sizeof(v));
	}

	clk_disable_unprepare(kfuse_info->clk);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_kfuse_read);

static int tegra_kfuse_probe(struct platform_device *pdev)
{
	struct clk *fuse_clk;
	struct resource *res;
	int err = 0;

	kfuse_info = devm_kzalloc(&pdev->dev,
				  sizeof(struct tegra_kfuse_info), GFP_KERNEL);
	if (!kfuse_info)
		return -ENOMEM;

	fuse_clk = devm_clk_get(&pdev->dev, "kfuse");
	if (IS_ERR(fuse_clk)) {
		dev_err(&pdev->dev, "Could not obtain clk");
		return PTR_ERR(fuse_clk);
	}

	kfuse_info->clk = fuse_clk;
	kfuse_info->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	kfuse_info->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(kfuse_info->base)) {
		dev_err(&pdev->dev, "Could not obtain base address");
		return PTR_ERR(kfuse_info->base);
	}

	return err;
}

static const struct of_device_id tegra_kfuse_of_match[] = {
	{ .compatible = "nvidia,tegra-kfuse" },
	{},
};

static struct platform_driver tegra_kfuse_driver = {
	.probe = tegra_kfuse_probe,
	.driver = {
		.name		= "tegra_kfuse",
		.owner		= THIS_MODULE,
		.of_match_table	=  tegra_kfuse_of_match,
	}
};
module_platform_driver(tegra_kfuse_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tegra KFuse driver for accessing HDCP key stores");
