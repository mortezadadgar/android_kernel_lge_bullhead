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
 #include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/regulator/consumer.h>
#include <linux/tegra-soc.h>

#include "fuse.h"

#define FUSE_CTRL	0x00
#define FUSE_REG_ADDR	0x04
#define FUSE_REG_READ	0x08
#define FUSE_REG_WRITE	0x0c
#define FUSE_TIME_PGM2	0x1c
#define FUSE_PRIV2INTFC_START	0x20
#define FUSE_DIS_PGM	0x2c
#define FUSE_WRITE_ACCESS	0x30
#define FUSE_PWR_GOOD_SW	0x34

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

#define FUSE_READ	0x1
#define FUSE_WRITE	0x2
#define FUSE_SENSE	0x3
#define FUSE_CMD_MASK	0x3

#define STATE_IDLE  (0x4 << 16)
#define SENSE_DONE  (0x1 << 30)

#define FUSETIME_PGM2_TWIDTH_PGM_MASK	0xffff

#define PRIV2INTFC_START_DATA	BIT(0)
#define PRIV2INTFC_SKIP_RAMREPAIR	BIT(1)

struct tegra_fuse_info {
	int	size;
	int	spare_bit;
	void	(*init_speedo_data)(struct tegra_sku_info *sku_info,
				    struct device *dev);
};

struct tegra_fuse_location {
	int fuse_addr;
	int start_bit;
	int end_bit;
};

struct tegra_fuse_data {
	int regular_addr;
	struct tegra_fuse_location loc_high;
	struct tegra_fuse_location loc_low;
	int bits_num;
};

static struct tegra_fuse_data tegra30_fuse_array[] = {
	{0x0a0, { 0,  0,  0}, { 0, 23, 23},  1},
	{0x0b8, { 0,  0,  0}, { 0, 24, 24},  1},
	{0x0bc, {26,  0,  5}, {24, 22, 31}, 16},
	{0x0c0, { 0,  0,  0}, {26,  6, 13},  8},
};

static struct tegra_fuse_data tegra114_fuse_array[] = {
	{0x0a0, { 0,  0,  0}, { 0,  7,  7},  1},
	{0x0b8, { 0,  0,  0}, { 0,  8,  8},  1},
	{0x0bc, { 0,  0,  0}, {46,  7, 22}, 16},
	{0x0c0, { 0,  0,  0}, {46, 23, 30},  8},
	{0x168, { 0,  0,  0}, {90, 22, 22},  1},
};

static struct tegra_fuse_data tegra124_fuse_array[] = {
	{0x0a0, { 0,  0,  0}, { 0, 11, 11},  1},
	{0x0b8, { 0,  0,  0}, { 0, 12, 12},  1},
	{0x0bc, { 0,  0,  0}, {44, 12, 27}, 16},
	{0x0c0, {46,  0,  3}, {44, 28, 31},  8},
	{0x0c8, {48,  0,  4}, {46,  5, 31}, 32},
	{0x0cc, {50,  0,  4}, {48,  5, 31}, 32},
	{0x0d0, {52,  0,  4}, {50,  5, 31}, 32},
	{0x0d4, {54,  0,  4}, {52,  5, 31}, 32},
	{0x0d8, {56,  0,  4}, {54,  5, 31}, 32},
	{0x0e0, {60,  0,  4}, {58,  5, 31}, 32},
	{0x0e4, {62,  0,  4}, {60,  5, 31}, 32},
	{0x168, { 0,  0,  0}, {90,  9,  9},  1},
};

static struct tegra_fuse_data *fuse_array;
static int fuse_array_size;
static void __iomem *fuse_base;
static struct clk *fuse_clk;
static struct tegra_fuse_info *fuse_info;
static struct tegra_sku_info sku_info;
static struct regulator *vpp_reg;
static struct platform_device *fuse_pdev;
static u32 pgm_cycles;
static bool need_sense_done;

int tegra_get_cpu_process_id(void)
{
	return sku_info.cpu_process_id;
}

int tegra_get_core_process_id(void)
{
	return sku_info.core_process_id;
}

int tegra_get_gpu_process_id(void)
{
	return sku_info.gpu_process_id;
}

int tegra_get_cpu_speedo_id(void)
{
	if (tegra_chip_id == TEGRA20)
		return -EINVAL;

	return sku_info.cpu_speedo_id;
}

int tegra_get_soc_speedo_id(void)
{
	return sku_info.soc_speedo_id;
}

int tegra_get_gpu_speedo_id(void)
{
	return sku_info.gpu_speedo_id;
}

int tegra_get_cpu_speedo_value(void)
{
	return sku_info.cpu_speedo_value;
}

int tegra_get_gpu_speedo_value(void)
{
	return sku_info.gpu_speedo_value;
}

int tegra_get_cpu_iddq_value(void)
{
	return sku_info.cpu_iddq_value;
}

void tegra_gpu_get_info(struct gpu_info *pinfo)
{
	if (tegra_chip_id == TEGRA114) {
		pinfo->num_pixel_pipes = 4;
		pinfo->num_alus_per_pixel_pipe = 3;
	} else {
		pinfo->num_pixel_pipes = 1;
		pinfo->num_alus_per_pixel_pipe = 1;
	}
}

static inline void wait_for_idle(void)
{
	u32 reg;

	do {
		udelay(1);
		reg = readl_relaxed(fuse_base + FUSE_CTRL);
	} while ((reg & (0x1F << 16)) != STATE_IDLE);
}

static inline void wait_for_sense_done(void)
{
	u32 reg;

	writel_relaxed(PRIV2INTFC_START_DATA | PRIV2INTFC_SKIP_RAMREPAIR,
			fuse_base + FUSE_PRIV2INTFC_START);

	do {
		udelay(1);
		reg = readl_relaxed(fuse_base + FUSE_CTRL);
	} while ((reg & BIT(30)) != SENSE_DONE ||
			(reg & (0x1F << 16)) != STATE_IDLE);

}

static u32 fuse_cmd_read(int addr)
{
	u32 reg;

	wait_for_idle();

	writel_relaxed(addr, fuse_base + FUSE_REG_ADDR);

	reg = readl_relaxed(fuse_base + FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_READ;
	writel_relaxed(reg, fuse_base + FUSE_CTRL);

	wait_for_idle();

	return readl_relaxed(fuse_base + FUSE_REG_READ);
}

/* Must be called by fuse_set_value() */
static void fuse_cmd_write(int addr, u32 value)
{
	u32 reg;

	wait_for_idle();

	writel_relaxed(addr, fuse_base + FUSE_REG_ADDR);
	writel_relaxed(value, fuse_base + FUSE_REG_WRITE);

	reg = readl_relaxed(fuse_base + FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_WRITE;
	writel_relaxed(reg, fuse_base + FUSE_CTRL);

	wait_for_idle();
}

static u32 fuse_get_value(int index)
{
	u32 low_bits, high_bits, val;
	struct tegra_fuse_location *low, *high;
	int bits_num[2] = {0, 0};

	if (index >= fuse_array_size || !fuse_array[index].bits_num)
		return ~0;

	low = &fuse_array[index].loc_low;
	high = &fuse_array[index].loc_high;

	bits_num[0] = low->end_bit + 1 - low->start_bit;

	low_bits = fuse_cmd_read(low->fuse_addr);
	low_bits >>= low->start_bit;
	if (bits_num[0] < 32)
		low_bits &= BIT(bits_num[0]) - 1;

	high_bits = 0;
	if (fuse_array[index].bits_num - bits_num[0] != 0) {
		bits_num[1] = high->end_bit + 1 - high->start_bit;

		high_bits = fuse_cmd_read(high->fuse_addr);
		high_bits >>= high->start_bit;

		if (bits_num[1] < 32)
			high_bits &= BIT(bits_num[1]) - 1;
	}

	val = (high_bits << bits_num[0]) | low_bits;

	return val;
}

/* Must be called by tegra30_fuse_program() */
static void fuse_set_value(int index, u32 value)
{
	u32 low_bits, high_bits;
	struct tegra_fuse_location *low, *high;
	int bits_num[2] = {0, 0};

	if (index >= fuse_array_size || !fuse_array[index].bits_num)
		return;

	low = &fuse_array[index].loc_low;
	high = &fuse_array[index].loc_high;

	bits_num[0] = low->end_bit + 1 - low->start_bit;
	if (fuse_array[index].bits_num - bits_num[0] != 0)
		bits_num[1] = high->end_bit + 1 - high->start_bit;

	if (bits_num[0] < 32)
		low_bits = value & (BIT(bits_num[0]) - 1);
	else
		low_bits = value;
	low_bits <<= low->start_bit;

	if (low_bits) {
		fuse_cmd_write(low->fuse_addr, low_bits);
		/* also write to redundant fuse */
		fuse_cmd_write(low->fuse_addr + 1, low_bits);
	}

	if (bits_num[1]) {
		high_bits = value >> bits_num[0];
		high_bits <<= high->start_bit;
		if (high_bits) {
			fuse_cmd_write(high->fuse_addr, high_bits);
			/* also write to redundant fuse */
			fuse_cmd_write(high->fuse_addr + 1, high_bits);
		}
	}
}

u32 tegra30_fuse_readl(const unsigned int offset)
{
	u32 val;

	clk_prepare_enable(fuse_clk);

	val = readl_relaxed(fuse_base + FUSE_BEGIN + offset);

	clk_disable_unprepare(fuse_clk);

	return val;
}

static void tegra30_fuse_sense(void)
{
	u32 reg;

	wait_for_idle();

	reg = readl_relaxed(fuse_base + FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_SENSE;
	writel_relaxed(reg, fuse_base + FUSE_CTRL);

	wait_for_idle();
}

static bool regulator_is_available(void)
{
	/* First time initialization */
	if (!vpp_reg) {
		/* Get power supply for fuse programming */
		vpp_reg = devm_regulator_get(&fuse_pdev->dev, "vdd");
		if (IS_ERR(vpp_reg))
			return false;
	} else if (IS_ERR(vpp_reg))
		return false;

	return true;
}

/*
 * Make the programming function as static and only allowed to be called
 * from fuse-tegra.c by sysfs call.
 */
static u32 tegra30_fuse_program(const unsigned int offset, const char *buf,
	u32 size)
{
	int ret;
	u32 val;
	int index = 0;
	int result = 0;
	int i, j;

	if (!regulator_is_available())
		return -EPERM;

	clk_prepare_enable(fuse_clk);

	/*
	 * Confirm fuse option write access hasn't already been permanenttly
	 * disabled
	 */
	val = readl_relaxed(fuse_base + FUSE_DIS_PGM);
	if (val)
		goto err_pgm_disabled;

	/* Enable software writes to fuse registers */
	writel_relaxed(0x0, fuse_base + FUSE_WRITE_ACCESS);

	/* Set the fuse strobe programming width */
	if (pgm_cycles)
		writel_relaxed(pgm_cycles, fuse_base + FUSE_TIME_PGM2);

	/* Turn on 1.8V power supply */
	ret = regulator_enable(vpp_reg);
	if (ret) {
		WARN(1, "failed to enable vpp_fuse power supply\n");
		goto err_reg;
	}

	/* Enable power */
	writel_relaxed(0x1, fuse_base + FUSE_PWR_GOOD_SW);
	udelay(1);

	for (i = 0; i < size; ) {
		int addr = offset + i;
		int remainder = addr % 4;
		u32 data = 0;
		int k;

		for (j = remainder; j < 4; j++) {
			data |= buf[index++] << (j * 8);
			if (index >= size)
				break;
		}

		/*
		 * Only set the bits that should be burned, not any bits that
		 * have alreardy been burned.
		 */
		for (k = 0; k < fuse_array_size; k++) {
			if (round_down(addr, 4) == fuse_array[k].regular_addr) {
				val = fuse_get_value(k);
				break;
			}
		}

		if (k == fuse_array_size)
			break;

		data &= ~val;
		if (data)
			fuse_set_value(k, data);

		i += 4 - remainder;
	}

	result = size;

	/* Disable power */
	writel_relaxed(0x0, fuse_base + FUSE_PWR_GOOD_SW);
	udelay(1);

	regulator_disable(vpp_reg);

	tegra30_fuse_sense();

	if (need_sense_done)
		wait_for_sense_done();

err_reg:

	/* Disable software writes to fuse registers */
	writel_relaxed(0x1, fuse_base + FUSE_WRITE_ACCESS);

err_pgm_disabled:

	clk_disable_unprepare(fuse_clk);

	return result;
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
	struct clk *osc_clk;

	of_dev_id = of_match_device(tegra30_fuse_of_match, &pdev->dev);
	if (!of_dev_id)
		return -ENODEV;
	fuse_info = (struct tegra_fuse_info *)of_dev_id->data;

	if (!strcmp(of_dev_id->compatible, "nvidia,tegra30-efuse"))
		need_sense_done = true;

	if (!strcmp(of_dev_id->compatible, "nvidia,tegra30-efuse")) {
		fuse_array = tegra30_fuse_array;
		fuse_array_size = ARRAY_SIZE(tegra30_fuse_array);
	} else if (!strcmp(of_dev_id->compatible, "nvidia,tegra114-efuse")) {
		fuse_array = tegra114_fuse_array;
		fuse_array_size = ARRAY_SIZE(tegra114_fuse_array);
	} else {
		fuse_array = tegra124_fuse_array;
		fuse_array_size = ARRAY_SIZE(tegra124_fuse_array);
	}

	fuse_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(fuse_clk)) {
		dev_err(&pdev->dev, "missing clock");
		return PTR_ERR(fuse_clk);
	}

	osc_clk = devm_clk_get(&pdev->dev, "clk_m");
	if (IS_ERR(osc_clk)) {
		/* Should be impossible to see this */
		dev_err(&pdev->dev, "failed to get clk_m");
		return PTR_ERR(osc_clk);
	}

	/*
	 * The strobe programming pulse is 12us based on the osc clock
	 * frequency
	 */
	pgm_cycles = DIV_ROUND_UP(clk_get_rate(osc_clk) * 12, 1000 * 1000);
	pgm_cycles &= FUSETIME_PGM2_TWIDTH_PGM_MASK;
	dev_dbg(&pdev->dev, "pgm_cycles is %d\n", pgm_cycles);

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

	fuse_pdev = pdev;

	if (tegra_fuse_create_sysfs(&pdev->dev, fuse_info->size,
			tegra30_fuse_readl, tegra30_fuse_program, &sku_info))
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

