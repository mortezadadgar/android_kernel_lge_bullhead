/*
 * drivers/powergate/tegra-powergate.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/clk/tegra.h>
#include <linux/tegra-powergate.h>
#include <linux/tegra-soc.h>

#include "iomap.h"

#define DPD_SAMPLE		0x020
#define  DPD_SAMPLE_ENABLE	(1 << 0)
#define  DPD_SAMPLE_DISABLE	(0 << 0)

#define PWRGATE_CLAMP_STATUS	0x2c

#define PWRGATE_TOGGLE		0x30
#define  PWRGATE_TOGGLE_START	(1 << 8)

#define REMOVE_CLAMPING		0x34

#define PWRGATE_STATUS		0x38

/* Timeout for powergate toggle operation if it takes affect */
#define PWRGATE_TOGGLE_TIMEOUT		10
/* Timeout for PMC to complete other requests before this */
#define PWRGATE_CONTENTION_TIMEOUT	100

#define IO_DPD_REQ		0x1b8
#define  IO_DPD_REQ_CODE_IDLE	(0 << 30)
#define  IO_DPD_REQ_CODE_OFF	(1 << 30)
#define  IO_DPD_REQ_CODE_ON	(2 << 30)
#define  IO_DPD_REQ_CODE_MASK	(3 << 30)

#define IO_DPD_STATUS		0x1bc
#define IO_DPD2_REQ		0x1c0
#define IO_DPD2_STATUS		0x1c4
#define SEL_DPD_TIM		0x1c8

#define GPU_RG_CNTRL		0x2d4

static int tegra_num_powerdomains;
static int tegra_num_cpu_domains;
static const u8 *tegra_cpu_domains;
static struct powergate *powergate;

static DEFINE_SPINLOCK(tegra_powergate_lock);

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static u32 pmc_read(unsigned long reg)
{
	return readl(pmc + reg);
}

static void pmc_write(u32 val, unsigned long reg)
{
	writel(val, pmc + reg);
}

static int tegra_powergate_set(int id, bool new_state)
{
	bool status;
	unsigned long flags;
	/*
	 * (TOGGLE_TIMEOUT * CONTENTION_TIMEOUT) timeout in microsecond for
	 * toggle command to take affect in case of contention with h/w
	 * initiated CPU power gating.
	 */
	int timeout = PWRGATE_CONTENTION_TIMEOUT * PWRGATE_TOGGLE_TIMEOUT;

	spin_lock_irqsave(&tegra_powergate_lock, flags);

	status = pmc_read(PWRGATE_STATUS) & (1 << id);

	if (status == new_state) {
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return 0;
	}

	pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);

	/* Check power gate status */
	do {
		udelay(1);
		status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

		timeout--;
	} while ((status != new_state) && (timeout > 0));

	spin_unlock_irqrestore(&tegra_powergate_lock, flags);

	if (status != new_state) {
		WARN(1, "Could not set powergate %d to %d",
				id, new_state);
		return -EBUSY;
	}

	return 0;
}

int tegra_powergate_power_on(int id)
{
	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	return tegra_powergate_set(id, true);
}

int tegra_powergate_power_off(int id)
{
	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	return tegra_powergate_set(id, false);
}

int tegra_powergate_is_powered(int id)
{
	u32 status;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	status = pmc_read(PWRGATE_STATUS) & (1 << id);
	return !!status;
}

int tegra_powergate_remove_clamping(int id)
{
	u32 mask;
	int contention_timeout = PWRGATE_CONTENTION_TIMEOUT;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	/*
	 * Tegra 2 has a bug where PCIE and VDE clamping masks are
	 * swapped relatively to the partition ids
	 */
	if (id ==  TEGRA_POWERGATE_VDEC)
		mask = (1 << TEGRA_POWERGATE_PCIE);
	else if	(id == TEGRA_POWERGATE_PCIE)
		mask = (1 << TEGRA_POWERGATE_VDEC);
	else
		mask = (1 << id);

	pmc_write(mask, REMOVE_CLAMPING);

	/* Wait until clamp is removed */
	do {
		udelay(1);
		contention_timeout--;
	} while ((contention_timeout > 0)
			&& (pmc_read(PWRGATE_CLAMP_STATUS) & BIT(id)));

	if (pmc_read(PWRGATE_CLAMP_STATUS) & BIT(id)) {
		WARN(1, "Couldn't remove clamping");
		return -EBUSY;
	}

	return 0;
}

/* Must be called with clk disabled, and returns with clk enabled */
int tegra_powergate_sequence_power_up(int id, struct clk *clk)
{
	int ret;

	tegra_periph_reset_assert(clk);

	ret = tegra_powergate_power_on(id);
	if (ret)
		goto err_power;

	ret = clk_prepare_enable(clk);
	if (ret)
		goto err_clk;

	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	udelay(10);
	tegra_periph_reset_deassert(clk);

	return 0;

err_clamp:
	clk_disable_unprepare(clk);
err_clk:
	tegra_powergate_power_off(id);
err_power:
	return ret;
}
EXPORT_SYMBOL(tegra_powergate_sequence_power_up);

int tegra_powergate_partition(int id)
{
	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	return powergate->ops->powergate_partition(id);
}
EXPORT_SYMBOL(tegra_powergate_partition);

int tegra_unpowergate_partition(int id)
{
	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	return powergate->ops->unpowergate_partition(id);
}
EXPORT_SYMBOL(tegra_unpowergate_partition);

int tegra_cpu_powergate_id(int cpuid)
{
	if (cpuid > 0 && cpuid < tegra_num_cpu_domains)
		return tegra_cpu_domains[cpuid];

	return -EINVAL;
}

int __init tegra_powergate_init(void)
{
	switch (tegra_chip_id) {
	case TEGRA20:
		powergate = tegra20_powergate_init();
		break;
	case TEGRA30:
		powergate = tegra30_powergate_init();
		break;
	case TEGRA114:
		powergate = tegra114_powergate_init();
		break;
	case TEGRA124:
		powergate = tegra124_powergate_init();
		break;
	default:
		/* Unknown Tegra variant. Disable powergating */
		tegra_num_powerdomains = 0;
		break;
	}

	if (powergate) {
		tegra_num_powerdomains = powergate->num_powerdomains;
		tegra_num_cpu_domains = powergate->num_cpu_domains;
		tegra_cpu_domains = powergate->cpu_domain_map;
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static int powergate_show(struct seq_file *s, void *data)
{
	int i;
	const char *name;

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	for (i = 0; i < tegra_num_powerdomains; i++) {
		name = powergate->ops->get_powergate_domain_name(i);
		if (!name)
			continue;

		seq_printf(s, " %9s %7s\n", name,
			tegra_powergate_is_powered(i) ? "yes" : "no");
	}

	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open		= powergate_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int __init tegra_powergate_debugfs_init(void)
{
	struct dentry *d;

	if (powergate && powergate->ops &&
		powergate->ops->get_powergate_domain_name) {
		d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
			&powergate_fops);
		if (!d)
			return -ENOMEM;
	}

	return 0;
}

#endif

static int tegra_io_rail_prepare(int id, unsigned long *request,
				 unsigned long *status, unsigned int *bit)
{
	unsigned long rate, value;
	struct clk *clk;

	*bit = id % 32;

	/*
	 * There are two sets of 30 bits to select IO rails, but bits 30 and
	 * 31 are control bits rather than IO rail selection bits.
	 */
	if (id > 63 || *bit == 30 || *bit == 31)
		return -EINVAL;

	if (id < 32) {
		*status = IO_DPD_STATUS;
		*request = IO_DPD_REQ;
	} else {
		*status = IO_DPD2_STATUS;
		*request = IO_DPD2_REQ;
	}

	clk = clk_get_sys(NULL, "pclk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	rate = clk_get_rate(clk);
	clk_put(clk);

	pmc_write(DPD_SAMPLE_ENABLE, DPD_SAMPLE);

	/* must be at least 200 ns, in APB (PCLK) clock cycles */
	value = DIV_ROUND_UP(1000000000, rate);
	value = DIV_ROUND_UP(200, value);
	pmc_write(value, SEL_DPD_TIM);

	return 0;
}

static int tegra_io_rail_poll(unsigned long offset, unsigned long mask,
			      unsigned long val, unsigned long timeout)
{
	unsigned long value;

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_after(timeout, jiffies)) {
		value = pmc_read(offset);
		if ((value & mask) == val)
			return 0;

		usleep_range(250, 1000);
	}

	return -ETIMEDOUT;
}

static void tegra_io_rail_unprepare(void)
{
	pmc_write(DPD_SAMPLE_DISABLE, DPD_SAMPLE);
}

int tegra_io_rail_power_on(int id)
{
	unsigned long request, status, value;
	unsigned int bit, mask;
	int err;

	err = tegra_io_rail_prepare(id, &request, &status, &bit);
	if (err < 0)
		return err;

	mask = 1 << bit;

	value = pmc_read(request);
	value |= mask;
	value &= ~IO_DPD_REQ_CODE_MASK;
	value |= IO_DPD_REQ_CODE_OFF;
	pmc_write(value, request);

	err = tegra_io_rail_poll(status, mask, 0, 250);
	if (err < 0)
		return err;

	tegra_io_rail_unprepare();

	return 0;
}

int tegra_io_rail_power_off(int id)
{
	unsigned long request, status, value;
	unsigned int bit, mask;
	int err;

	err = tegra_io_rail_prepare(id, &request, &status, &bit);
	if (err < 0)
		return err;

	mask = 1 << bit;

	value = pmc_read(request);
	value |= mask;
	value &= ~IO_DPD_REQ_CODE_MASK;
	value |= IO_DPD_REQ_CODE_ON;
	pmc_write(value, request);

	err = tegra_io_rail_poll(status, mask, mask, 250);
	if (err < 0)
		return err;

	tegra_io_rail_unprepare();

	return 0;
}
