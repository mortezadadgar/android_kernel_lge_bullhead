/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_data/tegra_mc.h>
#include <linux/tegra-powergate.h>
#include <linux/notifier.h>

#include "iomap.h"

#define TEGRA124_POWERGATE_NUM (TEGRA_POWERGATE_VIC + 1)

#define MAX_HOTRESET_CLIENT_NUM 4

enum mc_client {
	MC_CLIENT_AFI		= 0,
	MC_CLIENT_DC		= 2,
	MC_CLIENT_DCB		= 3,
	MC_CLIENT_ISP		= 8,
	MC_CLIENT_MSENC		= 11,
	MC_CLIENT_SATA		= 15,
	MC_CLIENT_VDE		= 16,
	MC_CLIENT_VI		= 17,
	MC_CLIENT_VIC		= 18,
	MC_CLIENT_XUSB_HOST	= 19,
	MC_CLIENT_XUSB_DEV	= 20,
	MC_CLIENT_ISPB		= 33,
	MC_CLIENT_GPU		= 34,
	MC_CLIENT_LAST		= -1,
};

struct mc_client_info {
	enum mc_client hot_reset_clients[MAX_HOTRESET_CLIENT_NUM];
};

static int tegra124_powergate_partition(int id);
static int tegra124_unpowergate_partition(int id);

static struct mc_client_info tegra124_pg_mc_info[TEGRA124_POWERGATE_NUM] = {
	[TEGRA_POWERGATE_VDEC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_VDE,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_MPE] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_MSENC,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_VENC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_ISP,
			[1] = MC_CLIENT_ISPB,
			[2] = MC_CLIENT_VI,
			[3] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_DIS] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_DC,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_DISB] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_DCB,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_XUSBA] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_XUSBB] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_XUSB_DEV,
			[1] = MC_CLIENT_LAST
		},
	},
	[TEGRA_POWERGATE_XUSBC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_XUSB_HOST,
			[1] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_SOR] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_LAST,
		},
	},
	[TEGRA_POWERGATE_VIC] = {
		.hot_reset_clients = {
			[0] = MC_CLIENT_VIC,
			[1] = MC_CLIENT_LAST,
		},
	},
};

static struct powergate_partition_info
tegra124_powergate_partition_info[TEGRA124_POWERGATE_NUM] = {
	[TEGRA_POWERGATE_VDEC] = {
		.name = "vde",
		.clk_info = {
			[0] = { .clk_name = "vde" },
		},
	},
	[TEGRA_POWERGATE_MPE] = {
		.name = "mpe",
		.clk_info = {
			[0] = { .clk_name = "msenc" },
		},
	},
	[TEGRA_POWERGATE_VENC] = {
		.name = "ve",
		.clk_info = {
			[0] = { .clk_name = "isp" },
			[1] = { .clk_name = "ispb" },
			[2] = { .clk_name = "vi"  },
			[3] = { .clk_name = "csi" },
		},
	},
	[TEGRA_POWERGATE_SOR] = {
		.name = "sor",
		.clk_info = {
			[0] = { .clk_name = "sor0" },
			[1] = { .clk_name = "dsia" },
			[2] = { .clk_name = "dsib" },
			[3] = { .clk_name = "hdmi" },
			[4] = { .clk_name = "mipi-cal" },
			[5] = { .clk_name = "dpaux" },
		},
	},
	[TEGRA_POWERGATE_DIS] = {
		.name = "disa",
		.clk_info = {
			[0] = { .clk_name = "disp1" },
		},
	},
	[TEGRA_POWERGATE_DISB] = {
		.name = "disb",
		.clk_info = {
			[0] = { .clk_name = "disp2" },
		},
	},
	[TEGRA_POWERGATE_XUSBA] = {
		.name = "xusba",
		.clk_info = {
			[0] = { .clk_name = "xusb_ss" },
		},
	},
	[TEGRA_POWERGATE_XUSBB] = {
		.name = "xusbb",
		.clk_info = {
			[0] = { .clk_name = "xusb_dev" },
		},
	},
	[TEGRA_POWERGATE_XUSBC] = {
		.name = "xusbc",
		.clk_info = {
			[0] = { .clk_name = "xusb_host" },
		},
	},
	[TEGRA_POWERGATE_VIC] = {
		.name = "vic",
		.clk_info = {
			[0] = { .clk_name = "vic03" },
		},
	},
};

static const char *tegra124_get_powerdomain_name(int id)
{
	return tegra124_powergate_partition_info[id].name;
}

static atomic_t ref_count_dispa = ATOMIC_INIT(0);
static atomic_t ref_count_dispb = ATOMIC_INIT(0);
static atomic_t ref_count_sor = ATOMIC_INIT(0);
static atomic_t ref_count_venc = ATOMIC_INIT(0);
static DEFINE_MUTEX(tegra124_powergate_lock);

static bool is_clk_inited;
static bool is_mc_ready;
static struct notifier_block nb;

static void release_clk(struct powergate_partition_info *pg_info, int last)
{
	struct partition_clk_info *clk_info;

	while (last--) {
		clk_info = &pg_info->clk_info[last];
		clk_put(clk_info->clk_ptr);
	}
}

static void tegra124_powergate_init_clk(void)
{
	int i, j;
	struct clk *clk;
	struct powergate_partition_info *pg_info;
	struct partition_clk_info *clk_info;

	for (i = 0; i < TEGRA124_POWERGATE_NUM; i++) {
		pg_info = &tegra124_powergate_partition_info[i];
		if (!pg_info->clk_info)
			break;

		for (j = 0; j < MAX_CLK_EN_NUM; j++) {
			clk_info = &pg_info->clk_info[j];
			if (!clk_info || !clk_info->clk_name)
				break;

			clk = clk_get(NULL, clk_info->clk_name);
			if (IS_ERR(clk)) {
				pr_err("Tegra124 powergate can't find the clk %s\n",
					clk_info->clk_name);
				release_clk(pg_info, j);
				break;
			}

			clk_info->clk_ptr = clk;
		}
	}

	is_clk_inited = true;
}

static int enable_clk(int id)
{
	int ret, i;
	struct clk *clk;
	struct powergate_partition_info *pg_info;
	struct partition_clk_info *clk_info;

	pg_info = &tegra124_powergate_partition_info[id];

	for (i = 0; i < MAX_CLK_EN_NUM; i++) {
		clk_info = &pg_info->clk_info[i];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		ret = clk_prepare_enable(clk);
		if (ret)
			goto err_clk_en;
	}

	return 0;

err_clk_en:
	WARN(1, "Could not enable clk %s, error %d",
			pg_info->clk_info[i].clk_name, ret);
	while (i--) {
		clk_info = &pg_info->clk_info[i];
		clk_disable_unprepare(clk_info->clk_ptr);
	}

	return ret;
}

static void reset_clk(int id, bool assert)
{
	int i;
	struct clk *clk;
	struct powergate_partition_info *pg_info;
	struct partition_clk_info *clk_info;

	pg_info = &tegra124_powergate_partition_info[id];

	for (i = 0; i < MAX_CLK_EN_NUM; i++) {
		clk_info = &pg_info->clk_info[i];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		if (assert)
			tegra_periph_reset_assert(clk);
		else
			tegra_periph_reset_deassert(clk);
	}
}

static void disable_clk(int id)
{
	int i;
	struct clk *clk;
	struct powergate_partition_info *pg_info;
	struct partition_clk_info *clk_info;

	pg_info = &tegra124_powergate_partition_info[id];

	for (i = 0; i < MAX_CLK_EN_NUM; i++) {
		clk_info = &pg_info->clk_info[i];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		clk_disable_unprepare(clk);
	}
}

static int reset_module(int id)
{
	int ret;

	reset_clk(id, true);
	udelay(10);

	ret = enable_clk(id);
	if (ret)
		return ret;
	udelay(10);

	reset_clk(id, false);

	disable_clk(id);

	return 0;
}

/*
 * MC related internal functions
 */

static int mc_flush(int id)
{
	u32 i;
	enum mc_client mc_client_bit;

	if (!is_mc_ready) {
		WARN(1, "Tegra124 memory controller is not ready\n");
		return -EPERM;
	}

	for (i = 0; i < MAX_HOTRESET_CLIENT_NUM; i++) {
		mc_client_bit =
			tegra124_pg_mc_info[id].hot_reset_clients[i];
		if (mc_client_bit == MC_CLIENT_LAST)
			break;
		tegra_mc_flush(mc_client_bit);
	}

	return 0;
}

static int mc_flush_done(int id)
{
	u32 i;
	enum mc_client mc_client_bit;

	if (!is_mc_ready) {
		WARN(1, "Tegra124 memory controller is not ready\n");
		return -EPERM;
	}

	for (i = 0; i < MAX_HOTRESET_CLIENT_NUM; i++) {
		mc_client_bit =
			tegra124_pg_mc_info[id].hot_reset_clients[i];
		if (mc_client_bit == MC_CLIENT_LAST)
			break;
		tegra_mc_flush_done(mc_client_bit);
	}

	return 0;
}

/**
 * do_powergate - power gating routine
 *
 * @id: power partition
 */
static int do_powergate(int id)
{
	int ret;

	ret = enable_clk(id);
	if (ret)
		WARN(1, "Couldn't enable clock");
	udelay(10);

	mc_flush(id);
	udelay(10);

	reset_clk(id, true);
	udelay(10);

	/* Powergating is done only if refcnt of all clks is 0 */
	disable_clk(id);
	udelay(10);

	ret = tegra_powergate_power_off(id);
	if (ret)
		goto err_power_off;

	return 0;

err_power_off:
	WARN(1, "Could not Powergate Partition %d", id);
	return ret;
}

/**
 * do_unpowergate - power ungating routine
 *
 * @id: power partition
 * @reset_needed: reset or not when the power is already on
 */
static int do_unpowergate(int id, int reset_needed)
{
	int ret;

	if (tegra_powergate_is_powered(id))
		return reset_needed ? reset_module(id) : 0;

	ret = tegra_powergate_power_on(id);
	if (ret)
		goto err_power;
	udelay(10);

	/* Un-Powergating fails if all clks are not enabled */
	ret = enable_clk(id);
	if (ret)
		goto err_clk_on;
	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;
	udelay(10);

	/* deassert reset */
	reset_clk(id, false);
	udelay(10);

	mc_flush_done(id);
	udelay(10);

	/* Disable all clks enabled earlier. Drivers should enable clks */
	disable_clk(id);

	return 0;

err_clamp:
	disable_clk(id);
err_clk_on:
	tegra_powergate_power_off(id);
err_power:
	WARN(1, "Could not Un-Powergate %d", id);
	return ret;
}

/**
 * do_group_powergate - solve the dependency between power domains
 *
 * @id: the power domain to be shut off
 *
 * "->" means "depends on"
 * VENC -> DISA
 * DISB -> DISA & SOR
 * DISA -> SOR
 */
static int do_group_powergate(int id)
{
	int ret;
	int counta = atomic_read(&ref_count_dispa);
	int countb = atomic_read(&ref_count_dispb);
	int countsor = atomic_read(&ref_count_sor);
	int countvenc = atomic_read(&ref_count_venc);

	switch (id) {
	case TEGRA_POWERGATE_DIS:
		counta = atomic_dec_return(&ref_count_dispa);
		countsor = atomic_dec_return(&ref_count_sor);
		break;
	case TEGRA_POWERGATE_DISB:
		countb = atomic_dec_return(&ref_count_dispb);
		counta = atomic_dec_return(&ref_count_dispa);
		countsor = atomic_dec_return(&ref_count_sor);
		break;
	case TEGRA_POWERGATE_VENC:
		countvenc = atomic_dec_return(&ref_count_venc);
		counta = atomic_dec_return(&ref_count_dispa);
		countsor = atomic_dec_return(&ref_count_sor);
		break;
	case TEGRA_POWERGATE_SOR:
		countsor = atomic_dec_return(&ref_count_sor);
		break;
	default:
		break;
	}

	WARN_ONCE(counta < 0, "DISPA ref count underflow");
	WARN_ONCE(countb < 0, "DISPB ref count underflow");
	WARN_ONCE(countsor < 0, "SOR ref count underflow");
	WARN_ONCE(countvenc < 0, "VENC ref count underflow");

	ret = 0;
	if (countvenc <= 0)
		ret = do_powergate(TEGRA_POWERGATE_VENC);
	if (countb <= 0 && !ret)
		ret = do_powergate(TEGRA_POWERGATE_DISB);
	if (counta <= 0 && !ret)
		ret = do_powergate(TEGRA_POWERGATE_DIS);
	if (countsor <= 0 && !ret)
		ret = do_powergate(TEGRA_POWERGATE_SOR);

	return ret;
}

/**
 * do_group_unpowergate - solve the dependency between power domains
 *
 * @id: the power domain to be powered on
 *
 * "->" means "depends on"
 * VENC -> DISA
 * DISB -> DISA & SOR
 * DISA -> SOR
 */
static int do_group_unpowergate(int id)
{
	int ret;
	int counta = atomic_read(&ref_count_dispa);
	int countb = atomic_read(&ref_count_dispb);
	int countsor = atomic_read(&ref_count_sor);
	int countvenc = atomic_read(&ref_count_venc);

	switch (id) {
	case TEGRA_POWERGATE_DIS:
		counta = atomic_inc_return(&ref_count_dispa);
		countsor = atomic_inc_return(&ref_count_sor);
		break;
	case TEGRA_POWERGATE_DISB:
		countb = atomic_inc_return(&ref_count_dispb);
		counta = atomic_inc_return(&ref_count_dispa);
		countsor = atomic_inc_return(&ref_count_sor);
		break;
	case TEGRA_POWERGATE_VENC:
		countvenc = atomic_inc_return(&ref_count_venc);
		counta = atomic_inc_return(&ref_count_dispa);
		countsor = atomic_inc_return(&ref_count_sor);
		break;
	case TEGRA_POWERGATE_SOR:
		countsor = atomic_inc_return(&ref_count_sor);
		break;
	default:
		break;
	}

	ret = 0;
	if (countsor > 0)
		ret = do_unpowergate(TEGRA_POWERGATE_SOR, 0);
	if (counta > 0 && !ret)
		ret = do_unpowergate(TEGRA_POWERGATE_DIS, 0);
	if (countb > 0 && !ret)
		ret = do_unpowergate(TEGRA_POWERGATE_DISB, 0);
	if (countvenc > 0 && !ret)
		ret = do_unpowergate(TEGRA_POWERGATE_VENC, 0);

	return ret;
}

static int tegra124_powergate_partition(int id)
{
	int ret;

	mutex_lock(&tegra124_powergate_lock);

	if (!is_clk_inited)
		tegra124_powergate_init_clk();

	switch (id) {
	case TEGRA_POWERGATE_DIS:
	case TEGRA_POWERGATE_DISB:
	case TEGRA_POWERGATE_VENC:
	/*
	 * SOR should not be powergated by someone outside the powergate code,
	 * but who knows?
	 */
	case TEGRA_POWERGATE_SOR:
		ret = do_group_powergate(id);
		break;
	default:
		ret = do_powergate(id);
		break;
	};

	mutex_unlock(&tegra124_powergate_lock);

	return ret;
}

static int tegra124_unpowergate_partition(int id)
{
	int ret;

	mutex_lock(&tegra124_powergate_lock);

	if (!is_clk_inited)
		tegra124_powergate_init_clk();

	switch (id) {
	case TEGRA_POWERGATE_DIS:
	case TEGRA_POWERGATE_DISB:
	case TEGRA_POWERGATE_VENC:
	/*
	 * SOR should not be powergated by someone outside the powergate code,
	 * but who knows?
	 */
	case TEGRA_POWERGATE_SOR:
		ret = do_group_unpowergate(id);
		break;
	default:
		ret = do_unpowergate(id, 1);
		break;
	};

	mutex_unlock(&tegra124_powergate_lock);

	return ret;
}

static struct powergate_ops tegra124_powergate_ops = {
	.get_powergate_domain_name = tegra124_get_powerdomain_name,
	.powergate_partition = tegra124_powergate_partition,
	.unpowergate_partition = tegra124_unpowergate_partition,
};

static struct powergate t124_powergate = {
	.soc_name = "tegra124",
	.num_powerdomains = TEGRA124_POWERGATE_NUM,
	.num_cpu_domains = 0,
	.cpu_domain_map = NULL,
	.ops = &tegra124_powergate_ops,
};

static int tegra124_powergate_clean(struct notifier_block *self,
				unsigned long event, void *data)
{
	is_mc_ready = true;

	if (!is_clk_inited)
		tegra124_powergate_init_clk();

	/* Powergate venc/dis/disb/sor to get a clean hardware environment. */
	WARN(do_powergate(TEGRA_POWERGATE_VENC), "Powergate VENC failed.");
	WARN(do_powergate(TEGRA_POWERGATE_DISB), "Powergate DISB failed.");
	WARN(do_powergate(TEGRA_POWERGATE_DIS), "Powergate DIS failed.");
	WARN(do_powergate(TEGRA_POWERGATE_SOR), "Powergate SOR failed.");

	return NOTIFY_DONE;
}

struct powergate * __init tegra124_powergate_init(void)
{
	memset(&nb, 0, sizeof(nb));
	nb.notifier_call = tegra124_powergate_clean;
	tegra124_mc_register_notify(&nb);

	return &t124_powergate;
}

