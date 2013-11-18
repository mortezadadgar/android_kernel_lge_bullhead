/*
 * Copyright (c) 2011-2013, NVIDIA CORPORATION. All rights reserved.
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
 */

#include <linux/cpufreq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/tegra-soc.h>

#include "tegra_edp_private.h"

static const int tegra124_leakage_ijk_common[4][4][4] = {
	/* i = 0 */
	{ {  -309609464,    197786326,    -40763150,    1613941, },
	  {   964716269,   -569081375,    115781607,   -4206296, },
	  {  -994324790,    529664031,   -106360108,    3454033, },
	  {   343209442,   -160577505,     31928605,    -895157, },
	},
	/* i = 1 */
	{ {   616319664,   -637007187,    137759592,    -7194133,  },
	  { -1853817283,   1896032851,   -407407611,    20868220,  },
	  {  1824097131,  -1831611624,    390753403,   -19530122,  },
	  {  -589155245,    578838526,   -122655676,     5985577,  },
	},
	/* i = 2 */
	{ {  -439994037,    455845250,   -104097013,     6191899, },
	  {  1354650774,  -1395561938,    318665647,   -18886906, },
	  { -1361677255,   1390149678,   -317474532,    18728266, },
	  {   447877887,   -451382027,    103201434,    -6046692, },
	},
	/* i = 3 */
	{ {    56797556,    -59779544,     13810295,     -848290, },
	  {  -175867301,    184753957,    -42708242,     2621537, },
	  {   177626357,   -185996541,     43029384,    -2638283, },
	  {   -58587547,     61075322,    -14145853,      865351, },
	},
};

#define TEGRA124_EDP_PARAMS_COMMON_PART					\
	.temp_scaled      = 10,						\
	.dyn_scaled       = 1000,					\
	.dyn_consts_n     = { 950,  1399, 2166, 3041 },	\
	.consts_scaled    = 100,					\
	.leakage_consts_n = { 45, 67, 87, 100 },			\
	.ijk_scaled       = 100000,					\
	.leakage_min      = 30,						\
	.volt_temp_cap = { 70, 1240 },					\
	.leakage_consts_ijk = tegra124_leakage_ijk_common

static const struct tegra_edp_cpu_leakage_params tegra124_leakage_params[] = {
	{
		.cpu_speedo_id      = 0, /* Engg SKU */
		TEGRA124_EDP_PARAMS_COMMON_PART,
	},
	{
		.cpu_speedo_id      = 1, /* Prod SKU */
		TEGRA124_EDP_PARAMS_COMMON_PART,
	},
	{
		.cpu_speedo_id      = 2, /* Prod SKU */
		TEGRA124_EDP_PARAMS_COMMON_PART,
	},
	{
		.cpu_speedo_id      = 3, /* Prod SKU */
		TEGRA124_EDP_PARAMS_COMMON_PART,
	},
};

static unsigned int tegra124_edp_get_max_cpu_freq(void)
{
	struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(0);
	unsigned int freq = 0;
	int i;

	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;

		if (table[i].frequency > freq)
			freq = table[i].frequency;
	}

	return freq;
}

int tegra124_cpu_edp_init(struct device_node *cpu_edp_dn)
{
	struct platform_device *pdev;
	struct tegra_edp *edp;

	pdev = of_find_device_by_node(cpu_edp_dn);
	if (!pdev) {
		dev_err(&pdev->dev, "Failed to get cpu edp device\n");
		return -EINVAL;
	}

	edp = dev_get_drvdata(&pdev->dev);

	edp->params = tegra124_leakage_params;
	edp->params_size = ARRAY_SIZE(tegra124_leakage_params);

	edp->max_cpu_freq = tegra124_edp_get_max_cpu_freq();

	return 0;
}
