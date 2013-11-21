/*
 * Copyright (c) 2012, 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>

#include "clk_pllg.h"

#define MHZ (1000 * 1000)

#define MASK(w)	((1 << w) - 1)

#define SYS_GPCPLL_CFG_BASE			0x00137000
#define GPC_BCASE_GPCPLL_CFG_BASE	0x00132800

#define GPCPLL_CFG		(SYS_GPCPLL_CFG_BASE + 0)
#define GPCPLL_CFG_ENABLE	BIT(0)
#define GPCPLL_CFG_IDDQ		BIT(1)
#define GPCPLL_CFG_LOCK_DET_OFF	BIT(4)
#define GPCPLL_CFG_LOCK		BIT(17)

#define GPCPLL_COEFF		(SYS_GPCPLL_CFG_BASE + 4)
#define GPCPLL_COEFF_M_SHIFT	0
#define GPCPLL_COEFF_N_SHIFT	8
#define GPCPLL_COEFF_P_SHIFT	16

#define GPCPLL_CFG2			(SYS_GPCPLL_CFG_BASE + 0xc)
#define GPCPLL_CFG2_SETUP2_SHIFT	16
#define GPCPLL_CFG2_PLL_STEPA_SHIFT	24

#define GPCPLL_CFG3			(SYS_GPCPLL_CFG_BASE + 0x18)
#define GPCPLL_CFG3_PLL_STEPB_SHIFT		16

#define GPCPLL_NDIV_SLOWDOWN	(SYS_GPCPLL_CFG_BASE + 0x1c)
#define GPCPLL_NDIV_SLOWDOWN_NDIV_LO_SHIFT	0
#define GPCPLL_NDIV_SLOWDOWN_NDIV_MID_SHIFT	8
#define GPCPLL_NDIV_SLOWDOWN_STEP_SIZE_LO2MID_SHIFT	16
#define GPCPLL_NDIV_SLOWDOWN_SLOWDOWN_USING_PLL_SHIFT	22
#define GPCPLL_NDIV_SLOWDOWN_EN_DYNRAMP_SHIFT	31

#define SEL_VCO		(SYS_GPCPLL_CFG_BASE + 0x100)
#define SEL_VCO_GPC2CLK_OUT_SHIFT	0

#define GPC2CLK_OUT	(SYS_GPCPLL_CFG_BASE + 0x250)
#define GPC2CLK_OUT_SDIV14_INDIV4_WIDTH 1
#define GPC2CLK_OUT_SDIV14_INDIV4_SHIFT 31
#define GPC2CLK_OUT_SDIV14_INDIV4_MODE 1
#define GPC2CLK_OUT_VCODIV_WIDTH 6
#define GPC2CLK_OUT_VCODIV_SHIFT 8
#define GPC2CLK_OUT_VCODIV1 0
#define GPC2CLK_OUT_VCODIV_MASK (MASK(GPC2CLK_OUT_VCODIV_WIDTH) << \
		GPC2CLK_OUT_VCODIV_SHIFT)
#define	GPC2CLK_OUT_BYPDIV_WIDTH 6
#define GPC2CLK_OUT_BYPDIV_SHIFT 0
#define GPC2CLK_OUT_BYPDIV31 0x3c
#define GPC2CLK_OUT_INIT_MASK	((MASK(GPC2CLK_OUT_SDIV14_INDIV4_WIDTH) << \
		GPC2CLK_OUT_SDIV14_INDIV4_SHIFT)\
		| (MASK(GPC2CLK_OUT_VCODIV_WIDTH) << GPC2CLK_OUT_VCODIV_SHIFT)\
		| (MASK(GPC2CLK_OUT_BYPDIV_WIDTH) << GPC2CLK_OUT_BYPDIV_SHIFT))
#define GPC2CLK_OUT_INIT_VAL	((GPC2CLK_OUT_SDIV14_INDIV4_MODE << \
		GPC2CLK_OUT_SDIV14_INDIV4_SHIFT) \
		| (GPC2CLK_OUT_VCODIV1 << GPC2CLK_OUT_VCODIV_SHIFT) \
		| (GPC2CLK_OUT_BYPDIV31 << GPC2CLK_OUT_BYPDIV_SHIFT))

#define GPC_BCAST_NDIV_SLOWDOWN_DEBUG	(GPC_BCASE_GPCPLL_CFG_BASE + 0xa0)
#define GPC_BCAST_NDIV_SLOWDOWN_DEBUG_PLL_DYNRAMP_DONE_SYNCED_SHIFT	24
#define GPC_BCAST_NDIV_SLOWDOWN_DEBUG_PLL_DYNRAMP_DONE_SYNCED_MASK \
		(0x1 << GPC_BCAST_NDIV_SLOWDOWN_DEBUG_PLL_DYNRAMP_DONE_SYNCED_SHIFT)

#define pllg_readl(offset, p) readl_relaxed(p->clk_base + offset)
#define pllg_writel(val, offset, p) writel_relaxed(val, p->clk_base + offset)

#define to_clk_pllg(_hw) container_of(_hw, struct tegra_pllg, hw)

static u8 pl_to_div[] = {
/* PL:   0, 1, 2, 3, 4, 5, 6,  7,  8,  9, 10, 11, 12, 13, 14 */
/* p: */ 1, 2, 3, 4, 5, 6, 8, 10, 12, 16, 12, 16, 20, 24, 32,
};

struct tegra_pllg_mnp {
	u32	m, n, pl;
};

struct tegra_pllg {
	struct clk_hw			hw;
	void __iomem			*clk_base;
	spinlock_t			*lock;
	struct tegra_clk_pllg_params	*params;
	struct tegra_pllg_mnp		coef;
	unsigned long			parent_rate;
	bool				enabled;
	bool				init;
};

static unsigned long _calc_rate(unsigned long parent_rate,
			struct tegra_pllg_mnp *coef)
{
	unsigned long rate;
	int divider;

	rate = parent_rate * coef->n;
	divider = coef->m * pl_to_div[coef->pl];

	do_div(rate, divider);

	return rate;
}

static int _pllg_calc_mnp(struct tegra_pllg *pll,  unsigned long rate,
			unsigned long parent_rate, struct tegra_pllg_mnp *coef)
{
	unsigned int min_vco_f, max_vco_f;
	unsigned int target_vco_f, vco_f;
	unsigned int low_pl, high_pl, best_pl;
	unsigned int u_f;
	unsigned int target_clk_f, ref_clk_f, target_freq;
	u32 best_m, best_n;
	u32 m, n, n2;
	u32 delta, lwv, best_delta = ~0;
	int pl;

	target_clk_f = (rate * 2) / MHZ;
	ref_clk_f = parent_rate / MHZ;

	pr_debug("%s:%d %u %u\n", __FILE__, __LINE__, target_clk_f, ref_clk_f);

	max_vco_f = pll->params->max_vco;
	min_vco_f = pll->params->min_vco;
	best_m = pll->params->max_m;
	best_n = pll->params->min_n;
	best_pl = pll->params->min_pl;

	target_vco_f = target_clk_f + target_clk_f / 50;
	if (max_vco_f < target_vco_f)
		max_vco_f = target_vco_f;

	high_pl = (max_vco_f + target_vco_f - 1) / target_vco_f;
	high_pl = min(high_pl, pll->params->max_pl);
	high_pl = max(high_pl, pll->params->min_pl);

	low_pl = min_vco_f / target_vco_f;
	low_pl = min(low_pl, pll->params->max_pl);
	low_pl = max(low_pl, pll->params->min_pl);

	/* Find Indices of high_pl and low_pl */
	for (pl = 0; pl < 14; pl++) {
		if (pl_to_div[pl] >= low_pl) {
			low_pl = pl;
			break;
		}
	}
	for (pl = 0; pl < 14; pl++) {
		if (pl_to_div[pl] >= high_pl) {
			high_pl = pl;
			break;
		}
	}

	/* Select lowest possible VCO */
	for (pl = low_pl; pl <= high_pl; pl++) {
		target_vco_f = target_clk_f * pl_to_div[pl];

		for (m = pll->params->min_m; m <= pll->params->max_m; m++) {
			u_f = ref_clk_f / m;

			if (u_f < pll->params->min_u)
				break;
			if (u_f > pll->params->max_u)
				continue;

			n = (target_vco_f * m) / ref_clk_f;
			n2 = ((target_vco_f * m) + (ref_clk_f - 1)) / ref_clk_f;

			if (n > pll->params->max_n)
				break;

			for (; n <= n2; n++) {
				if (n < pll->params->min_n)
					continue;
				if (n > pll->params->max_n)
					break;

				vco_f = ref_clk_f * n / m;

				if (vco_f >= min_vco_f && vco_f <= max_vco_f) {
					lwv = (vco_f + (pl_to_div[pl] / 2))
						/ pl_to_div[pl];
					delta = abs(lwv - target_clk_f);

					if (delta < best_delta) {
						best_delta = delta;
						best_m = m;
						best_n = n;
						best_pl = pl;

						if (best_delta == 0)
							goto found_match;
					}
				}
			}
		}
	}

found_match:
	WARN_ON(best_delta == ~0);

	if (best_delta != 0)
		pr_debug("no best match for target @ %dMHz on gpc_pll",
			target_clk_f);

	coef->m = best_m;
	coef->n = best_n;
	coef->pl = best_pl;

	target_freq = _calc_rate(parent_rate, coef) / MHZ;

	pr_debug("actual target freq %d MHz, M %d, N %d, PL %d(div%d)\n",
		target_freq, coef->m, coef->n, coef->pl, pl_to_div[coef->pl]);

	return 0;
}

static int _pllg_slide(struct tegra_pllg *pllg, u32 n)
{
	u32 val;
	int ramp_timeout;

	/* get old coefficients */
	val = pllg_readl(GPCPLL_COEFF, pllg);
	/* do nothing if NDIV is same */
	if (n == ((val >> GPCPLL_COEFF_N_SHIFT) & 0xff))
		return 0;

	/* setup */
	val = pllg_readl(GPCPLL_CFG2, pllg);
	val &= ~(0xff << GPCPLL_CFG2_PLL_STEPA_SHIFT);
	val |= 0x2b << GPCPLL_CFG2_PLL_STEPA_SHIFT;
	pllg_writel(val, GPCPLL_CFG2, pllg);
	val = pllg_readl(GPCPLL_CFG3, pllg);
	val &= ~(0xff << GPCPLL_CFG3_PLL_STEPB_SHIFT);
	val |= 0xb << GPCPLL_CFG3_PLL_STEPB_SHIFT;
	pllg_writel(val, GPCPLL_CFG3, pllg);

	/* pll slowdown mode */
	val = pllg_readl(GPCPLL_NDIV_SLOWDOWN, pllg);
	val |= 0x1 << GPCPLL_NDIV_SLOWDOWN_SLOWDOWN_USING_PLL_SHIFT;
	pllg_writel(val, GPCPLL_NDIV_SLOWDOWN, pllg);

	/* new ndiv ready for ramp */
	val = pllg_readl(GPCPLL_COEFF, pllg);
	val &= ~(0xff << GPCPLL_COEFF_N_SHIFT);
	val |= (n & 0xff) << GPCPLL_COEFF_N_SHIFT;
	udelay(1);
	pllg_writel(val, GPCPLL_COEFF, pllg);

	/* dynamic ramp to new ndiv */
	val = pllg_readl(GPCPLL_NDIV_SLOWDOWN, pllg);
	val |= 0x1 << GPCPLL_NDIV_SLOWDOWN_EN_DYNRAMP_SHIFT;
	udelay(1);
	pllg_writel(val, GPCPLL_NDIV_SLOWDOWN, pllg);

	for (ramp_timeout = 500; ramp_timeout; ramp_timeout--) {
		udelay(1);
		val = pllg_readl(GPC_BCAST_NDIV_SLOWDOWN_DEBUG, pllg);
		if (val & GPC_BCAST_NDIV_SLOWDOWN_DEBUG_PLL_DYNRAMP_DONE_SYNCED_MASK)
			break;
	}

	/* exit slowdown mode */
	val = pllg_readl(GPCPLL_NDIV_SLOWDOWN, pllg);
	val &= ~(0x1 << GPCPLL_NDIV_SLOWDOWN_SLOWDOWN_USING_PLL_SHIFT);
	val &= ~(0x1 << GPCPLL_NDIV_SLOWDOWN_EN_DYNRAMP_SHIFT);
	pllg_writel(val, GPCPLL_NDIV_SLOWDOWN, pllg);
	pllg_readl(GPCPLL_NDIV_SLOWDOWN, pllg);

	if (ramp_timeout <= 0) {
		pr_err("gpcpll dynamic ramp timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void __pllg_disable(struct tegra_pllg *pllg)
{
	u32 val;

	/* disable PLL */
	val = pllg_readl(GPCPLL_CFG, pllg);
	val &= ~GPCPLL_CFG_ENABLE;
	pllg_writel(val, GPCPLL_CFG, pllg);
	pllg_readl(GPCPLL_CFG, pllg);
}

static void _pllg_disable(struct tegra_pllg *pllg)
{
	u32 val;

	/* slide to VCO min */
	val = pllg_readl(GPCPLL_CFG, pllg);
	if (val & GPCPLL_CFG_ENABLE) {
		u32 coeff, m, n_lo;

		coeff = pllg_readl(GPCPLL_COEFF, pllg);
		m = (coeff >> GPCPLL_COEFF_M_SHIFT) & 0xFF;
		n_lo = DIV_ROUND_UP(m * pllg->params->min_vco,
				pllg->parent_rate / MHZ);
		_pllg_slide(pllg, n_lo);
	}

	/* put PLL in bypass before disabling it */
	val = pllg_readl(SEL_VCO, pllg);
	val &= ~(0x1 << SEL_VCO_GPC2CLK_OUT_SHIFT);
	pllg_writel(val, SEL_VCO, pllg);

	__pllg_disable(pllg);
}

static int __program_pll_mnp(struct tegra_pllg *pllg, int allow_slide)
{
	u32 val, cfg;
	int delay;
	u32 m_old, pl_old, n_lo;

	/* get old coefficients */
	val = pllg_readl(GPCPLL_COEFF, pllg);
	m_old = (val >> GPCPLL_COEFF_M_SHIFT) & 0xFF;
	pl_old = (val >> GPCPLL_COEFF_P_SHIFT) & 0xFF;

	/* do NDIV slide if there is no change in M and PL */
	cfg = pllg_readl(GPCPLL_CFG, pllg);
	if (allow_slide && pllg->coef.m == m_old && pllg->coef.pl == pl_old &&
		(cfg & GPCPLL_CFG_ENABLE))
		return _pllg_slide(pllg, pllg->coef.n);

	/* slide down to NDIV_LO */
	n_lo = DIV_ROUND_UP(m_old * pllg->params->min_vco,
			pllg->parent_rate / MHZ);
	if (allow_slide && (cfg & GPCPLL_CFG_ENABLE)) {
		int ret = _pllg_slide(pllg, n_lo);
		if (ret)
			return ret;
	}

	/* split FO-to-bypass jump in halfs by setting out divider 1:2 */
	val = pllg_readl(GPC2CLK_OUT, pllg);
	val &= ~GPC2CLK_OUT_VCODIV_MASK;
	val |= 0x2 << GPC2CLK_OUT_VCODIV_SHIFT;
	pllg_writel(val, GPC2CLK_OUT, pllg);

	/* put PLL in bypass before programming it */
	val = pllg_readl(SEL_VCO, pllg);
	val &= ~(0x1 << SEL_VCO_GPC2CLK_OUT_SHIFT);
	udelay(2);
	pllg_writel(val, SEL_VCO, pllg);

	/* get out from IDDQ */
	val = pllg_readl(GPCPLL_CFG, pllg);
	if (val & GPCPLL_CFG_IDDQ) {
		val &= ~GPCPLL_CFG_IDDQ;
		pllg_writel(val, GPCPLL_CFG, pllg);
		pllg_readl(GPCPLL_CFG, pllg);
		udelay(2);
	}

	__pllg_disable(pllg);

	pr_debug("%s: m=%d n=%d pl=%d\n", __func__, pllg->coef.m,
					pllg->coef.n, pllg->coef.pl);

	n_lo = DIV_ROUND_UP(pllg->coef.m * pllg->params->min_vco,
			pllg->parent_rate / MHZ);
	val = pllg->coef.m << GPCPLL_COEFF_M_SHIFT;
	val |= (allow_slide ? n_lo : pllg->coef.n) << GPCPLL_COEFF_N_SHIFT;
	val |= pllg->coef.pl << GPCPLL_COEFF_P_SHIFT;
	pllg_writel(val, GPCPLL_COEFF, pllg);

	/* enable PLL */
	val = pllg_readl(GPCPLL_CFG, pllg);
	val |= GPCPLL_CFG_ENABLE;
	pllg_writel(val, GPCPLL_CFG, pllg);

	val = pllg_readl(GPCPLL_CFG, pllg);
	if (val & GPCPLL_CFG_LOCK_DET_OFF) {
		val &= ~GPCPLL_CFG_LOCK_DET_OFF;
		pllg_writel(val, GPCPLL_CFG, pllg);
	}

	for (delay = 0; delay < 150; delay++) {
		udelay(2);
		val = pllg_readl(GPCPLL_CFG, pllg);
		if (val & GPCPLL_CFG_LOCK)
			goto pll_locked;
	}

	pr_err("%s: Timed out waiting for pll %s lock\n", __func__,
		__clk_get_name(pllg->hw.clk));

	return -ETIMEDOUT;

pll_locked:
	/* switch to VCO mode */
	val = pllg_readl(SEL_VCO, pllg);
	val |= 0x1 << SEL_VCO_GPC2CLK_OUT_SHIFT;
	pllg_writel(val, SEL_VCO, pllg);

	/* restore out divider 1:1 */
	val = pllg_readl(GPC2CLK_OUT, pllg);
	val &= ~GPC2CLK_OUT_VCODIV_MASK;
	udelay(2);
	pllg_writel(val, GPC2CLK_OUT, pllg);

	/* slide up to new NDIV */
	return allow_slide ? _pllg_slide(pllg, pllg->coef.n) : 0;
}

static int _program_pll_mnp(struct tegra_pllg *pllg)
{
	int err;

	err = __program_pll_mnp(pllg, 1);
	if (err)
		err = __program_pll_mnp(pllg, 0);

	return err;
}

static void _init_pllg(struct tegra_pllg *pllg)
{
	u32 val;

	val = pllg_readl(GPC2CLK_OUT, pllg);
	val &= ~GPC2CLK_OUT_INIT_MASK;
	val |= GPC2CLK_OUT_INIT_VAL;
	pllg_writel(val, GPC2CLK_OUT, pllg);

	pllg->init = true;
}

static int clk_pllg_is_enabled(struct clk_hw *hw)
{
	struct tegra_pllg *pllg = to_clk_pllg(hw);

	return pllg->enabled;
}

static int clk_pllg_enable(struct clk_hw *hw)
{
	struct tegra_pllg *pllg = to_clk_pllg(hw);
	int err;

	if (!pllg->init)
		_init_pllg(pllg);

	err = _program_pll_mnp(pllg);
	if (!err)
		pllg->enabled = true;

	return err;
}

static void clk_pllg_disable(struct clk_hw *hw)
{
	struct tegra_pllg *pllg = to_clk_pllg(hw);

	_pllg_disable(pllg);

	pllg->enabled = false;
}

static unsigned long clk_pllg_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct tegra_pllg *pllg = to_clk_pllg(hw);

	return _calc_rate(parent_rate, &pllg->coef) / 2;
}

static long clk_pllg_round_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long *parent_rate)
{
	struct tegra_pllg *pllg = to_clk_pllg(hw);
	struct tegra_pllg_mnp coef;
	unsigned long min_freq = pllg->params->min_freq / 2 * MHZ;
	unsigned long max_freq = pllg->params->max_freq / 2 * MHZ;

	if (rate < min_freq)
		rate = min_freq;
	else if (rate > max_freq)
		rate = max_freq;

	_pllg_calc_mnp(pllg, rate, *parent_rate, &coef);

	return _calc_rate(*parent_rate, &coef) / 2;
}

static int clk_pllg_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	struct tegra_pllg *pllg = to_clk_pllg(hw);
	struct tegra_pllg_mnp coef;

	_pllg_calc_mnp(pllg, rate, parent_rate, &coef);
	pllg->coef = coef;
	pllg->parent_rate = parent_rate;

	if (pllg->enabled)
		return _program_pll_mnp(pllg);

	return 0;
}

const struct clk_ops tegra_pllg_ops = {
	.is_enabled = clk_pllg_is_enabled,
	.enable = clk_pllg_enable,
	.disable = clk_pllg_disable,
	.recalc_rate = clk_pllg_recalc_rate,
	.round_rate = clk_pllg_round_rate,
	.set_rate = clk_pllg_set_rate,
};

struct clk *tegra_clk_register_pllg(const char *name, const char *parent_name,
				void __iomem *clk_base, unsigned long flags,
				struct tegra_clk_pllg_params *params,
				spinlock_t *lock)
{
	struct clk_init_data init;
	struct tegra_pllg *pllg;
	struct tegra_pllg_mnp coef;
	struct clk *parent;
	unsigned long parent_rate;

	parent = __clk_lookup(parent_name);
	if (IS_ERR(parent)) {
		WARN(1, "parent clk %s of %s must be registered first\n",
			name, parent_name);
		return ERR_PTR(-EINVAL);
	}

	parent_rate = __clk_get_rate(parent);

	pllg = kzalloc(sizeof(*pllg), GFP_KERNEL);
	if (!pllg)
		return ERR_PTR(-ENOMEM);

	pllg->clk_base = clk_base;
	pllg->params = params;
	pllg->lock = lock;

	_pllg_calc_mnp(pllg, 72 * MHZ, parent_rate, &coef);
	pllg->coef = coef;
	pllg->parent_rate = parent_rate;

	init.name = name;
	init.ops = &tegra_pllg_ops;
	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	pllg->hw.init = &init;

	return clk_register(NULL, &pllg->hw);
}
