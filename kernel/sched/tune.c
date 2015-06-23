#include <linux/kernel.h>

#include "sched.h"
#include "tune.h"

unsigned int sysctl_sched_cfs_boost __read_mostly = 0;

/* Performance Boost region (B) threshold params */
static int perf_boost_idx;

/* Performance Constraint region (C) threshold params */
static int perf_constrain_idx;

/**
 * Performance-Energy (P-E) Space thresholds constants
 */
struct threshold_params {
	int nrg_gain;
	int cap_gain;
};

/*
 * System specific P-E space thresholds constants
 */
static struct threshold_params
threshold_gains[] = {
	{ 0, 4 }, /* >=  0% */
	{ 1, 4 }, /* >= 10% */
	{ 2, 4 }, /* >= 20% */
	{ 3, 4 }, /* >= 30% */
	{ 4, 4 }, /* >= 40% */
	{ 4, 3 }, /* >= 50% */
	{ 4, 2 }, /* >= 60% */
	{ 4, 1 }, /* >= 70% */
	{ 4, 0 }, /* >= 80% */
	{ 4, 0 }  /* >= 90% */
};

/*
 * System energy normalization constants
 */
struct target_nrg {
	unsigned long min_power;
	unsigned long max_power;
	unsigned long nrg_shift;
	unsigned long nrg_mult;
};

/*
 * Target specific system energy normalization constants
 * NOTE: These values are specific for ARM TC2 and they are derived from the
 *       energy model data defined in: arch/arm/kernel/topology.c
 */
static struct target_nrg
schedtune_target_nrg = {

	/*
	 * TC2 Min CPUs power:
	 * all CPUs idle, all clusters in deep idle:
	 *   0 * 3 + 0 * 2 + 10 + 25
	 */
	.min_power = 35,

	/*
	 * TC2 Max CPUs power:
	 * all CPUs fully utilized while running at max OPP:
	 *   1024 * 3 + 6997 * 2 + 4905 + 15200
	 */

	.max_power = 37171,

	/*
	 * Fast integer division by constant:
	 *  Constant   : Max - Min       (C) = 37171 - 35 = 37136
	 *  Precision  : 0.1%            (P) = 0.1
	 *  Reference  : C * 100 / P     (R) = 3713600
	 *
	 * Thus:
	 *  Shift bifs : ceil(log(R,2))  (S) = 26
	 *  Mult const : round(2^S/C)    (M) = 1807
	 *
	 * This allows to compute the normalized energy:
	 *   system_energy / C
	 * as:
	 *   (system_energy * M) >> S
	 */
	.nrg_shift = 26,	/* S */
	.nrg_mult  = 1807,	/* M */
};

/*
 * System energy normalization
 * Returns the normalized value, in the range [0..SCHED_LOAD_SCALE],
 * corresponding to the specified energy variation.
 */
int
schedtune_normalize_energy(int energy_diff)
{
	long long normalized_nrg = energy_diff;
	int max_delta;

	/* Check for boundaries */
	max_delta  = schedtune_target_nrg.max_power;
	max_delta -= schedtune_target_nrg.min_power;
	WARN_ON(abs(energy_diff) >= max_delta);

	/* Scale by energy magnitude */
	normalized_nrg <<= SCHED_LOAD_SHIFT;

	/* Normalize on max energy for target platform */
	normalized_nrg  *= schedtune_target_nrg.nrg_mult;
	normalized_nrg >>= schedtune_target_nrg.nrg_shift;

	return normalized_nrg;
}

static int
__schedtune_accept_deltas(int nrg_delta, int cap_delta,
		int perf_boost_idx, int perf_constrain_idx) {
	int energy_payoff;

	/* Performance Boost (B) region */
	if (nrg_delta > 0 && cap_delta > 0) {
		/*
		 * energy_payoff criteria:
		 *    cap_delta / nrg_delta > cap_gain / nrg_gain
		 * which is:
		 *    nrg_delta * cap_gain < cap_delta * nrg_gain
		 */
		energy_payoff  = cap_delta * threshold_gains[perf_boost_idx].nrg_gain;
		energy_payoff -= nrg_delta * threshold_gains[perf_boost_idx].cap_gain;
		return energy_payoff;
	}

	/* Performance Constraint (C) region */
	if (nrg_delta < 0 && cap_delta < 0) {
		/*
		 * energy_payoff criteria:
		 *    cap_delta / nrg_delta < cap_gain / nrg_gain
		 * which is:
		 *    nrg_delta * cap_gain > cap_delta * nrg_gain
		 */
		energy_payoff  = nrg_delta * threshold_gains[perf_constrain_idx].cap_gain;
		energy_payoff -= cap_delta * threshold_gains[perf_constrain_idx].nrg_gain;
		return energy_payoff;
	}

	/* Default: reject schedule candidate */
	return -INT_MAX;
}

int
schedtune_accept_deltas(int nrg_delta, int cap_delta) {

	/* Optimal (O) region */
	if (nrg_delta < 0 && cap_delta > 0)
		return INT_MAX;

	/* Suboptimal (S) region */
	if (nrg_delta > 0 && cap_delta < 0)
		return -INT_MAX;

	return __schedtune_accept_deltas(nrg_delta, cap_delta,
			perf_boost_idx, perf_constrain_idx);

}

int
sysctl_sched_cfs_boost_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	/* Performance Boost (B) region threshold params */
	perf_boost_idx  = sysctl_sched_cfs_boost;
	perf_boost_idx /= 10;

	/* Performance Constraint (C) region threshold params */
	perf_constrain_idx  = 100 - sysctl_sched_cfs_boost;
	perf_constrain_idx /= 10;

	return 0;
}

