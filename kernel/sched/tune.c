#include "sched.h"
#include "tune.h"

unsigned int sysctl_sched_cfs_boost __read_mostly = 0;

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

int
sysctl_sched_cfs_boost_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	return 0;
}
