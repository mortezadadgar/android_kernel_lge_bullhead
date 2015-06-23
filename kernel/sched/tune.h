
#ifdef CONFIG_SCHED_TUNE

extern int schedtune_normalize_energy(int energy);

#ifdef CONFIG_CGROUP_SCHEDTUNE

extern int schedtune_accept_deltas(int nrg_delta, int cap_delta,
		struct task_struct *task);

#else /* CONFIG_CGROUP_SCHEDTUNE */

extern int schedtune_accept_deltas(int nrg_delta, int cap_delta);

#endif /* CONFIG_CGROUP_SCHEDTUNE */

#else /* CONFIG_SCHED_TUNE */

#define schedtune_normalize_energy(energy) energy
#define schedtune_accept_deltas(nrg_delta, cap_delta) nrg_delta

#endif /* CONFIG_SCHED_TUNE */
