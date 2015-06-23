
#ifdef CONFIG_SCHED_TUNE

extern int schedtune_normalize_energy(int energy);

#else /* CONFIG_SCHED_TUNE */

#define schedtune_normalize_energy(energy) energy

#endif /* CONFIG_SCHED_TUNE */
