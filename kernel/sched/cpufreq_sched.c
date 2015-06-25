/*
 *  Copyright (C)  2015 Michael Turquette <mturquette@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/irq_work.h>

#include "sched.h"

#define THROTTLE_NSEC		50000000 /* 50ms default */

static DEFINE_PER_CPU(unsigned long, pcpu_capacity);
static DEFINE_PER_CPU(struct cpufreq_policy *, pcpu_policy);

/**
 * gov_data - per-policy data internal to the governor
 * @throttle: next throttling period expiry. Derived from throttle_nsec
 * @throttle_nsec: throttle period length in nanoseconds
 * @task: worker thread for dvfs transition that may block/sleep
 * @irq_work: callback used to wake up worker thread
 * @freq: new frequency stored in *_sched_update_cpu and used in *_sched_thread
 *
 * struct gov_data is the per-policy cpufreq_sched-specific data structure. A
 * per-policy instance of it is created when the cpufreq_sched governor receives
 * the CPUFREQ_GOV_START condition and a pointer to it exists in the gov_data
 * member of struct cpufreq_policy.
 *
 * Readers of this data must call down_read(policy->rwsem). Writers must
 * call down_write(policy->rwsem).
 */
struct gov_data {
	ktime_t throttle;
	unsigned int throttle_nsec;
	struct task_struct *task;
	struct irq_work irq_work;
	struct cpufreq_policy *policy;
	unsigned int freq;
};

static void cpufreq_sched_try_driver_target(struct cpufreq_policy *policy, unsigned int freq)
{
	struct gov_data *gd = policy->governor_data;

	/* avoid race with cpufreq_sched_stop */
	if (!down_write_trylock(&policy->rwsem))
		return;

	__cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_L);

	gd->throttle = ktime_add_ns(ktime_get(), gd->throttle_nsec);
	up_write(&policy->rwsem);
}

/*
 * we pass in struct cpufreq_policy. This is safe because changing out the
 * policy requires a call to __cpufreq_governor(policy, CPUFREQ_GOV_STOP),
 * which tears down all of the data structures and __cpufreq_governor(policy,
 * CPUFREQ_GOV_START) will do a full rebuild, including this kthread with the
 * new policy pointer
 */
static int cpufreq_sched_thread(void *data)
{
	struct sched_param param;
	struct cpufreq_policy *policy;
	struct gov_data *gd;
	int ret;

	policy = (struct cpufreq_policy *) data;
	if (!policy) {
		pr_warn("%s: missing policy\n", __func__);
		do_exit(-EINVAL);
	}

	gd = policy->governor_data;
	if (!gd) {
		pr_warn("%s: missing governor data\n", __func__);
		do_exit(-EINVAL);
	}

	param.sched_priority = 50;
	ret = sched_setscheduler_nocheck(gd->task, SCHED_FIFO, &param);
	if (ret) {
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		do_exit(-EINVAL);
	} else {
		pr_debug("%s: kthread (%d) set to SCHED_FIFO\n",
				__func__, gd->task->pid);
	}

	ret = set_cpus_allowed_ptr(gd->task, policy->related_cpus);
	if (ret) {
		pr_warn("%s: failed to set allowed ptr\n", __func__);
		do_exit(-EINVAL);
	}

	/* main loop of the per-policy kthread */
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		if (kthread_should_stop())
			break;

		cpufreq_sched_try_driver_target(policy, gd->freq);
	} while (!kthread_should_stop());

	do_exit(0);
}

static void cpufreq_sched_irq_work(struct irq_work *irq_work)
{
	struct gov_data *gd;

	gd = container_of(irq_work, struct gov_data, irq_work);
	if (!gd) {
		return;
	}

	wake_up_process(gd->task);
}

/**
 * cpufreq_sched_set_capacity - interface to scheduler for changing capacity values
 * @cpu: cpu whose capacity utilization has recently changed
 * @capacity: the new capacity requested by cpu
 *
 * cpufreq_sched_sched_capacity is an interface exposed to the scheduler so
 * that the scheduler may inform the governor of updates to capacity
 * utilization and make changes to cpu frequency. Currently this interface is
 * designed around PELT values in CFS. It can be expanded to other scheduling
 * classes in the future if needed.
 *
 * cpufreq_sched_set_capacity raises an IPI. The irq_work handler for that IPI
 * wakes up the thread that does the actual work, cpufreq_sched_thread.
 *
 * This functions bails out early if either condition is true:
 * 1) this cpu did not the new maximum capacity for its frequency domain
 * 2) no change in cpu frequency is necessary to meet the new capacity request
 */
void cpufreq_sched_set_cap(int cpu, unsigned long capacity)
{
	unsigned int freq_new, cpu_tmp;
	struct cpufreq_policy *policy;
	struct gov_data *gd;
	unsigned long capacity_max = 0;

	/* update per-cpu capacity request */
	per_cpu(pcpu_capacity, cpu) = capacity;

	policy = cpufreq_cpu_get(cpu);
	if (IS_ERR_OR_NULL(policy)) {
		return;
	}

	if (!policy->governor_data)
		goto out;

	gd = policy->governor_data;

	/* bail early if we are throttled */
	if (ktime_before(ktime_get(), gd->throttle))
		goto out;

	/* find max capacity requested by cpus in this policy */
	for_each_cpu(cpu_tmp, policy->cpus)
		capacity_max = max(capacity_max, per_cpu(pcpu_capacity, cpu_tmp));

	/*
	 * We only change frequency if this cpu's capacity request represents a
	 * new max. If another cpu has requested a capacity greater than the
	 * previous max then we rely on that cpu to hit this code path and make
	 * the change. IOW, the cpu with the new max capacity is responsible
	 * for setting the new capacity/frequency.
	 *
	 * If this cpu is not the new maximum then bail
	 */
	if (capacity_max > capacity)
		goto out;

	/* Convert the new maximum capacity request into a cpu frequency */
	freq_new = (capacity * policy->max) / capacity_orig_of(cpu);

	/* No change in frequency? Bail and return current capacity. */
	if (freq_new == policy->cur)
		goto out;

	/* store the new frequency and perform the transition */
	gd->freq = freq_new;

	if (cpufreq_driver_might_sleep())
		irq_work_queue_on(&gd->irq_work, cpu);
	else
		cpufreq_sched_try_driver_target(policy, freq_new);

out:
	cpufreq_cpu_put(policy);
	return;
}

/**
 * cpufreq_sched_reset_capacity - interface to scheduler for resetting capacity
 *                                requests
 * @cpu: cpu whose capacity request has to be reset
 *
 * This _wont trigger_ any capacity update.
 */
void cpufreq_sched_reset_cap(int cpu)
{
	per_cpu(pcpu_capacity, cpu) = 0;
}

static inline void set_sched_energy_freq(void)
{
	if (!sched_energy_freq())
		static_key_slow_inc(&__sched_energy_freq);
}

static inline void clear_sched_energy_freq(void)
{
	if (sched_energy_freq())
		static_key_slow_dec(&__sched_energy_freq);
}

static int cpufreq_sched_start(struct cpufreq_policy *policy)
{
	struct gov_data *gd;
	int cpu;

	/* prepare per-policy private data */
	gd = kzalloc(sizeof(*gd), GFP_KERNEL);
	if (!gd) {
		pr_debug("%s: failed to allocate private data\n", __func__);
		return -ENOMEM;
	}

	/* initialize per-cpu data */
	for_each_cpu(cpu, policy->cpus) {
		per_cpu(pcpu_capacity, cpu) = 0;
		per_cpu(pcpu_policy, cpu) = policy;
	}

	/*
	 * Don't ask for freq changes at an higher rate than what
	 * the driver advertises as transition latency.
	 */
	gd->throttle_nsec = policy->cpuinfo.transition_latency ?
			    policy->cpuinfo.transition_latency :
			    THROTTLE_NSEC;
	pr_debug("%s: throttle threshold = %u [ns]\n",
		  __func__, gd->throttle_nsec);

	if (cpufreq_driver_might_sleep()) {
		/* init per-policy kthread */
		gd->task = kthread_run(cpufreq_sched_thread, policy, "kcpufreq_sched_task");
		if (IS_ERR_OR_NULL(gd->task)) {
			pr_err("%s: failed to create kcpufreq_sched_task thread\n", __func__);
			goto err;
		}
		init_irq_work(&gd->irq_work, cpufreq_sched_irq_work);
	}

	policy->governor_data = gd;
	gd->policy = policy;
	set_sched_energy_freq();
	return 0;

err:
	kfree(gd);
	return -ENOMEM;
}

static int cpufreq_sched_stop(struct cpufreq_policy *policy)
{
	struct gov_data *gd = policy->governor_data;

	clear_sched_energy_freq();
	if (cpufreq_driver_might_sleep()) {
		kthread_stop(gd->task);
	}

	policy->governor_data = NULL;

	/* FIXME replace with devm counterparts? */
	kfree(gd);
	return 0;
}

static int cpufreq_sched_setup(struct cpufreq_policy *policy, unsigned int event)
{
	switch (event) {
		case CPUFREQ_GOV_START:
			/* Start managing the frequency */
			return cpufreq_sched_start(policy);

		case CPUFREQ_GOV_STOP:
			return cpufreq_sched_stop(policy);

		case CPUFREQ_GOV_LIMITS:	/* unused */
		case CPUFREQ_GOV_POLICY_INIT:	/* unused */
		case CPUFREQ_GOV_POLICY_EXIT:	/* unused */
			break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHED
static
#endif
struct cpufreq_governor cpufreq_gov_sched = {
	.name			= "sched",
	.governor		= cpufreq_sched_setup,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_sched_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_sched);
}

static void __exit cpufreq_sched_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_sched);
}

/* Try to make this the default governor */
fs_initcall(cpufreq_sched_init);

MODULE_LICENSE("GPL v2");
