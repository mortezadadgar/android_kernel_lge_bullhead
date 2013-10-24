/*
 * intel-mid.c: Intel MID platform setup code
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#define	SFI_SIG_OEM0	"OEM0"

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/intel_pmic_gpio.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/hsi/hsi.h>
#include <linux/spinlock.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/blkdev.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/mrst-vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <asm/reboot.h>
#include "intel_mid_weak_decls.h"
/*
 * the clockevent devices on Moorestown/Medfield can be APBT or LAPIC clock,
 * cmdline option x86_intel_mid_timer can be used to override the configuration
 * to prefer one or the other.
 * at runtime, there are basically three timer configurations:
 * 1. per cpu apbt clock only
 * 2. per cpu always-on lapic clocks only, this is Penwell/Medfield only
 * 3. per cpu lapic clock (C3STOP) and one apbt clock, with broadcast.
 *
 * by default (without cmdline option), platform code first detects cpu type
 * to see if we are on lincroft or penwell, then set up both lapic or apbt
 * clocks accordingly.
 * i.e. by default, medfield uses configuration #2, moorestown uses #1.
 * config #3 is supported but not recommended on medfield.
 *
 * rating and feature summary:
 * lapic (with C3STOP) --------- 100
 * apbt (always-on) ------------ 110
 * lapic (always-on,ARAT) ------ 150
 */

__cpuinitdata enum intel_mid_timer_options intel_mid_timer_options;

struct kobject *spid_kobj;
u32 board_id;
static struct sfi_timer_table_entry sfi_mtimer_array[SFI_MTMR_MAX_NUM];
enum intel_mid_cpu_type __intel_mid_cpu_chip;
EXPORT_SYMBOL_GPL(__intel_mid_cpu_chip);

#ifdef CONFIG_X86_MRFLD
enum intel_mrfl_sim_type __intel_mrfl_sim_platform;
EXPORT_SYMBOL_GPL(__intel_mrfl_sim_platform);
#endif /* X86_CONFIG_MRFLD */

#ifdef CONFIG_X86_MDFLD
void (*saved_shutdown)(void);

/* This function is here just to have a hook to execute code before
 * generic x86 shutdown is executed. saved_shutdown contains pointer
 * to original generic x86 shutdown function */
void mfld_shutdown(void)
{
//BYT	down(&mid_pmu_cxt->scu_ready_sem);

	if (saved_shutdown)
		saved_shutdown();
}
#endif

/* parse all the mtimer info to a static mtimer array */
static int __init sfi_parse_mtmr(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_timer_table_entry *pentry;
	struct mpc_intsrc mp_irq;
	int totallen;

	sb = (struct sfi_table_simple *)table;
	if (!sfi_mtimer_num) {
		sfi_mtimer_num = SFI_GET_NUM_ENTRIES(sb,
					struct sfi_timer_table_entry);
		pentry = (struct sfi_timer_table_entry *) sb->pentry;
		totallen = sfi_mtimer_num * sizeof(*pentry);
		memcpy(sfi_mtimer_array, pentry, totallen);
	}

	pr_debug("SFI MTIMER info (num = %d):\n", sfi_mtimer_num);
	pentry = sfi_mtimer_array;
	for (totallen = 0; totallen < sfi_mtimer_num; totallen++, pentry++) {
		pr_debug("timer[%d]: paddr = 0x%08x, freq = %dHz,"
			" irq = %d\n", totallen, (u32)pentry->phys_addr,
			pentry->freq_hz, pentry->irq);
			if (!pentry->irq)
				continue;
			mp_irq.type = MP_INTSRC;
			mp_irq.irqtype = mp_INT;
/* triggering mode edge bit 2-3, active high polarity bit 0-1 */
			mp_irq.irqflag = 5;
			mp_irq.srcbus = MP_BUS_ISA;
			mp_irq.srcbusirq = pentry->irq;	/* IRQ */
			mp_irq.dstapic = MP_APIC_ALL;
			mp_irq.dstirq = pentry->irq;
			mp_save_irq(&mp_irq);
	}

	return 0;
}

unsigned long __init intel_mid_calibrate_tsc(void)
{
	return 0;
}

static void __init intel_mid_time_init(void)
{
	sfi_table_parse(SFI_SIG_MTMR, NULL, NULL, sfi_parse_mtmr);

/* [REVERT ME] ARAT capability not set in VP. Force setting */
#ifdef CONFIG_X86_MRFLD
	if (intel_mrfl_identify_sim() == INTEL_MRFL_CPU_SIMULATION_VP)
		set_cpu_cap(&boot_cpu_data, X86_FEATURE_ARAT);
#endif /* CONFIG_X86_MRFLD */

	switch (intel_mid_timer_options) {
	case INTEL_MID_TIMER_APBT_ONLY:
		break;
	case INTEL_MID_TIMER_LAPIC_APBT:
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		break;
	default:
		if (!boot_cpu_has(X86_FEATURE_ARAT))
			break;
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		return;
	}
	/* we need at least one APB timer */
	pre_init_apic_IRQ0();
	apbt_time_init();
}

static void __cpuinit intel_mid_arch_setup(void)
{
	if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x27)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_PENWELL;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x26)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_LINCROFT;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x35)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_CLOVERVIEW;
	else if (boot_cpu_data.x86 == 6 && (boot_cpu_data.x86_model == 0x3C ||
					boot_cpu_data.x86_model == 0x4A))
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_TANGIER;
	else {
		pr_err("Unknown Moorestown CPU (%d:%d), default to Lincroft\n",
			boot_cpu_data.x86, boot_cpu_data.x86_model);
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_LINCROFT;
	}
}

static bool force_cold_boot;
module_param(force_cold_boot, bool, 0644);
MODULE_PARM_DESC(force_cold_boot,
		 "Set to Y to force a COLD BOOT instead of a COLD RESET "
		 "on the next reboot system call.");

static void intel_mid_reboot(char *cmd)
{
	if (force_cold_boot) {
		pr_info("Immediate COLD BOOT\n");
		intel_scu_ipc_simple_command(IPCMSG_COLD_BOOT, 0);
	} else {
		pr_info("Immediate COLD RESET\n");
		intel_scu_ipc_simple_command(IPCMSG_COLD_RESET, 0);
	}
}

static void intel_mid_emergency_reboot(void)
{
	/* Change system state to poll IPC status until IPC not busy*/
	system_state = SYSTEM_RESTART;
}

/*
 * Moorestown specific x86_init function overrides and early setup
 * calls.
 */
void __init x86_intel_mid_early_setup(void)
{
	x86_init.resources.probe_roms = x86_init_noop;
	x86_init.resources.reserve_resources = x86_init_noop;

	x86_init.timers.timer_init = intel_mid_time_init;
	x86_init.timers.setup_percpu_clockev = x86_init_noop;

	x86_init.irqs.pre_vector_init = x86_init_noop;

	x86_init.oem.arch_setup = intel_mid_arch_setup;

	x86_cpuinit.setup_percpu_clockev = apbt_setup_secondary_clock;

	x86_platform.calibrate_tsc = intel_mid_calibrate_tsc;
	x86_init.pci.init = pci_mrst_init;
	x86_init.pci.fixup_irqs = x86_init_noop;

	legacy_pic = &null_legacy_pic;

	/* Moorestown specific power_off/restart method */
	pm_power_off = intel_mid_power_off;
#ifdef CONFIG_X86_MDFLD
	if (mfld_shutdown) {
		saved_shutdown = machine_ops.shutdown;
		machine_ops.shutdown = mfld_shutdown;
	}
#endif
	machine_ops.restart = intel_mid_reboot;
	machine_ops.emergency_restart  = intel_mid_emergency_reboot;

	/* Avoid searching for BIOS MP tables */
	x86_init.mpparse.find_smp_config = x86_init_noop;
	x86_init.mpparse.get_smp_config = x86_init_uint_noop;
	set_bit(MP_BUS_ISA, mp_bus_not_pci);
}

#if 0 /* unused code */
#define MAX_SCU_SPI	24
static struct spi_board_info *spi_devs[MAX_SCU_SPI];
static int spi_next_dev;

#define MAX_SCU_I2C	24
static struct i2c_board_info *i2c_devs[MAX_SCU_I2C];
static int i2c_bus[MAX_SCU_I2C];
static int i2c_next_dev;

static void __init intel_scu_spi_device_register(struct spi_board_info *sdev)
{
	struct spi_board_info *new_dev;

	if (spi_next_dev == MAX_SCU_SPI) {
		pr_err("too many SCU SPI devices");
		return;
	}

	new_dev = kzalloc(sizeof(*sdev), GFP_KERNEL);
	if (!new_dev) {
		pr_err("failed to alloc mem for delayed spi dev %s\n",
			sdev->modalias);
		return;
	}
	memcpy(new_dev, sdev, sizeof(*sdev));

	spi_devs[spi_next_dev++] = new_dev;
}

static void __init intel_scu_i2c_device_register(int bus,
						struct i2c_board_info *idev)
{
	struct i2c_board_info *new_dev;

	if (i2c_next_dev == MAX_SCU_I2C) {
		pr_err("too many SCU I2C devices");
		return;
	}

	new_dev = kzalloc(sizeof(*idev), GFP_KERNEL);
	if (!new_dev) {
		pr_err("failed to alloc mem for delayed i2c dev %s\n",
			idev->type);
		return;
	}
	memcpy(new_dev, idev, sizeof(*idev));

	i2c_bus[i2c_next_dev] = bus;
	i2c_devs[i2c_next_dev++] = new_dev;
}
#endif

#define MAX_DELAYEDDEVS 60
static void *delayed_devs[MAX_DELAYEDDEVS];
typedef void (*delayed_callback_t)(void *dev_desc);
static delayed_callback_t delayed_callbacks[MAX_DELAYEDDEVS];
static int delayed_next_dev;

void intel_delayed_device_register(void *dev,
				void (*delayed_callback)(void *dev_desc))
{
	delayed_devs[delayed_next_dev] = dev;
	delayed_callbacks[delayed_next_dev++] = delayed_callback;
	BUG_ON(delayed_next_dev == MAX_DELAYEDDEVS);
}

extern void *byt_audio_platform_data(void *info);

const struct devs_id __initconst platform_device_ids[] = {
#ifdef CONFIG_SND_BYT_AK4614
		{"byt_ak4614",SFI_DEV_TYPE_IPC, 1, &byt_audio_platform_data, NULL },
#elif defined CONFIG_SND_BYT_RT5642
		{"byt_rt5642",SFI_DEV_TYPE_IPC, 1, &byt_audio_platform_data, NULL },
#endif
		{},
};

static void intel_handle_platform_dev ( void )
{
	void *pdata = NULL;
	const struct devs_id *dev = &platform_device_ids[0];
	pr_debug("Platform : intel_handle_platform_dev entry\n");
	
	/*Callback function to register platform devices*/
	pdata = dev->get_platform_data(NULL);
	pr_debug("Platform : intel_handle_platform_dev exit\n");

}

static int __init intel_platform_init(void)
{
	intel_handle_platform_dev ();

	return 0;
}
arch_initcall(intel_platform_init);
