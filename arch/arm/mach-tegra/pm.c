/*
 * CPU complex suspend & resume functions for Tegra SoCs
 *
 * Copyright (c) 2009-2012, NVIDIA Corporation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/err.h>
#include <linux/clk/tegra.h>
#include <linux/serial_reg.h>
#include <linux/syscore_ops.h>
#include <linux/regulator/machine.h>

#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <asm/idmap.h>
#include <asm/proc-fns.h>
#include <asm/tlbflush.h>

#include "common.h"
#include "iomap.h"
#include "reset.h"
#include "flowctrl.h"
#include "fuse.h"
#include "pm.h"
#include "pmc.h"
#include "sleep.h"

#ifdef CONFIG_PM_SLEEP
static DEFINE_SPINLOCK(tegra_lp2_lock);
static u32 iram_save_size;
static void *iram_save_addr;
struct tegra_lp1_iram tegra_lp1_iram;
void (*tegra_tear_down_cpu)(void);
void (*tegra_sleep_core_finish)(unsigned long v2p);
static int (*tegra_sleep_func)(unsigned long v2p);

#ifdef CONFIG_DEBUG_LL
static u8 sctx_uart[5];

static int tegra_debug_uart_suspend(void)
{
	void __iomem *uart;
	u32 lcr;

	/* Debug UART virtual address */
	if (!tegra_uart_config[2])
		return 0;
	else
		uart = (void __iomem *)tegra_uart_config[2];

	lcr = readb(uart + UART_LCR * 4);

	sctx_uart[0] = lcr;
	sctx_uart[1] = readb(uart + UART_MCR * 4);

	/* DLAB = 0 */
	writeb(lcr & ~UART_LCR_DLAB, uart + UART_LCR * 4);

	sctx_uart[2] = readb(uart + UART_IER * 4);

	/* DLAB = 1 */
	writeb(lcr | UART_LCR_DLAB, uart + UART_LCR * 4);

	sctx_uart[3] = readb(uart + UART_DLL * 4);
	sctx_uart[4] = readb(uart + UART_DLM * 4);

	writeb(lcr, uart + UART_LCR * 4);

	return 0;
}

static void tegra_debug_uart_resume(void)
{
	void __iomem *uart;
	u32 lcr;

	/* Debug UART virtual address */
	if (!tegra_uart_config[2])
		return;
	else
		uart = (void __iomem *)tegra_uart_config[2];

	lcr = sctx_uart[0];

	writeb(sctx_uart[1], uart + UART_MCR * 4);

	/* DLAB = 0 */
	writeb(lcr & ~UART_LCR_DLAB, uart + UART_LCR * 4);

	writeb(UART_FCR_ENABLE_FIFO | UART_FCR_T_TRIG_01 | UART_FCR_R_TRIG_01,
	       uart + UART_FCR * 4);
	writeb(sctx_uart[2], uart + UART_IER * 4);

	/* DLAB = 1 */
	writeb(lcr | UART_LCR_DLAB, uart + UART_LCR * 4);

	writeb(sctx_uart[3], uart + UART_DLL * 4);
	writeb(sctx_uart[4], uart + UART_DLM * 4);

	writeb(lcr, uart + UART_LCR * 4);
}

static struct syscore_ops tegra_debug_uart_syscore_ops = {
	.suspend = tegra_debug_uart_suspend,
	.resume = tegra_debug_uart_resume,
};

static void tegra_debug_uart_syscore_init(void)
{
	register_syscore_ops(&tegra_debug_uart_syscore_ops);
}
#endif

static void tegra_tear_down_cpu_init(void)
{
	switch (tegra_chip_id) {
	case TEGRA20:
		if (IS_ENABLED(CONFIG_ARCH_TEGRA_2x_SOC))
			tegra_tear_down_cpu = tegra20_tear_down_cpu;
		break;
	case TEGRA30:
	case TEGRA114:
	case TEGRA124:
		if (IS_ENABLED(CONFIG_ARCH_TEGRA_3x_SOC) ||
		    IS_ENABLED(CONFIG_ARCH_TEGRA_114_SOC) ||
		    IS_ENABLED(CONFIG_ARCH_TEGRA_124_SOC))
			tegra_tear_down_cpu = tegra30_tear_down_cpu;
		break;
	}
}

/*
 * restore_cpu_complex
 *
 * restores cpu clock setting, clears flow controller
 *
 * Always called on CPU 0.
 */
void restore_cpu_complex(void)
{
	int cpu = smp_processor_id();

	BUG_ON(cpu != 0);

#ifdef CONFIG_SMP
	cpu = cpu_logical_map(cpu);
#endif

	/* Restore the CPU clock settings */
	tegra_cpu_clock_resume();

	flowctrl_cpu_suspend_exit(cpu);
}

/*
 * suspend_cpu_complex
 *
 * saves pll state for use by restart_plls, prepares flow controller for
 * transition to suspend state
 *
 * Must always be called on cpu 0.
 */
void suspend_cpu_complex(void)
{
	int cpu = smp_processor_id();

	BUG_ON(cpu != 0);

#ifdef CONFIG_SMP
	cpu = cpu_logical_map(cpu);
#endif

	/* Save the CPU clock settings */
	tegra_cpu_clock_suspend();

	flowctrl_cpu_suspend_enter(cpu);
}

void tegra_clear_cpu_in_lp2(void)
{
	int phy_cpu_id = cpu_logical_map(smp_processor_id());
	u32 *cpu_in_lp2 = tegra_cpu_lp2_mask;

	spin_lock(&tegra_lp2_lock);

	BUG_ON(!(*cpu_in_lp2 & BIT(phy_cpu_id)));
	*cpu_in_lp2 &= ~BIT(phy_cpu_id);

	spin_unlock(&tegra_lp2_lock);
}

bool tegra_set_cpu_in_lp2(void)
{
	int phy_cpu_id = cpu_logical_map(smp_processor_id());
	bool last_cpu = false;
	cpumask_t *cpu_lp2_mask = tegra_cpu_lp2_mask;
	u32 *cpu_in_lp2 = tegra_cpu_lp2_mask;

	spin_lock(&tegra_lp2_lock);

	BUG_ON((*cpu_in_lp2 & BIT(phy_cpu_id)));
	*cpu_in_lp2 |= BIT(phy_cpu_id);

	if ((phy_cpu_id == 0) && cpumask_equal(cpu_lp2_mask, cpu_online_mask))
		last_cpu = true;
	else if (tegra_chip_id == TEGRA20 && phy_cpu_id == 1)
		tegra20_cpu_set_resettable_soon();

	spin_unlock(&tegra_lp2_lock);
	return last_cpu;
}

int tegra_cpu_do_idle(void)
{
	return cpu_do_idle();
}

static int tegra_sleep_cpu(unsigned long v2p)
{
	setup_mm_for_reboot();
	tegra_sleep_cpu_finish(v2p);

	/* should never here */
	BUG();

	return 0;
}

void tegra_idle_last(void)
{
	cpu_cluster_pm_enter();
	suspend_cpu_complex();

	cpu_suspend(PHYS_OFFSET - PAGE_OFFSET, &tegra_sleep_cpu);

	restore_cpu_complex();
	cpu_cluster_pm_exit();
}

void tegra_idle_lp2_last(void)
{
	tegra_pmc_pm_set(TEGRA_SUSPEND_LP2);

	tegra_idle_last();
}

enum tegra_suspend_mode tegra_pm_validate_suspend_mode(
				enum tegra_suspend_mode mode)
{
	enum tegra_suspend_mode avail_mode = TEGRA_SUSPEND_NONE;
	/*
	 * The Tegra devices support suspending to LP1 or lower currently.
	 * Only Tegra114 & Tegra124 supports LP0.
	 */
	switch (tegra_chip_id) {
	case TEGRA114:
	case TEGRA124:
		avail_mode = mode;
		break;
	default:
		if (mode > TEGRA_SUSPEND_LP1)
			avail_mode = TEGRA_SUSPEND_LP1;
		break;
	}

	return avail_mode;
}

static int tegra_sleep_core(unsigned long v2p)
{
	setup_mm_for_reboot();
	tegra_sleep_core_finish(v2p);

	/* should never here */
	BUG();

	return 0;
}

/*
 * tegra_lp1_iram_hook
 *
 * Hooking the address of LP1 reset vector and SDRAM self-refresh code in
 * SDRAM. These codes not be copied to IRAM in this fuction. We need to
 * copy these code to IRAM before LP0/LP1 suspend and restore the content
 * of IRAM after resume.
 */
static bool tegra_lp1_iram_hook(void)
{
	switch (tegra_chip_id) {
	case TEGRA20:
		if (IS_ENABLED(CONFIG_ARCH_TEGRA_2x_SOC))
			tegra20_lp1_iram_hook();
		break;
	case TEGRA30:
	case TEGRA114:
	case TEGRA124:
		if (IS_ENABLED(CONFIG_ARCH_TEGRA_3x_SOC) ||
		    IS_ENABLED(CONFIG_ARCH_TEGRA_114_SOC) ||
		    IS_ENABLED(CONFIG_ARCH_TEGRA_124_SOC))
			tegra30_lp1_iram_hook();
		break;
	default:
		break;
	}

	if (!tegra_lp1_iram.start_addr || !tegra_lp1_iram.end_addr)
		return false;

	iram_save_size = tegra_lp1_iram.end_addr - tegra_lp1_iram.start_addr;
	iram_save_addr = kmalloc(iram_save_size, GFP_KERNEL);
	if (!iram_save_addr)
		return false;

	return true;
}

static bool tegra_sleep_core_init(void)
{
	switch (tegra_chip_id) {
	case TEGRA20:
		if (IS_ENABLED(CONFIG_ARCH_TEGRA_2x_SOC))
			tegra20_sleep_core_init();
		break;
	case TEGRA30:
	case TEGRA114:
	case TEGRA124:
		if (IS_ENABLED(CONFIG_ARCH_TEGRA_3x_SOC) ||
		    IS_ENABLED(CONFIG_ARCH_TEGRA_114_SOC) ||
		    IS_ENABLED(CONFIG_ARCH_TEGRA_124_SOC))
			tegra30_sleep_core_init();
		break;
	default:
		break;
	}

	if (!tegra_sleep_core_finish)
		return false;

	return true;
}

static void tegra_suspend_enter_lp0(void)
{
	tegra_smp_clear_cpu_init_mask();
	tegra_cpu_reset_handler_save();
	tegra_tsc_suspend();
}

static void tegra_suspend_exit_lp0(void)
{
	tegra_tsc_resume();
	tegra_cpu_reset_handler_restore();
}

static void tegra_suspend_enter_lp1(void)
{
	tegra_pmc_suspend();

	/* copy the reset vector & SDRAM shutdown code into IRAM */
	memcpy(iram_save_addr, IO_ADDRESS(TEGRA_IRAM_CODE_AREA),
		iram_save_size);
	memcpy(IO_ADDRESS(TEGRA_IRAM_CODE_AREA), tegra_lp1_iram.start_addr,
		iram_save_size);

	*((u32 *)tegra_cpu_lp1_mask) = 1;
}

static void tegra_suspend_exit_lp1(void)
{
	tegra_pmc_resume();

	/* restore IRAM */
	memcpy(IO_ADDRESS(TEGRA_IRAM_CODE_AREA), iram_save_addr,
		iram_save_size);

	*(u32 *)tegra_cpu_lp1_mask = 0;
}

static const char *lp_state[TEGRA_MAX_SUSPEND_MODE] = {
	[TEGRA_SUSPEND_NONE] = "none",
	[TEGRA_SUSPEND_LP2] = "LP2",
	[TEGRA_SUSPEND_LP1] = "LP1",
	[TEGRA_SUSPEND_LP0] = "LP0",
};

static int __cpuinit tegra_suspend_enter(suspend_state_t state)
{
	enum tegra_suspend_mode mode = tegra_pmc_get_suspend_mode();

	if (WARN_ON(mode < TEGRA_SUSPEND_NONE ||
		    mode >= TEGRA_MAX_SUSPEND_MODE))
		return -EINVAL;

	pr_info("Entering suspend state %s\n", lp_state[mode]);

	tegra_pmc_pm_set(mode);

	local_fiq_disable();

	suspend_cpu_complex();
	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		tegra_suspend_enter_lp0();
	case TEGRA_SUSPEND_LP1:
		tegra_suspend_enter_lp1();
		break;
	case TEGRA_SUSPEND_LP2:
		tegra_set_cpu_in_lp2();
		break;
	default:
		break;
	}

	cpu_suspend(PHYS_OFFSET - PAGE_OFFSET, tegra_sleep_func);

	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		tegra_suspend_exit_lp0();
	case TEGRA_SUSPEND_LP1:
		tegra_suspend_exit_lp1();
		break;
	case TEGRA_SUSPEND_LP2:
		tegra_clear_cpu_in_lp2();
		break;
	default:
		break;
	}
	restore_cpu_complex();

	local_fiq_enable();

	return 0;
}

/*
 * Allow pmc to check if the state is valid first, then return "valid_only_mem".
 * The pmc check has to be done in the valid callback so that user space isn't
 * frozen yet and Tegra124 can load firmware if needed.
 */
static int tegra_suspend_valid(suspend_state_t state)
{
	int ret = tegra_pmc_suspend_valid();
	if (ret)
		return ret;

	ret = regulator_suspend_prepare(state);
	if (ret)
		return ret;

	return suspend_valid_only_mem(state);
}

static const struct platform_suspend_ops tegra_suspend_ops = {
	.valid		= tegra_suspend_valid,
	.enter		= tegra_suspend_enter,
};

void __init tegra_init_suspend(void)
{
	enum tegra_suspend_mode mode = tegra_pmc_get_suspend_mode();

	if (mode == TEGRA_SUSPEND_NONE)
		return;

#ifdef CONFIG_DEBUG_LL
	tegra_debug_uart_syscore_init();
#endif

	tegra_tear_down_cpu_init();
	tegra_pmc_suspend_init();

	if (mode >= TEGRA_SUSPEND_LP1) {
		if (!tegra_lp1_iram_hook() || !tegra_sleep_core_init()) {
			pr_err("%s: unable to allocate memory for SDRAM"
			       "self-refresh -- LP0/LP1 unavailable\n",
			       __func__);
			tegra_pmc_set_suspend_mode(TEGRA_SUSPEND_LP2);
			mode = TEGRA_SUSPEND_LP2;
		}
	}

	/* set up sleep function for cpu_suspend */
	switch (mode) {
	case TEGRA_SUSPEND_LP0:
	case TEGRA_SUSPEND_LP1:
		tegra_sleep_func = tegra_sleep_core;
		break;
	case TEGRA_SUSPEND_LP2:
		tegra_sleep_func = tegra_sleep_cpu;
		break;
	default:
		break;
	}

	suspend_set_ops(&tegra_suspend_ops);
}
#endif
