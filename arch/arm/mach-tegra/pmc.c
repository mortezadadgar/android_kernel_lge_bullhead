/*
 * Copyright (C) 2012,2013 NVIDIA CORPORATION. All rights reserved.
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
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/memblock.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/tegra-powergate.h>
#include <linux/tegra-soc.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>
#include <linux/tegra-soc.h>

#include "flowctrl.h"
#include "pm.h"
#include "pmc.h"
#include "sleep.h"

/*
 * When another Tegra variant supports loading LP0 code with request_firmware,
 * this #define should be converted to a lookup table so each SoC can have its
 * own firmware name.
 */
#define LP0_FW_NAME "tegra12x/tegra_lp0_resume.fw"

#define TEGRA_POWER_PWRREQ_POLARITY	(1 << 8)   /* core power req polarity */
#define TEGRA_POWER_PWRREQ_OE		(1 << 9)   /* core power req enable */
#define TEGRA_POWER_SYSCLK_POLARITY	(1 << 10)  /* sys clk polarity */
#define TEGRA_POWER_SYSCLK_OE		(1 << 11)  /* system clock enable */
#define TEGRA_POWER_EFFECT_LP0		(1 << 14)  /* LP0 when CPU pwr gated */
#define TEGRA_POWER_CPU_PWRREQ_POLARITY	(1 << 15)  /* CPU pwr req polarity */
#define TEGRA_POWER_CPU_PWRREQ_OE	(1 << 16)  /* CPU pwr req enable */

#define PMC_CTRL			0x0
#define PMC_CTRL_LATCH_WAKEUPS		(1 << 5)
#define PMC_CTRL_INTR_LOW		(1 << 17)
#define PMC_WAKE_MASK			0xc
#define PMC_WAKE_LEVEL			0x10
#define PMC_WAKE_STATUS			0x14
#define PMC_SW_WAKE_STATUS		0x18
#define PMC_DPD_SAMPLE			0x20
#define PMC_DPD_ENABLE			0x24
#define PMC_DPD_ENABLE_ON		0x1
#define PMC_DPD_ENABLE_TSC_MULT_ENABLE	(1 << 1)
#define PMC_PWRGATE_TOGGLE		0x30
#define PMC_PWRGATE_TOGGLE_START	(1 << 8)
#define PMC_REMOVE_CLAMPING		0x34
#define PMC_PWRGATE_STATUS		0x38
#define PMC_COREPWRGOOD_TIMER		0x3c
#define PMC_SCRATCH0			0x50
#define PMC_SCRATCH1			0x54
#define PMC_CPUPWRGOOD_TIMER		0xc8
#define PMC_CPUPWROFF_TIMER		0xcc
#define PMC_WAKE_DELAY			0xe0
#define PMC_COREPWROFF_TIMER		PMC_WAKE_DELAY
#define PMC_WAKE2_MASK			0x160
#define PMC_WAKE2_LEVEL			0x164
#define PMC_WAKE2_STATUS		0x168
#define PMC_SW_WAKE2_STATUS		0x16C
#define PMC_IO_DPD_REQ			0x1B8
#define IO_DPD_CSIA			(1 << 0)
#define IO_DPD_CSIB			(1 << 1)
#define IO_DPD_DSI			(1 << 2)
#define IO_DPD_MIPI_BIAS		(1 << 3)
#define IO_DPD_PEX_BIAS			(1 << 4)
#define IO_DPD_PEX_CLK1			(1 << 5)
#define IO_DPD_PEX_CLK2			(1 << 6)
#define IO_DPD_PEX_CLK3			(1 << 7)
#define IO_DPD_DAC			(1 << 8)
#define IO_DPD_USB0			(1 << 9)
#define IO_DPD_USB1			(1 << 10)
#define IO_DPD_USB2			(1 << 11)
#define IO_DPD_USB_BIAS			(1 << 12)
#define IO_DPD_NAND			(1 << 13)
#define IO_DPD_UART			(1 << 14)
#define IO_DPD_BB			(1 << 15)
#define IO_DPD_VI			(1 << 16)
#define IO_DPD_AUDIO			(1 << 17)
#define IO_DPD_LCD			(1 << 18)
#define IO_DPD_HSIC			(1 << 19)
#define IO_DPD_ON			(2 << 30)
#define IO_DPD_OFF			(1 << 30)
#define PMC_IO_DPD2_REQ			0x1C0
#define IO_DPD2_PEX_CNTRL		(1 << 0)
#define IO_DPD2_SDMMC1			(1 << 1)
#define IO_DPD2_SDMMC3			(1 << 2)
#define IO_DPD2_SDMMC4			(1 << 3)
#define IO_DPD2_CAM			(1 << 4)
#define IO_DPD2_RES_RAIL		(1 << 5)
#define IO_DPD2_HV			(1 << 6)
#define IO_DPD2_DSIB			(1 << 7)
#define IO_DPD2_DSIC			(1 << 8)
#define IO_DPD2_DSID			(1 << 9)
#define IO_DPD2_CSIC			(1 << 10)
#define IO_DPD2_CSID			(1 << 11)
#define IO_DPD2_CSIE			(1 << 12)

#define DPD_STATE_CHANGE_DELAY		700

static u8 tegra_cpu_domains[] = {
	0xFF,			/* not available for CPU0 */
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};
static DEFINE_SPINLOCK(tegra_powergate_lock);

static void __iomem *tegra_pmc_base;
static bool tegra_pmc_invert_interrupt;
static struct clk *tegra_pclk;

struct pmc_pm_data {
	u32 cpu_good_time;	/* CPU power good time in uS */
	u32 cpu_off_time;	/* CPU power off time in uS */
	u32 core_osc_time;	/* Core power good osc time in uS */
	u32 core_pmu_time;	/* Core power good pmu time in uS */
	u32 core_off_time;	/* Core power off time in uS */
	bool corereq_high;	/* Core power request active-high */
	bool sysclkreq_high;	/* System clock request active-high */
	bool combined_req;	/* Combined pwr req for CPU & Core */
	bool cpu_pwr_good_en;	/* CPU power good signal is enabled */
	u32 lp0_vec_phy_addr;	/* The phy addr of LP0 warm boot code */
	u32 lp0_vec_size;	/* The size of LP0 warm boot code */
	enum tegra_suspend_mode suspend_mode;
	int reset_gpio;		/* GPIO to assert to reset the system */
	bool reset_active_low;	/* Reset GPIO is active-low */
};
static struct pmc_pm_data pmc_pm_data;

#ifdef CONFIG_PM_SLEEP
#define PMC_WAKE_TYPE_GPIO	0
#define PMC_WAKE_TYPE_EVENT	1
#define PMC_WAKE_TYPE_INDEX	0
#define PMC_WAKE_MASK_INDEX	1
#define PMC_TRIGGER_TYPE_INDEX	2
struct pmc_wakeup {
	u32 wake_type;
	u32 wake_mask_offset;
	u32 irq_num;
	struct list_head list;
};

struct pmc_lp0_wakeup {
	struct device_node *of_node;
	u64 enable;
	u64 level;
	u64 level_any;
	struct list_head wake_list;
};
static struct pmc_lp0_wakeup tegra_lp0_wakeup;
static u32 io_dpd_reg, io_dpd2_reg;
#endif

static inline u32 tegra_pmc_readl(u32 reg)
{
	return readl(tegra_pmc_base + reg);
}

static inline void tegra_pmc_writel(u32 val, u32 reg)
{
	writel(val, tegra_pmc_base + reg);
}

static int tegra_pmc_get_cpu_powerdomain_id(int cpuid)
{
	if (cpuid <= 0 || cpuid >= num_possible_cpus())
		return -EINVAL;
	return tegra_cpu_domains[cpuid];
}

static bool tegra_pmc_powergate_is_powered(int id)
{
	return (tegra_pmc_readl(PMC_PWRGATE_STATUS) >> id) & 1;
}

static int tegra_pmc_powergate_set(int id, bool new_state)
{
	bool old_state;
	unsigned long flags;

	spin_lock_irqsave(&tegra_powergate_lock, flags);

	old_state = tegra_pmc_powergate_is_powered(id);
	WARN_ON(old_state == new_state);

	tegra_pmc_writel(PMC_PWRGATE_TOGGLE_START | id, PMC_PWRGATE_TOGGLE);

	spin_unlock_irqrestore(&tegra_powergate_lock, flags);

	return 0;
}

static int tegra_pmc_powergate_remove_clamping(int id)
{
	u32 mask;

	/*
	 * Tegra has a bug where PCIE and VDE clamping masks are
	 * swapped relatively to the partition ids.
	 */
	if (id ==  TEGRA_POWERGATE_VDEC)
		mask = (1 << TEGRA_POWERGATE_PCIE);
	else if	(id == TEGRA_POWERGATE_PCIE)
		mask = (1 << TEGRA_POWERGATE_VDEC);
	else
		mask = (1 << id);

	tegra_pmc_writel(mask, PMC_REMOVE_CLAMPING);

	return 0;
}

bool tegra_pmc_cpu_is_powered(int cpuid)
{
	int id;

	id = tegra_pmc_get_cpu_powerdomain_id(cpuid);
	if (id < 0)
		return false;
	return tegra_pmc_powergate_is_powered(id);
}

int tegra_pmc_cpu_power_on(int cpuid)
{
	int id;

	id = tegra_pmc_get_cpu_powerdomain_id(cpuid);
	if (id < 0)
		return id;
	return tegra_pmc_powergate_set(id, true);
}

int tegra_pmc_cpu_remove_clamping(int cpuid)
{
	int id;

	id = tegra_pmc_get_cpu_powerdomain_id(cpuid);
	if (id < 0)
		return id;
	return tegra_pmc_powergate_remove_clamping(id);
}

void tegra_pmc_restart(char mode, const char *cmd)
{
	u32 val;

	/*
	 * If there's a reset GPIO, attempt to use that first and then fall
	 * back to PMC reset if that fails.
	 */
	if (gpio_is_valid(pmc_pm_data.reset_gpio)) {
		val = pmc_pm_data.reset_active_low ? 0 : 1;
		gpio_direction_output(pmc_pm_data.reset_gpio, val);
		udelay(100);
		pr_err("GPIO reset failed; using PMC reset...\n");
	}

	val = tegra_pmc_readl(0);
	val |= 0x10;
	tegra_pmc_writel(val, 0);
}

#ifdef CONFIG_PM_SLEEP
void tegra_tsc_suspend(void)
{
	if (IS_ENABLED(CONFIG_ARM_ARCH_TIMER)) {
		u32 reg;
		reg = tegra_pmc_readl(PMC_DPD_ENABLE);
		reg |= PMC_DPD_ENABLE_TSC_MULT_ENABLE;
		tegra_pmc_writel(reg, PMC_DPD_ENABLE);
	}
}

void tegra_tsc_resume(void)
{
	if (IS_ENABLED(CONFIG_ARM_ARCH_TIMER)) {
		u32 reg;
		reg = tegra_pmc_readl(PMC_DPD_ENABLE);
		reg &= ~PMC_DPD_ENABLE_TSC_MULT_ENABLE;
		if (tegra_chip_id == TEGRA124) {
			/* WAR to avoid PMC wake status getting cleared */
			reg &= ~PMC_DPD_ENABLE_ON;
		}
		tegra_pmc_writel(reg, PMC_DPD_ENABLE);
	}
}

static void tegra_pmc_add_wakeup_event(struct of_phandle_args *ph_args,
				       struct device *dev,
				       struct device_node *np)
{
	if (ph_args->np == tegra_lp0_wakeup.of_node) {
		struct platform_device *pdev;
		struct pmc_wakeup *pmc_wake_source;
		int pmc_wake_type, wake;
		int irq = 0, pmc_trigger_type = 0;

		pdev = to_platform_device(dev);
		irq = platform_get_irq(pdev, 0);
		pmc_wake_type = ph_args->args[PMC_WAKE_TYPE_INDEX];

		switch (pmc_wake_type) {
		case PMC_WAKE_TYPE_GPIO:
			if (pdev != NULL) {
				struct irq_desc *irqd;
				struct irq_data *irq_data;

				if (irq < 0) {
					int gpio;
					gpio = of_get_named_gpio(np,
								 "gpios", 0);
					irq = gpio_to_irq(gpio);
					if (irq < 0) {
						WARN_ON(1);
						return;
					}
				}
				irqd = irq_to_desc(irq);
				irq_data = &irqd->irq_data;
				pmc_trigger_type =
					irqd_get_trigger_type(irq_data);
			}
			break;
		case PMC_WAKE_TYPE_EVENT:
			pmc_trigger_type =
				ph_args->args[PMC_TRIGGER_TYPE_INDEX];
			break;
		default:
			return;
		}

		pmc_wake_source = kzalloc(sizeof(*pmc_wake_source),
					  GFP_KERNEL);
		if (!pmc_wake_source) {
			pr_err("%s: fail to alloc memory.", __func__);
			return;
		}

		pmc_wake_source->wake_type = pmc_wake_type;
		pmc_wake_source->irq_num = irq;
		pmc_wake_source->wake_mask_offset =
				ph_args->args[PMC_WAKE_MASK_INDEX];
		wake = pmc_wake_source->wake_mask_offset;

		list_add_tail(&pmc_wake_source->list,
			      &tegra_lp0_wakeup.wake_list);

		tegra_lp0_wakeup.enable |= 1ull << wake;
		switch (pmc_trigger_type) {
		case IRQF_TRIGGER_FALLING:
		case IRQF_TRIGGER_LOW:
			tegra_lp0_wakeup.level &= ~(1ull << wake);
			tegra_lp0_wakeup.level_any &= ~(1ull << wake);
			break;
		case IRQF_TRIGGER_HIGH:
		case IRQF_TRIGGER_RISING:
			tegra_lp0_wakeup.level |= (1ull << wake);
			tegra_lp0_wakeup.level_any &= ~(1ull << wake);
			break;
		case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING:
			tegra_lp0_wakeup.level_any |= (1ull << wake);
			break;
		default:
			break;
		}
	}
}

static void tegra_of_device_add_pmc_wake(struct device *dev)
{
	struct of_phandle_args ph_args;
	struct device_node *np = NULL;
	int child_node_num;

	child_node_num = of_get_child_count(dev->of_node);
	if (child_node_num == 0) {
		if (!of_parse_phandle_with_args(dev->of_node,
					       "nvidia,pmc-wakeup",
					       "#wake-cells", 0, &ph_args))
			tegra_pmc_add_wakeup_event(&ph_args, dev, dev->of_node);
	} else {
		for_each_child_of_node(dev->of_node, np)
			if (!of_parse_phandle_with_args(np,
						"nvidia,pmc-wakeup",
						"#wake-cells", 0, &ph_args))
				tegra_pmc_add_wakeup_event(&ph_args, dev, np);
	}

	of_node_put(ph_args.np);
}

static int tegra_pmc_wake_notifier_call(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	struct device *dev = data;

	switch (event) {
	case BUS_NOTIFY_BOUND_DRIVER:
		if (dev->of_node)
			tegra_of_device_add_pmc_wake(dev);
	break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tegra_pmc_wake_notifier = {
	.notifier_call = tegra_pmc_wake_notifier_call,
};

void __init tegra_pmc_lp0_wakeup_init(void)
{
	bus_register_notifier(&platform_bus_type, &tegra_pmc_wake_notifier);
}

static inline void write_pmc_wake_mask(u64 value)
{
	pr_info("PMC wake enable = 0x%llx\n", value);
	tegra_pmc_writel((u32)value, PMC_WAKE_MASK);
	if (tegra_chip_id != TEGRA20)
		tegra_pmc_writel((u32)(value >> 32), PMC_WAKE2_MASK);
}

static inline u64 read_pmc_wake_level(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_WAKE_LEVEL);
	if (tegra_chip_id != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_WAKE2_LEVEL)) << 32;

	return reg;
}

static inline void write_pmc_wake_level(u64 value)
{
	pr_info("PMC wake level = 0x%llx\n", value);
	tegra_pmc_writel((u32)value, PMC_WAKE_LEVEL);
	if (tegra_chip_id != TEGRA20)
		tegra_pmc_writel((u32)(value >> 32), PMC_WAKE2_LEVEL);
}

static inline u64 read_pmc_wake_status(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_WAKE_STATUS);
	if (tegra_chip_id != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_WAKE2_STATUS)) << 32;

	return reg;
}

static inline void clear_pmc_wake_status(void)
{
	u32 reg;

	reg = tegra_pmc_readl(PMC_WAKE_STATUS);
	if (reg)
		tegra_pmc_writel(reg, PMC_WAKE_STATUS);
	if (tegra_chip_id != TEGRA20) {
		reg = tegra_pmc_readl(PMC_WAKE2_STATUS);
		if (reg)
			tegra_pmc_writel(reg, PMC_WAKE2_STATUS);
	}
}

static inline u64 read_pmc_sw_wake_status(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_SW_WAKE_STATUS);
	if (tegra_chip_id != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_SW_WAKE2_STATUS)) << 32;

	return reg;
}

static inline void clear_pmc_sw_wake_status(void)
{
	tegra_pmc_writel(0, PMC_SW_WAKE_STATUS);
	if (tegra_chip_id != TEGRA20)
		tegra_pmc_writel(0, PMC_SW_WAKE2_STATUS);
}

/* translate lp0 wake sources back into irqs to catch edge triggered wakeups */
static void tegra_pmc_wake_irq_helper(unsigned long wake_status, u32 index)
{
	u32 wake;

	for_each_set_bit(wake, &wake_status, 32) {
		struct pmc_wakeup *wake_source;
		list_for_each_entry(wake_source,
				    &tegra_lp0_wakeup.wake_list, list) {
			if (wake_source->wake_mask_offset ==
					(wake + 32 * index)) {
				struct irq_desc *desc;

				if (wake_source->irq_num <= 0) {
					pr_info("Resume caused by PMC WAKE%d\n",
							(wake + 32 * index));
					continue;
				}

				desc = irq_to_desc(wake_source->irq_num);
				if (!desc || !desc->action ||
						!desc->action->name) {
					pr_info("Resume caused by PMC WAKE%d",
						(wake + 32 * index));
					pr_cont(", irq %d\n",
						wake_source->irq_num);
					continue;
				}

				pr_info("Resume caused by PMC WAKE%d, %s\n",
						(wake + 32 * index),
						desc->action->name);
				generic_handle_irq(wake_source->irq_num);
			}
		}
	}
}

static void tegra_pmc_wake_syscore_resume(void)
{
	u64 wake_status = read_pmc_wake_status();

	pr_info("PMC wake status = 0x%llx\n", wake_status);
	tegra_pmc_wake_irq_helper((unsigned long)wake_status, 0);
	if (tegra_chip_id != TEGRA20)
		tegra_pmc_wake_irq_helper((unsigned long)(wake_status >> 32),
					  1);
}

static int tegra_pmc_wake_syscore_suspend(void)
{
	u32 reg;
	u64 status;
	u64 lvl;
	u64 wake_level;
	u64 wake_enb;

	clear_pmc_sw_wake_status();

	/* enable PMC wake */
	reg = tegra_pmc_readl(PMC_CTRL);
	reg |= PMC_CTRL_LATCH_WAKEUPS;
	tegra_pmc_writel(reg, PMC_CTRL);
	udelay(120);

	reg &= ~PMC_CTRL_LATCH_WAKEUPS;
	tegra_pmc_writel(reg, PMC_CTRL);
	udelay(120);

	status = read_pmc_sw_wake_status();

	lvl = read_pmc_wake_level();

	/*
	 * flip the wakeup trigger for any-edge triggered pads
	 * which are currently asserting as wakeups
	 */
	lvl ^= status;

	lvl &= tegra_lp0_wakeup.level_any;

	wake_level = lvl | tegra_lp0_wakeup.level;
	wake_enb = tegra_lp0_wakeup.enable;

	/* Clear PMC Wake Status registers while going to suspend */
	clear_pmc_wake_status();

	write_pmc_wake_level(wake_level);

	write_pmc_wake_mask(wake_enb);

	return 0;
}

static struct syscore_ops tegra_pmc_wake_syscore_ops = {
	.suspend = tegra_pmc_wake_syscore_suspend,
	.resume = tegra_pmc_wake_syscore_resume,
};

static void tegra_pmc_wake_syscore_init(void)
{
	register_syscore_ops(&tegra_pmc_wake_syscore_ops);
}

static void set_core_power_timers(void)
{
	unsigned long osc, pmu, off;

	osc = DIV_ROUND_UP_ULL(pmc_pm_data.core_osc_time * 32768, 1000000);
	pmu = DIV_ROUND_UP_ULL(pmc_pm_data.core_pmu_time * 32768, 1000000);
	off = DIV_ROUND_UP_ULL(pmc_pm_data.core_off_time * 32768, 1000000);

	tegra_pmc_writel(((osc << 8) & 0xff00) | (pmu & 0xff),
			 PMC_COREPWRGOOD_TIMER);
	tegra_pmc_writel(off, PMC_COREPWROFF_TIMER);
}

static void set_power_timers(u32 us_on, u32 us_off, unsigned long rate)
{
	unsigned long long ticks;
	unsigned long long pclk;
	static unsigned long tegra_last_pclk;

	if (WARN_ON_ONCE(rate <= 0))
		pclk = 100000000;
	else
		pclk = rate;

	if ((rate != tegra_last_pclk)) {
		ticks = (us_on * pclk) + 999999ull;
		do_div(ticks, 1000000);
		tegra_pmc_writel((unsigned long)ticks, PMC_CPUPWRGOOD_TIMER);

		ticks = (us_off * pclk) + 999999ull;
		do_div(ticks, 1000000);
		tegra_pmc_writel((unsigned long)ticks, PMC_CPUPWROFF_TIMER);
		wmb();
	}
	tegra_last_pclk = pclk;
}

void tegra_pmc_remove_dpd_req(void)
{
	/* Clear DPD req */
	tegra_pmc_writel(io_dpd_reg | IO_DPD_OFF, PMC_IO_DPD_REQ);
	tegra_pmc_readl(PMC_IO_DPD_REQ); /* unblock posted write */
	/* delay apb_clk * (SEL_DPD_TIM*5) */
	udelay(DPD_STATE_CHANGE_DELAY);

	tegra_pmc_writel(io_dpd2_reg | IO_DPD_OFF, PMC_IO_DPD2_REQ);
	tegra_pmc_readl(PMC_IO_DPD2_REQ); /* unblock posted write */
	udelay(DPD_STATE_CHANGE_DELAY);
}
EXPORT_SYMBOL(tegra_pmc_remove_dpd_req);

void tegra_pmc_clear_dpd_sample(void)
{
	/* Clear DPD sample */
	tegra_pmc_writel(0x0, PMC_DPD_SAMPLE);
}
EXPORT_SYMBOL(tegra_pmc_clear_dpd_sample);

enum tegra_suspend_mode tegra_pmc_get_suspend_mode(void)
{
	return pmc_pm_data.suspend_mode;
}

void tegra_pmc_set_suspend_mode(enum tegra_suspend_mode mode)
{
	if (mode < TEGRA_SUSPEND_NONE || mode >= TEGRA_MAX_SUSPEND_MODE)
		return;

	pmc_pm_data.suspend_mode = mode;
}

void tegra_pmc_suspend(void)
{
	tegra_pmc_writel(virt_to_phys(tegra_resume), PMC_SCRATCH41);
}

void tegra_pmc_resume(void)
{
	/* Clear DPD Enable */
	if (tegra_chip_id == TEGRA124)
		tegra_pmc_writel(0x0, PMC_DPD_ENABLE);

	tegra_pmc_writel(0x0, PMC_SCRATCH41);
}

void tegra_pmc_pm_set(enum tegra_suspend_mode mode)
{
	u32 reg, csr_reg, boot_flag;
	u32 us_off = pmc_pm_data.cpu_off_time;
	unsigned long rate = 0;

	reg = tegra_pmc_readl(PMC_CTRL);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	if (pmc_pm_data.combined_req)
		reg &= ~TEGRA_POWER_PWRREQ_OE;
	else
		reg |= TEGRA_POWER_PWRREQ_OE;
	reg &= ~TEGRA_POWER_EFFECT_LP0;

	switch (tegra_chip_id) {
	case TEGRA20:
	case TEGRA30:
		break;
	default:
		/* Turn off CRAIL */
		if (mode != TEGRA_CLUSTER_SWITCH) {
			csr_reg = flowctrl_read_cpu_csr(0);
			csr_reg &= ~FLOW_CTRL_CSR_ENABLE_EXT_MASK;
			if (is_lp_cluster())
				csr_reg |= FLOW_CTRL_CSR_ENABLE_EXT_NCPU;
			else
				csr_reg |= FLOW_CTRL_CSR_ENABLE_EXT_CRAIL;
			flowctrl_write_cpu_csr(0, csr_reg);
		}
		break;
	}

	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		/*
		 * Enable DPD sample to trigger sampling pads data and direction
		 * in which pad will be driven during LP0 mode.
		 */
		tegra_pmc_writel(0x1, PMC_DPD_SAMPLE);

		/*
		 * Power down IO logic
		 */
		switch (tegra_chip_id) {
		case TEGRA114:
		case TEGRA124:
			io_dpd_reg = IO_DPD_CSIA | IO_DPD_CSIB | IO_DPD_DSI |
				IO_DPD_MIPI_BIAS | IO_DPD_PEX_BIAS |
				IO_DPD_PEX_CLK1 | IO_DPD_PEX_CLK2 |
				IO_DPD_PEX_CLK3 | IO_DPD_DAC | IO_DPD_USB0 |
				IO_DPD_USB1 | IO_DPD_USB2 | IO_DPD_USB_BIAS |
				IO_DPD_UART | IO_DPD_BB | IO_DPD_VI |
				IO_DPD_AUDIO | IO_DPD_LCD | IO_DPD_HSIC;
			io_dpd2_reg = IO_DPD2_PEX_CNTRL | IO_DPD2_SDMMC1 |
				IO_DPD2_SDMMC3 | IO_DPD2_SDMMC4 | IO_DPD2_CAM |
				IO_DPD2_RES_RAIL | IO_DPD2_HV | IO_DPD2_DSIB |
				IO_DPD2_DSIC | IO_DPD2_DSID | IO_DPD2_CSIC |
				IO_DPD2_CSID | IO_DPD2_CSIE;
			break;
		default:
			break;
		}
		tegra_pmc_writel(io_dpd_reg | IO_DPD_ON, PMC_IO_DPD_REQ);
		tegra_pmc_readl(PMC_IO_DPD_REQ); /* unblock posted write */

		/* delay apb_clk * (SEL_DPD_TIM*5) */
		udelay(DPD_STATE_CHANGE_DELAY);

		tegra_pmc_writel(io_dpd2_reg | IO_DPD_ON, PMC_IO_DPD2_REQ);
		tegra_pmc_readl(PMC_IO_DPD2_REQ); /* unblock posted write */
		udelay(DPD_STATE_CHANGE_DELAY);

		/* Set warmboot flag */
		boot_flag = tegra_pmc_readl(PMC_SCRATCH0);
		tegra_pmc_writel(boot_flag | 1, PMC_SCRATCH0);

		tegra_pmc_writel(pmc_pm_data.lp0_vec_phy_addr, PMC_SCRATCH1);
		reg |= TEGRA_POWER_EFFECT_LP0;
	case TEGRA_SUSPEND_LP1:
		rate = 32768;
		break;
	case TEGRA_SUSPEND_LP2:
		rate = __clk_get_rate(tegra_pclk);
		break;
	case TEGRA_CLUSTER_SWITCH:
		rate = __clk_get_rate(tegra_pclk);
		us_off = 2;
		break;
	default:
		break;
	}

	set_power_timers(pmc_pm_data.cpu_good_time, us_off, rate);

	tegra_pmc_writel(reg, PMC_CTRL);
}

void tegra_pmc_suspend_init(void)
{
	u32 reg;

	/* Always enable CPU power request */
	reg = tegra_pmc_readl(PMC_CTRL);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	tegra_pmc_writel(reg, PMC_CTRL);

	set_core_power_timers();

	reg = tegra_pmc_readl(PMC_CTRL);

	if (!pmc_pm_data.sysclkreq_high)
		reg |= TEGRA_POWER_SYSCLK_POLARITY;
	else
		reg &= ~TEGRA_POWER_SYSCLK_POLARITY;

	if (!pmc_pm_data.corereq_high)
		reg |= TEGRA_POWER_PWRREQ_POLARITY;
	else
		reg &= ~TEGRA_POWER_PWRREQ_POLARITY;

	/* configure the output polarity while the request is tristated */
	tegra_pmc_writel(reg, PMC_CTRL);

	/* now enable the request */
	reg |= TEGRA_POWER_SYSCLK_OE;
	tegra_pmc_writel(reg, PMC_CTRL);

	tegra_pmc_wake_syscore_init();
}

/*
 * When starting to enter LP0 without LP0 boot code, try to request the
 * code with request_firmware, if it can't be loaded, switch to LP1.
 */
int tegra_pmc_suspend_valid(void)
{
	const struct firmware *fw;
	int ret;
	uint8_t *fw_buff;

	if (pmc_pm_data.suspend_mode != TEGRA_SUSPEND_LP0 ||
	    pmc_pm_data.lp0_vec_size)
		return 0;

	ret = request_firmware(&fw, LP0_FW_NAME, NULL);
	if (ret) {
		pr_info("Disabling LP0, no resume code found\n");
		pmc_pm_data.suspend_mode = TEGRA_SUSPEND_LP1;
		return 0;
	}

	fw_buff = kmalloc(fw->size, GFP_KERNEL);
	if (!fw_buff) {
		pr_debug("Couldn't allocate %zu bytes LP0 code, disabling\n",
			 fw->size);
		pmc_pm_data.suspend_mode = TEGRA_SUSPEND_LP1;
		goto suspend_check_done;
	}
	pr_info("Loaded LP0 firmware with request_firmware.\n");

	memcpy(fw_buff, fw->data, fw->size);
	pmc_pm_data.lp0_vec_phy_addr = virt_to_phys(fw_buff);
	pmc_pm_data.lp0_vec_size = fw->size;

suspend_check_done:
	release_firmware(fw);
	return 0;
}
#endif

static const struct of_device_id matches[] __initconst = {
	{ .compatible = "nvidia,tegra124-pmc" },
	{ .compatible = "nvidia,tegra114-pmc" },
	{ .compatible = "nvidia,tegra30-pmc" },
	{ .compatible = "nvidia,tegra20-pmc" },
	{ }
};

void __init tegra_pmc_init_irq(void)
{
	struct device_node *np;
	u32 val;

	np = of_find_matching_node(NULL, matches);
	BUG_ON(!np);

	tegra_pmc_base = of_iomap(np, 0);

	tegra_pmc_invert_interrupt = of_property_read_bool(np,
				     "nvidia,invert-interrupt");

	val = tegra_pmc_readl(PMC_CTRL);
	if (tegra_pmc_invert_interrupt)
		val |= PMC_CTRL_INTR_LOW;
	else
		val &= ~PMC_CTRL_INTR_LOW;
	tegra_pmc_writel(val, PMC_CTRL);
}

void __init tegra_pmc_init(void)
{
	struct device_node *np;
	u32 prop;
	enum tegra_suspend_mode suspend_mode;
	u32 core_good_time[2] = {0, 0};
	u32 lp0_vec[2] = {0, 0};

	np = of_find_matching_node(NULL, matches);
	BUG_ON(!np);

	tegra_pclk = of_clk_get_by_name(np, "pclk");
	WARN_ON(IS_ERR(tegra_pclk));

	/* Grabbing the power management configurations */
	if (of_property_read_u32(np, "nvidia,suspend-mode", &prop)) {
		suspend_mode = TEGRA_SUSPEND_NONE;
	} else {
		switch (prop) {
		case 0:
			suspend_mode = TEGRA_SUSPEND_LP0;
			break;
		case 1:
			suspend_mode = TEGRA_SUSPEND_LP1;
			break;
		case 2:
			suspend_mode = TEGRA_SUSPEND_LP2;
			break;
		default:
			suspend_mode = TEGRA_SUSPEND_NONE;
			break;
		}
	}
	suspend_mode = tegra_pm_validate_suspend_mode(suspend_mode);

	if (of_property_read_u32(np, "nvidia,cpu-pwr-good-time", &prop))
		suspend_mode = TEGRA_SUSPEND_NONE;
	pmc_pm_data.cpu_good_time = prop;

	if (of_property_read_u32(np, "nvidia,cpu-pwr-off-time", &prop))
		suspend_mode = TEGRA_SUSPEND_NONE;
	pmc_pm_data.cpu_off_time = prop;

	if (of_property_read_u32_array(np, "nvidia,core-pwr-good-time",
			core_good_time, ARRAY_SIZE(core_good_time)))
		suspend_mode = TEGRA_SUSPEND_NONE;
	pmc_pm_data.core_osc_time = core_good_time[0];
	pmc_pm_data.core_pmu_time = core_good_time[1];

	if (of_property_read_u32(np, "nvidia,core-pwr-off-time",
				 &prop))
		suspend_mode = TEGRA_SUSPEND_NONE;
	pmc_pm_data.core_off_time = prop;

	pmc_pm_data.corereq_high = of_property_read_bool(np,
				"nvidia,core-power-req-active-high");

	pmc_pm_data.sysclkreq_high = of_property_read_bool(np,
				"nvidia,sys-clock-req-active-high");

	pmc_pm_data.combined_req = of_property_read_bool(np,
				"nvidia,combined-power-req");

	pmc_pm_data.cpu_pwr_good_en = of_property_read_bool(np,
				"nvidia,cpu-pwr-good-en");

	/*
	 * If LP0 suspend is requested, try to load the address of the resume
	 * code.  If the resume code isn't passed from the bootloader, an
	 * attempt to load it will be made at the first suspend.
	 */
	if (suspend_mode == TEGRA_SUSPEND_LP0 &&
	    !of_property_read_u32_array(np, "nvidia,lp0-vec", lp0_vec,
					ARRAY_SIZE(lp0_vec))) {
		pmc_pm_data.lp0_vec_phy_addr = lp0_vec[0];
		pmc_pm_data.lp0_vec_size = lp0_vec[1];

		if (!memblock_is_reserved(pmc_pm_data.lp0_vec_phy_addr)) {
			pr_info("Suspend mode LP0 requested, but init failed");
			pr_cont("-- disabling LP0\n");
			suspend_mode = TEGRA_SUSPEND_LP1;
		}
	}

	pmc_pm_data.suspend_mode = suspend_mode;

#ifdef CONFIG_PM_SLEEP
	tegra_lp0_wakeup.of_node = np;
	INIT_LIST_HEAD(&tegra_lp0_wakeup.wake_list);
#endif
}

void __init tegra_pmc_init_late(void)
{
	struct device_node *np;
	enum of_gpio_flags flags;
	int ret;

	np = of_find_matching_node(NULL, matches);
	if (!np)
		return;

	pmc_pm_data.reset_gpio = of_get_named_gpio_flags(np,
						"nvidia,reset-gpio", 0, &flags);
	if (gpio_is_valid(pmc_pm_data.reset_gpio)) {
		ret = gpio_request_one(pmc_pm_data.reset_gpio,
				       GPIOF_OUT_INIT_HIGH, "soc-warm-reset");
		if (ret) {
			pr_err("Failed to request reset GPIO: %d\n", ret);
			pmc_pm_data.reset_gpio = -1;
		}
		if (flags & OF_GPIO_ACTIVE_LOW)
			pmc_pm_data.reset_active_low = true;
	}
}
