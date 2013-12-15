/*
 * NVIDIA Tegra SoC device tree board support
 *
 * Copyright (C) 2011, 2013, NVIDIA Corporation
 * Copyright (C) 2010 Secret Lab Technologies, Ltd.
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clocksource.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/pda_power.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/usb/tegra_usb_phy.h>
#include <linux/clk-provider.h>
#include <linux/clk/tegra.h>
#include <linux/regulator/machine.h>
#include <linux/tegra-dvfs.h>
#include <linux/irqchip.h>
#include <linux/tegra-soc.h>
#include <linux/nvmap.h>
#include <linux/memblock.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include "apbio.h"
#include "board.h"
#include "board-panel.h"
#include "common.h"
#include "cpuidle.h"
#include "fuse.h"
#include "iomap.h"
#include "irq.h"
#include "pmc.h"
#include "pm.h"
#include "reset.h"
#include "sleep.h"

unsigned long tegra_fb_start;
unsigned long tegra_fb_size;
unsigned long tegra_fb2_start;
unsigned long tegra_fb2_size;
unsigned long tegra_carveout_start;
unsigned long tegra_carveout_size;

/*
 * Storage for debug-macro.S's state.
 *
 * This must be in .data not .bss so that it gets initialized each time the
 * kernel is loaded. The data is declared here rather than debug-macro.S so
 * that multiple inclusions of debug-macro.S point at the same data.
 */
u32 tegra_uart_config[4] = {
	/* Debug UART initialization required */
	1,
	/* Debug UART physical address */
	0,
	/* Debug UART virtual address */
	0,
	/* Scratch space for debug macro */
	0,
};

static void __init tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	int ret;
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;
	u32 aux_ctrl, cache_type;

	cache_type = readl(p + L2X0_CACHE_TYPE);
	aux_ctrl = (cache_type & 0x700) << (17-8);
	aux_ctrl |= 0x7C400001;

	ret = l2x0_of_init(aux_ctrl, 0x8200c3fe);
	if (!ret)
		l2x0_saved_regs_addr = virt_to_phys(&l2x0_saved_regs);
#endif
}

static void __init tegra_init_early(void)
{
	tegra_cpu_reset_handler_init();
	tegra_apb_io_init();
	tegra_init_fuse();
	tegra_init_cache();
	tegra_powergate_init();
	tegra_hotplug_init();
}

static void __init tegra_dt_init_irq(void)
{
	tegra_pmc_init_irq();
	tegra_init_irq();
	irqchip_init();
	tegra_legacy_irq_syscore_init();
}

#ifdef CONFIG_TEGRA_NVMAP
static struct nvmap_platform_carveout venice_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		.buddy_size	= 0, /* no buddy allocation for IRAM */
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0, /* Filled in by venice_panel_init() */
		.size		= 0, /* Filled in by venice_panel_init() */
		.buddy_size	= SZ_32K,
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0, /* Filled in by venice_panel_init() */
		.size		= 0, /* Filled in by venice_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data venice_nvmap_data = {
	.carveouts	= venice_carveouts,
	.nr_carveouts	= ARRAY_SIZE(venice_carveouts),
};

static struct platform_device venice_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &venice_nvmap_data,
	},
};
#endif

static void __init venice2_init(void)
{
#ifdef CONFIG_TEGRA_NVMAP
	int err;

	venice_carveouts[1].base = tegra_carveout_start;
	venice_carveouts[1].size = tegra_carveout_size;
	err = platform_device_register(&venice_nvmap_device);
	if (err)
		pr_err("nvmap device registration failed: %d\n", err);
#endif

	venice_panel_init();
}

static struct of_dev_auxdata tegra_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("pwm-backlight", 0, "pwm-backlight", &venice_bl_data),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", 0x54200000, "tegradc.0", &venice_disp1_pdata),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", 0x54240000, "tegradc.1", &venice_disp2_pdata),
	{}
};

static void __init tegra_dt_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device *parent = NULL;

	tegra_pmc_init();

	tegra_clocks_apply_init_table();

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		goto out;

	soc_dev_attr->family = kasprintf(GFP_KERNEL, "Tegra");
	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%d", tegra_revision);
	soc_dev_attr->soc_id = kasprintf(GFP_KERNEL, "%d", tegra_chip_id);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr->family);
		kfree(soc_dev_attr->revision);
		kfree(soc_dev_attr->soc_id);
		kfree(soc_dev_attr);
		goto out;
	}

	parent = soc_device_to_device(soc_dev);

	regulator_has_full_constraints();

	/*
	 * Finished with the static registrations now; fill in the missing
	 * devices
	 */
out:
#ifdef CONFIG_PM_SLEEP
	tegra_pmc_lp0_wakeup_init();
#endif

	of_platform_populate(NULL, of_default_bus_match_table,
				tegra_auxdata_lookup, parent);
	tegra_dvfs_init();

	if (of_machine_is_compatible("nvidia,venice2") ||
	    of_machine_is_compatible("nvidia,norrin"))
		venice2_init();
}

static void __init trimslice_init(void)
{
#ifdef CONFIG_TEGRA_PCI
	int ret;

	ret = tegra_pcie_init(true, true);
	if (ret)
		pr_err("tegra_pci_init() failed: %d\n", ret);
#endif
}

static void __init harmony_init(void)
{
#ifdef CONFIG_TEGRA_PCI
	int ret;

	ret = harmony_pcie_init();
	if (ret)
		pr_err("harmony_pcie_init() failed: %d\n", ret);
#endif
}

static void __init tegra_dt_init_time(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
}

static void __init paz00_init(void)
{
	if (IS_ENABLED(CONFIG_ARCH_TEGRA_2x_SOC))
		tegra_paz00_wifikill_init();
}

static struct {
	char *machine;
	void (*init)(void);
} board_init_funcs[] = {
	{ "compulab,trimslice", trimslice_init },
	{ "nvidia,harmony", harmony_init },
	{ "compal,paz00", paz00_init },
};

static void __init tegra_dt_init_late(void)
{
	int i;

	tegra_cpufreq_init();
	tegra_init_suspend();
	tegra_cpuidle_init();
	tegra_powergate_debugfs_init();
	tegra_pmc_init_late();
	tegra_cpuquiet_init();

	for (i = 0; i < ARRAY_SIZE(board_init_funcs); i++) {
		if (of_machine_is_compatible(board_init_funcs[i].machine)) {
			board_init_funcs[i].init();
			break;
		}
	}
}

static void __init tegra_reserve(unsigned long carveout_size,
				 unsigned long fb_size,
				 unsigned long fb2_size)
{
	if (carveout_size) {
		tegra_carveout_start = memblock_end_of_DRAM() - carveout_size;
		if (memblock_remove(tegra_carveout_start, carveout_size)) {
			pr_err("Failed to remove carveout %08lx@%08lx from memory map\n",
				carveout_size, tegra_carveout_start);
			tegra_carveout_start = 0;
			tegra_carveout_size = 0;
		} else
			tegra_carveout_size = carveout_size;
	}

	if (fb2_size) {
		tegra_fb2_start = memblock_end_of_DRAM() - fb2_size;
		if (memblock_remove(tegra_fb2_start, fb2_size)) {
			pr_err("Failed to remove second framebuffer %08lx@%08lx from memory map\n",
				fb2_size, tegra_fb2_start);
			tegra_fb2_start = 0;
			tegra_fb2_size = 0;
		} else
			tegra_fb2_size = fb2_size;
	}

	if (fb_size) {
		tegra_fb_start = memblock_end_of_DRAM() - fb_size;
		if (memblock_remove(tegra_fb_start, fb_size)) {
			pr_err("Failed to remove framebuffer %08lx@%08lx from memory map\n",
				fb_size, tegra_fb_start);
			tegra_fb_start = 0;
			tegra_fb_size = 0;
		} else
			tegra_fb_size = fb_size;
	}

	pr_info("Tegra reserved memory:\n"
		"Framebuffer:	    %08lx - %08lx\n"
		"2nd Framebuffer:	%08lx - %08lx\n"
		"Carveout:	       %08lx - %08lx\n",
		tegra_fb_start,
		tegra_fb_size ?
			tegra_fb_start + tegra_fb_size - 1 : 0,
		tegra_fb2_start,
		tegra_fb2_size ?
			tegra_fb2_start + tegra_fb2_size - 1 : 0,
		tegra_carveout_start,
		tegra_carveout_size ?
			tegra_carveout_start + tegra_carveout_size - 1 : 0);
}

static void __init tegra_board_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* TODO: Need to find a way to distiguish Venice 2 & Norrin */
	/* Reserve framebuffer, size: 2560*1700*4*2 */
	tegra_reserve(0, SZ_32M + SZ_2M, SZ_16M);
#else
	tegra_reserve(SZ_128M, SZ_16M + SZ_2M, SZ_4M);
#endif
}

static const char * const tegra_dt_board_compat[] = {
	"nvidia,tegra124",
	"nvidia,tegra114",
	"nvidia,tegra30",
	"nvidia,tegra20",
	NULL
};

DT_MACHINE_START(TEGRA_DT, "NVIDIA Tegra SoC (Flattened Device Tree)")
	.map_io		= tegra_map_common_io,
	.smp		= smp_ops(tegra_smp_ops),
	.reserve	= tegra_board_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_dt_init_irq,
	.init_time	= tegra_dt_init_time,
	.init_machine	= tegra_dt_init,
	.init_late	= tegra_dt_init_late,
	.restart	= tegra_pmc_restart,
	.dt_compat	= tegra_dt_board_compat,
MACHINE_END
