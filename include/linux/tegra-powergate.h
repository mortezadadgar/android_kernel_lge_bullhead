/*
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#ifndef _MACH_TEGRA_POWERGATE_H_
#define _MACH_TEGRA_POWERGATE_H_

struct clk;

#define TEGRA_POWERGATE_CPU	0
#define TEGRA_POWERGATE_3D	1
#define TEGRA_POWERGATE_VENC	2
#define TEGRA_POWERGATE_PCIE	3
#define TEGRA_POWERGATE_VDEC	4
#define TEGRA_POWERGATE_L2	5
#define TEGRA_POWERGATE_MPE	6
#define TEGRA_POWERGATE_HEG	7
#define TEGRA_POWERGATE_SATA	8
#define TEGRA_POWERGATE_CPU1	9
#define TEGRA_POWERGATE_CPU2	10
#define TEGRA_POWERGATE_CPU3	11
#define TEGRA_POWERGATE_CELP	12
#define TEGRA_POWERGATE_3D1	13
#define TEGRA_POWERGATE_CPU0	14
#define TEGRA_POWERGATE_C0NC	15
#define TEGRA_POWERGATE_C1NC	16
#define TEGRA_POWERGATE_SOR	17
#define TEGRA_POWERGATE_DIS	18
#define TEGRA_POWERGATE_DISB	19
#define TEGRA_POWERGATE_XUSBA	20
#define TEGRA_POWERGATE_XUSBB	21
#define TEGRA_POWERGATE_XUSBC	22
#define TEGRA_POWERGATE_VIC	23

#define TEGRA_POWERGATE_3D0	TEGRA_POWERGATE_3D

#define MAX_CLK_EN_NUM	9

/**
 * struct partition_clk_info - clock infomation needed by power partition
 *
 * @clk_name: name of the clock
 * @clk_ptr: pointer to the clock
 */
struct partition_clk_info {
	const char *clk_name;
	struct clk *clk_ptr;
};

/**
 * struct powergate_partition_info - power partition information
 *
 * @name: name of the power domain
 * @clk_info: clock information needed by the power partition
 */
struct powergate_partition_info {
	const char *name;
	struct partition_clk_info clk_info[MAX_CLK_EN_NUM];
};

/**
 * struct powergate_ops - callback of SoC specific power operations
 *
 * get_powergate_lock: retrieve the lock for power partition control
 * get_powergate_domain_name: get the domain name
 * powergate_partition: shut off parition , only needed by Tegra114 or later
 * unpowergate_partition: power on partition, only needed by Tegra114 or later
 */
struct powergate_ops {
	const char *(*get_powergate_domain_name)(int id);
	int (*powergate_partition)(int id);
	int (*unpowergate_partition)(int id);
};

/**
 * struct powergate - SoC powergate manager
 *
 * @soc_name: name of the SoC
 * @num_powerdomains: number of power domains supported in this SoC
 * @num_cpu_domains: number of cpu domains supported in this SoC
 * @cpu_domain_map: map from the physical cpu to the cpu domain id
 * @ops: SoC specific operations for power domain
 */
struct powergate {
	const char *soc_name;

	int num_powerdomains;
	int num_cpu_domains;
	const u8 *cpu_domain_map;

	struct powergate_ops *ops;
};

#define TEGRA_IO_RAIL_CSIA	0
#define TEGRA_IO_RAIL_CSIB	1
#define TEGRA_IO_RAIL_DSI	2
#define TEGRA_IO_RAIL_MIPI_BIAS	3
#define TEGRA_IO_RAIL_PEX_BIAS	4
#define TEGRA_IO_RAIL_PEX_CLK1	5
#define TEGRA_IO_RAIL_PEX_CLK2	6
#define TEGRA_IO_RAIL_USB0	9
#define TEGRA_IO_RAIL_USB1	10
#define TEGRA_IO_RAIL_USB2	11
#define TEGRA_IO_RAIL_USB_BIAS	12
#define TEGRA_IO_RAIL_NAND	13
#define TEGRA_IO_RAIL_UART	14
#define TEGRA_IO_RAIL_BB	15
#define TEGRA_IO_RAIL_AUDIO	17
#define TEGRA_IO_RAIL_HSIC	19
#define TEGRA_IO_RAIL_COMP	22
#define TEGRA_IO_RAIL_HDMI	28
#define TEGRA_IO_RAIL_PEX_CNTRL	32
#define TEGRA_IO_RAIL_SDMMC1	33
#define TEGRA_IO_RAIL_SDMMC3	34
#define TEGRA_IO_RAIL_SDMMC4	35
#define TEGRA_IO_RAIL_CAM	36
#define TEGRA_IO_RAIL_RES	37
#define TEGRA_IO_RAIL_HV	38
#define TEGRA_IO_RAIL_DSIB	39
#define TEGRA_IO_RAIL_DSIC	40
#define TEGRA_IO_RAIL_DSID	41
#define TEGRA_IO_RAIL_CSIE	44
#define TEGRA_IO_RAIL_LVDS	57
#define TEGRA_IO_RAIL_SYS_DDC	58

int tegra_powergate_is_powered(int id);
int tegra_powergate_power_on(int id);
int tegra_powergate_power_off(int id);
int tegra_powergate_remove_clamping(int id);

/* Must be called with clk disabled, and returns with clk enabled */
int tegra_powergate_sequence_power_up(int id, struct clk *clk);

int tegra_powergate_partition(int id);
int tegra_unpowergate_partition(int id);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
struct powergate * __init tegra20_powergate_init(void);
#else
static inline struct powergate * __init tegra20_powergate_init(void)
{ return NULL; }
#endif

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
struct powergate * __init tegra30_powergate_init(void);
#else
static inline struct powergate * __init tegra30_powergate_init(void)
{ return NULL; }
#endif

#ifdef CONFIG_ARCH_TEGRA_114_SOC
struct powergate * __init tegra114_powergate_init(void);
#else
static inline struct powergate * __init tegra114_powergate_init(void)
{ return NULL; }
#endif

#ifdef CONFIG_ARCH_TEGRA_124_SOC
struct powergate * __init tegra124_powergate_init(void);
#else
static inline struct powergate * __init tegra124_powergate_init(void)
{ return NULL; }
#endif

int tegra_io_rail_power_on(int id);
int tegra_io_rail_power_off(int id);

#endif /* _MACH_TEGRA_POWERGATE_H_ */
