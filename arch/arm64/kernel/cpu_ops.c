/*
 * CPU kernel entry/exit control
 *
 * Copyright (C) 2013 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/acpi.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/string.h>
#include <asm/acpi.h>
#include <asm/cpu_ops.h>
#include <asm/smp_plat.h>

const struct cpu_operations *cpu_ops[NR_CPUS];
extern struct of_cpu_method __cpu_method_of_table[];
static const struct of_cpu_method __cpu_method_of_table_sentinel
	__used __section(__cpu_method_of_table_end);

static const struct cpu_operations * __init cpu_get_ops(const char *name)
{
	struct of_cpu_method *m = __cpu_method_of_table;

	for (; m->method; m++)
		if (!strcmp(m->method, name))
			return m->ops;

	return NULL;
}

static const char *__init cpu_read_enable_method(int cpu)
{
	const char *enable_method;

	if (acpi_disabled) {
		struct device_node *dn = of_get_cpu_node(cpu, NULL);

		if (!dn) {
			if (!cpu)
				pr_err("Failed to find device node for boot cpu\n");
			return NULL;
		}

		enable_method = of_get_property(dn, "enable-method", NULL);
		if (!enable_method) {
			/*
			 * The boot CPU may not have an enable method (e.g.
			 * when spin-table is used for secondaries).
			 * Don't warn spuriously.
			 */
			if (cpu != 0)
				pr_err("%s: missing enable-method property\n",
					dn->full_name);
		}
		of_node_put(dn);
	} else {
		enable_method = acpi_get_enable_method(cpu);
		if (!enable_method) {
			/*
			 * In ACPI systems the boot CPU does not require
			 * checking the enable method since for some
			 * boot protocol (ie parking protocol) it need not
			 * be initialized. Don't warn spuriously.
			 */
			if (cpu != 0)
				pr_err("Unsupported ACPI enable-method\n");
		}
	}

	return enable_method;
}
/*
 * Read a cpu's enable method and record it in cpu_ops.
 */
int __init cpu_read_ops(int cpu)
{
	const char *enable_method = cpu_read_enable_method(cpu);

	if (!enable_method)
		return -ENODEV;

	cpu_ops[cpu] = cpu_get_ops(enable_method);
	if (!cpu_ops[cpu]) {
		pr_warn("Unsupported enable-method: %s\n", enable_method);
		return -EOPNOTSUPP;
	}

	return 0;
}
