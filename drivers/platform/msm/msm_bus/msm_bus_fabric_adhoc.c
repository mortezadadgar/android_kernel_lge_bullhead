/* Copyright (c) 2014-2015, Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <soc/qcom/rpm-smd.h>
#include <trace/events/trace_msm_bus.h>
#include "msm_bus_core.h"
#include "msm_bus_adhoc.h"
#include "msm_bus_noc.h"
#include "msm_bus_bimc.h"

ssize_t vrail_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct msm_bus_node_info_type *node_info = NULL;
	struct msm_bus_node_device_type *bus_node = NULL;

	bus_node = dev->platform_data;
	if (!bus_node)
		return -EINVAL;
	node_info = bus_node->node_info;

	return snprintf(buf, PAGE_SIZE, "%u", node_info->vrail_comp);
}

ssize_t vrail_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct msm_bus_node_info_type *node_info = NULL;
	struct msm_bus_node_device_type *bus_node = NULL;
	int ret = 0;

	bus_node = dev->platform_data;
	if (!bus_node)
		return -EINVAL;
	node_info = bus_node->node_info;

	ret = sscanf(buf, "%u", &node_info->vrail_comp);
	if (ret != 1)
		return -EINVAL;
	return count;
}

DEVICE_ATTR(vrail, 0600, vrail_show, vrail_store);

ssize_t bw_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct msm_bus_node_info_type *node_info = NULL;
	struct msm_bus_node_device_type *bus_node = NULL;
	int i;
	int off = 0;

	bus_node = to_msm_bus_node(dev);
	if (!bus_node)
		return -EINVAL;

	node_info = bus_node->node_info;

	for (i = 0; i < bus_node->num_lnodes; i++) {
		if (!bus_node->lnode_list[i].in_use)
			continue;
		off += scnprintf((buf + off), PAGE_SIZE,
		"[%d]:%s:Act_IB %llu Act_AB %llu Slp_IB %llu Slp_AB %llu\n",
			i, bus_node->lnode_list[i].cl_name,
			bus_node->lnode_list[i].lnode_ib[ACTIVE_CTX],
			bus_node->lnode_list[i].lnode_ab[ACTIVE_CTX],
			bus_node->lnode_list[i].lnode_ib[DUAL_CTX],
			bus_node->lnode_list[i].lnode_ab[DUAL_CTX]);
		trace_printk(
		"[%d]:%s:Act_IB %llu Act_AB %llu Slp_IB %llu Slp_AB %llu\n",
			i, bus_node->lnode_list[i].cl_name,
			bus_node->lnode_list[i].lnode_ib[ACTIVE_CTX],
			bus_node->lnode_list[i].lnode_ab[ACTIVE_CTX],
			bus_node->lnode_list[i].lnode_ib[DUAL_CTX],
			bus_node->lnode_list[i].lnode_ab[DUAL_CTX]);
	}
	off += scnprintf((buf + off), PAGE_SIZE,
	"Max_Act_IB %llu Sum_Act_AB %llu\nMax_Slp_IB %llu Sum_Slp_AB %llu\n",
		bus_node->node_bw[ACTIVE_CTX].max_ib,
		bus_node->node_bw[ACTIVE_CTX].sum_ab,
		bus_node->node_bw[DUAL_CTX].max_ib,
		bus_node->node_bw[DUAL_CTX].sum_ab);
	trace_printk(
	"Max_Act_IB %llu Sum_Act_AB %llu\nMax_Slp_IB %llu Sum_Slp_AB %llu\n",
		bus_node->node_bw[ACTIVE_CTX].max_ib,
		bus_node->node_bw[ACTIVE_CTX].sum_ab,
		bus_node->node_bw[DUAL_CTX].max_ib,
		bus_node->node_bw[DUAL_CTX].sum_ab);
	return off;
}

ssize_t bw_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	return count;
}

DEVICE_ATTR(bw, 0600, bw_show, bw_store);

struct static_rules_type {
	int num_rules;
	struct bus_rule_type *rules;
};

static struct static_rules_type static_rules;

static int enable_nodeclk(struct nodeclk *nclk)
{
	int ret = 0;

	if (!nclk->enable) {
		ret = clk_prepare_enable(nclk->clk);

		if (ret) {
			MSM_BUS_ERR("%s: failed to enable clk ", __func__);
			nclk->enable = false;
		} else
			nclk->enable = true;
	}
	return ret;
}

static int disable_nodeclk(struct nodeclk *nclk)
{
	int ret = 0;

	if (nclk->enable) {
		clk_disable_unprepare(nclk->clk);
		nclk->enable = false;
	}
	return ret;
}

static int setrate_nodeclk(struct nodeclk *nclk, long rate)
{
	int ret = 0;

	ret = clk_set_rate(nclk->clk, rate);

	if (ret)
		MSM_BUS_ERR("%s: failed to setrate clk", __func__);
	return ret;
}

static int send_rpm_msg(struct msm_bus_node_device_type *ndev, int ctx)
{
	int ret = 0;
	int rsc_type;
	struct msm_rpm_kvp rpm_kvp;
	int rpm_ctx;

	if (!ndev) {
		MSM_BUS_ERR("%s: Error getting node info.", __func__);
		ret = -ENODEV;
		goto exit_send_rpm_msg;
	}

	rpm_kvp.length = sizeof(uint64_t);
	rpm_kvp.key = RPM_MASTER_FIELD_BW;

	if (ctx == DUAL_CTX)
		rpm_ctx = MSM_RPM_CTX_SLEEP_SET;
	else
		rpm_ctx = MSM_RPM_CTX_ACTIVE_SET;

	rpm_kvp.data = (uint8_t *)&ndev->node_bw[ctx].sum_ab;

	if (ndev->node_info->mas_rpm_id != -1) {
		rsc_type = RPM_BUS_MASTER_REQ;
		ret = msm_rpm_send_message(rpm_ctx, rsc_type,
			ndev->node_info->mas_rpm_id, &rpm_kvp, 1);
		if (ret) {
			MSM_BUS_ERR("%s: Failed to send RPM message:",
					__func__);
			MSM_BUS_ERR("%s:Node Id %d RPM id %d",
			__func__, ndev->node_info->id,
				 ndev->node_info->mas_rpm_id);
			goto exit_send_rpm_msg;
		}
	}

	if (ndev->node_info->slv_rpm_id != -1) {
		rsc_type = RPM_BUS_SLAVE_REQ;
		ret = msm_rpm_send_message(rpm_ctx, rsc_type,
			ndev->node_info->slv_rpm_id, &rpm_kvp, 1);
		if (ret) {
			MSM_BUS_ERR("%s: Failed to send RPM message:",
						__func__);
			MSM_BUS_ERR("%s: Node Id %d RPM id %d",
			__func__, ndev->node_info->id,
				ndev->node_info->slv_rpm_id);
			goto exit_send_rpm_msg;
		}
	}
exit_send_rpm_msg:
	return ret;
}

static int flush_bw_data(struct msm_bus_node_device_type *node_info, int ctx)
{
	int ret = 0;

	if (!node_info) {
		MSM_BUS_ERR("%s: Unable to find bus device for device %d",
			__func__, node_info->node_info->id);
		ret = -ENODEV;
		goto exit_flush_bw_data;
	}

	if (node_info->node_bw[ctx].last_sum_ab !=
				node_info->node_bw[ctx].sum_ab) {
		if (node_info->ap_owned) {
			struct msm_bus_node_device_type *bus_device =
			to_msm_bus_node(node_info->node_info->bus_device);
			struct msm_bus_fab_device_type *fabdev =
							bus_device->fabdev;

			/*
			 * For AP owned ports, only care about the Active
			 * context bandwidth.
			 */
			if (fabdev && (ctx == ACTIVE_CTX) &&
				fabdev->noc_ops.update_bw_reg &&
				fabdev->noc_ops.update_bw_reg
					(node_info->node_info->qos_params.mode))
				ret = fabdev->noc_ops.set_bw(node_info,
							fabdev->qos_base,
							fabdev->base_offset,
							fabdev->qos_off,
							fabdev->qos_freq);
		} else {
			ret = send_rpm_msg(node_info, ctx);

			if (ret)
				MSM_BUS_ERR("%s: Failed to send RPM msg for%d",
				__func__, node_info->node_info->id);
		}
		node_info->node_bw[ctx].last_sum_ab =
					node_info->node_bw[ctx].sum_ab;
	}

exit_flush_bw_data:
	return ret;

}

static int flush_clk_data(struct msm_bus_node_device_type *node, int ctx)
{
	struct nodeclk *nodeclk = NULL;
	int ret = 0;

	if (!node) {
		MSM_BUS_ERR("Unable to find bus device");
		ret = -ENODEV;
		goto exit_flush_clk_data;
	}

	nodeclk = &node->clk[ctx];

	if (IS_ERR_OR_NULL(nodeclk) || IS_ERR_OR_NULL(nodeclk->clk))
		goto exit_flush_clk_data;

	if (nodeclk->rate != node->node_bw[ctx].cur_clk_hz) {
		long rounded_rate;

		nodeclk->rate = node->node_bw[ctx].cur_clk_hz;
		nodeclk->dirty = true;

		if (nodeclk->rate) {
			rounded_rate = clk_round_rate(nodeclk->clk,
							nodeclk->rate);
			ret = setrate_nodeclk(nodeclk, rounded_rate);

			if (ret) {
				MSM_BUS_ERR("%s: Failed to set_rate %lu for %d",
					__func__, rounded_rate,
						node->node_info->id);
				ret = -ENODEV;
				goto exit_flush_clk_data;
			}

			ret = enable_nodeclk(nodeclk);

			if ((node->node_info->is_fab_dev) &&
				!IS_ERR_OR_NULL(node->qos_clk.clk))
					ret = enable_nodeclk(&node->qos_clk);
		} else {
			if ((node->node_info->is_fab_dev) &&
				!IS_ERR_OR_NULL(node->qos_clk.clk))
					ret = disable_nodeclk(&node->qos_clk);

			ret = disable_nodeclk(nodeclk);
		}

		if (ret) {
			MSM_BUS_ERR("%s: Failed to enable for %d", __func__,
						node->node_info->id);
			ret = -ENODEV;
			goto exit_flush_clk_data;
		}
		trace_bus_agg_clk(node->node_info->id, ctx, nodeclk->rate);
		MSM_BUS_DBG("%s: Updated %d clk to %llu", __func__,
				node->node_info->id, nodeclk->rate);
	}
exit_flush_clk_data:
	/* Reset the aggregated clock rate for fab devices*/
	if (node && node->node_info->is_fab_dev)
		node->node_bw[ctx].cur_clk_hz = 0;

	if (nodeclk)
		nodeclk->dirty = 0;
	return ret;
}

static int msm_bus_agg_fab_clks(struct msm_bus_node_device_type *bus_dev)
{
	int ret = 0;
	struct msm_bus_node_device_type *node;
	int ctx;

	list_for_each_entry(node, &bus_dev->devlist, dev_link) {
		for (ctx = 0; ctx < NUM_CTX; ctx++) {
			if (node->node_bw[ctx].cur_clk_hz >=
					bus_dev->node_bw[ctx].cur_clk_hz)
				bus_dev->node_bw[ctx].cur_clk_hz =
						node->node_bw[ctx].cur_clk_hz;
		}
	}
	return ret;
}

int msm_bus_commit_data(struct list_head *clist)
{
	int ret = 0;
	int ctx;
	struct msm_bus_node_device_type *node;
	struct msm_bus_node_device_type *node_tmp;

	list_for_each_entry(node, clist, link) {
		/* Aggregate the bus clocks */
		if (node->node_info->is_fab_dev)
			msm_bus_agg_fab_clks(node);
	}

	list_for_each_entry_safe(node, node_tmp, clist, link) {
		for (ctx = 0; ctx < NUM_CTX; ctx++) {
			ret = flush_clk_data(node, ctx);
			if (ret)
				MSM_BUS_ERR("%s: Err flushing clk data for:%d",
						__func__, node->node_info->id);
			ret = flush_bw_data(node, ctx);
			if (ret)
				MSM_BUS_ERR("%s: Error flushing bw data for %d",
					__func__, node->node_info->id);
		}
		node->dirty = false;
		list_del_init(&node->link);
	}
	return ret;
}

void *msm_bus_realloc_devmem(struct device *dev, void *p, size_t old_size,
					size_t new_size, gfp_t flags)
{
	void *ret;
	size_t copy_size = old_size;

	if (!new_size) {
		devm_kfree(dev, p);
		return ZERO_SIZE_PTR;
	}

	if (new_size < old_size)
		copy_size = new_size;

	ret = devm_kzalloc(dev, new_size, flags);
	if (!ret) {
		MSM_BUS_ERR("%s: Error Reallocating memory", __func__);
		goto exit_realloc_devmem;
	}

	memcpy(ret, p, copy_size);
	devm_kfree(dev, p);
exit_realloc_devmem:
	return ret;
}

static void msm_bus_fab_init_noc_ops(struct msm_bus_node_device_type *bus_dev)
{
	switch (bus_dev->fabdev->bus_type) {
	case MSM_BUS_NOC:
		msm_bus_noc_set_ops(bus_dev);
		break;
	case MSM_BUS_BIMC:
		msm_bus_bimc_set_ops(bus_dev);
		break;
	default:
		MSM_BUS_ERR("%s: Invalid Bus type", __func__);
	}
}

static int msm_bus_qos_disable_clk(struct msm_bus_node_device_type *node,
				int disable_bus_qos_clk)
{
	struct msm_bus_node_device_type *bus_node = NULL;
	int ret = 0;

	if (!node) {
		ret = -ENXIO;
		goto exit_disable_qos_clk;
	}
	bus_node = to_msm_bus_node(node->node_info->bus_device);

	if (!bus_node) {
		ret = -ENXIO;
		goto exit_disable_qos_clk;
	}

	if (disable_bus_qos_clk)
		ret = disable_nodeclk(&bus_node->clk[DUAL_CTX]);

	if (ret) {
		MSM_BUS_ERR("%s: Failed to disable bus clk, node %d",
			__func__, node->node_info->id);
		goto exit_disable_qos_clk;
	}

	if (!IS_ERR_OR_NULL(node->qos_clk.clk)) {
		ret = disable_nodeclk(&node->qos_clk);

		if (ret) {
			MSM_BUS_ERR("%s: Failed to disable mas qos clk,node %d",
				__func__, node->node_info->id);
			goto exit_disable_qos_clk;
		}
	}

exit_disable_qos_clk:
	return ret;
}

static int msm_bus_qos_enable_clk(struct msm_bus_node_device_type *node)
{
	struct msm_bus_node_device_type *bus_node = NULL;
	long rounded_rate;
	int ret = 0;
	int bus_qos_enabled = 0;

	if (!node) {
		ret = -ENXIO;
		goto exit_enable_qos_clk;
	}
	bus_node = to_msm_bus_node(node->node_info->bus_device);

	if (!bus_node) {
		ret = -ENXIO;
		goto exit_enable_qos_clk;
	}

	/* Check if the bus clk is already set before trying to set it
	 * Do this only during
	 *	a. Bootup
	 *	b. Only for bus clks
	 **/
	if (!clk_get_rate(bus_node->clk[DUAL_CTX].clk)) {
		rounded_rate = clk_round_rate(bus_node->clk[DUAL_CTX].clk, 1);
		ret = setrate_nodeclk(&bus_node->clk[DUAL_CTX], rounded_rate);
		if (ret) {
			MSM_BUS_ERR("%s: Failed to set bus clk, node %d",
				__func__, node->node_info->id);
			goto exit_enable_qos_clk;
		}
	}

	ret = enable_nodeclk(&bus_node->clk[DUAL_CTX]);
	if (ret) {
		MSM_BUS_ERR("%s: Failed to enable bus clk, node %d",
			__func__, node->node_info->id);
		goto exit_enable_qos_clk;
	}
	bus_qos_enabled = 1;

	if (!IS_ERR_OR_NULL(bus_node->qos_clk.clk)) {
		ret = enable_nodeclk(&bus_node->qos_clk);
		if (ret) {
			MSM_BUS_ERR("%s: Failed to enable bus QOS clk, node %d",
				__func__, node->node_info->id);
			goto exit_enable_qos_clk;
		}
	}

	if (!IS_ERR_OR_NULL(node->qos_clk.clk)) {
		rounded_rate = clk_round_rate(node->qos_clk.clk, 1);
		ret = setrate_nodeclk(&node->qos_clk, rounded_rate);
		if (ret) {
			MSM_BUS_ERR("%s: Failed to enable mas qos clk, node %d",
				__func__, node->node_info->id);
			goto exit_enable_qos_clk;
		}

		ret = enable_nodeclk(&node->qos_clk);
		if (ret) {
			MSM_BUS_ERR("Err enable mas qos clk, node %d ret %d",
				node->node_info->id, ret);
			goto exit_enable_qos_clk;
		}
	}
	ret = bus_qos_enabled;

exit_enable_qos_clk:
	return ret;
}

int msm_bus_enable_limiter(struct msm_bus_node_device_type *node_dev,
				int enable, uint64_t lim_bw)
{
	int ret = 0;
	struct msm_bus_node_device_type *bus_node_dev;

	if (!node_dev) {
		MSM_BUS_ERR("No device specified");
		ret = -ENXIO;
		goto exit_enable_limiter;
	}

	if (!node_dev->ap_owned) {
		MSM_BUS_ERR("Device is not AP owned %d.",
						node_dev->node_info->id);
		ret = -ENXIO;
		goto exit_enable_limiter;
	}

	bus_node_dev = to_msm_bus_node(node_dev->node_info->bus_device);
	if (!bus_node_dev) {
		MSM_BUS_ERR("Unable to get bus device infofor %d",
			node_dev->node_info->id);
		ret = -ENXIO;
		goto exit_enable_limiter;
	}
	if (bus_node_dev->fabdev &&
		bus_node_dev->fabdev->noc_ops.limit_mport) {
		if (ret < 0) {
			MSM_BUS_ERR("Can't Enable QoS clk %d",
				node_dev->node_info->id);
			goto exit_enable_limiter;
		}
		bus_node_dev->fabdev->noc_ops.limit_mport(
				node_dev,
				bus_node_dev->fabdev->qos_base,
				bus_node_dev->fabdev->base_offset,
				bus_node_dev->fabdev->qos_off,
				bus_node_dev->fabdev->qos_freq,
				enable, lim_bw);
	}

exit_enable_limiter:
	return ret;
}

static int msm_bus_dev_init_qos(struct device *dev, void *data)
{
	int ret = 0;
	struct msm_bus_node_device_type *node_dev = NULL;

	node_dev = to_msm_bus_node(dev);
	if (!node_dev) {
		MSM_BUS_ERR("%s: Unable to get node device info" , __func__);
		ret = -ENXIO;
		goto exit_init_qos;
	}

	MSM_BUS_DBG("Device = %d", node_dev->node_info->id);

	if (node_dev->ap_owned) {
		struct msm_bus_node_device_type *bus_node_info;

		bus_node_info =
			to_msm_bus_node(node_dev->node_info->bus_device);

		if (!bus_node_info) {
			MSM_BUS_ERR("%s: Unable to get bus device infofor %d",
				__func__,
				node_dev->node_info->id);
			ret = -ENXIO;
			goto exit_init_qos;
		}

		if (bus_node_info->fabdev &&
			bus_node_info->fabdev->noc_ops.qos_init) {
			int ret = 0;

			if (node_dev->ap_owned &&
				(node_dev->node_info->qos_params.mode) != -1) {

				if (bus_node_info->fabdev->bypass_qos_prg)
					goto exit_init_qos;

				ret = msm_bus_qos_enable_clk(node_dev);
				if (ret < 0) {
					MSM_BUS_ERR("Can't Enable QoS clk %d",
					node_dev->node_info->id);
					goto exit_init_qos;
				}

				bus_node_info->fabdev->noc_ops.qos_init(
					node_dev,
					bus_node_info->fabdev->qos_base,
					bus_node_info->fabdev->base_offset,
					bus_node_info->fabdev->qos_off,
					bus_node_info->fabdev->qos_freq);
				ret = msm_bus_qos_disable_clk(node_dev, ret);
			}
		} else
			MSM_BUS_ERR("%s: Skipping QOS init for %d",
				__func__, node_dev->node_info->id);
	}
exit_init_qos:
	return ret;
}

static int msm_bus_fabric_init(struct device *dev,
			struct msm_bus_node_device_type *pdata)
{
	struct msm_bus_fab_device_type *fabdev;
	struct msm_bus_node_device_type *node_dev = NULL;
	int ret = 0;

	node_dev = to_msm_bus_node(dev);
	if (!node_dev) {
		MSM_BUS_ERR("%s: Unable to get bus device info" , __func__);
		ret = -ENXIO;
		goto exit_fabric_init;
	}

	if (node_dev->node_info->virt_dev) {
		MSM_BUS_ERR("%s: Skip Fab init for virtual device %d", __func__,
						node_dev->node_info->id);
		goto exit_fabric_init;
	}

	fabdev = devm_kzalloc(dev, sizeof(struct msm_bus_fab_device_type),
								GFP_KERNEL);
	if (!fabdev) {
		MSM_BUS_ERR("Fabric alloc failed\n");
		ret = -ENOMEM;
		goto exit_fabric_init;
	}

	node_dev->fabdev = fabdev;
	fabdev->pqos_base = pdata->fabdev->pqos_base;
	fabdev->qos_range = pdata->fabdev->qos_range;
	fabdev->base_offset = pdata->fabdev->base_offset;
	fabdev->qos_off = pdata->fabdev->qos_off;
	fabdev->qos_freq = pdata->fabdev->qos_freq;
	fabdev->bus_type = pdata->fabdev->bus_type;
	fabdev->bypass_qos_prg = pdata->fabdev->bypass_qos_prg;
	fabdev->util_fact = pdata->fabdev->util_fact;
	fabdev->vrail_comp = pdata->fabdev->vrail_comp;
	msm_bus_fab_init_noc_ops(node_dev);

	fabdev->qos_base = devm_ioremap(dev,
				fabdev->pqos_base, fabdev->qos_range);
	if (!fabdev->qos_base) {
		MSM_BUS_ERR("%s: Error remapping address 0x%zx :bus device %d",
			__func__,
			 (size_t)fabdev->pqos_base, node_dev->node_info->id);
		ret = -ENOMEM;
		goto exit_fabric_init;
	}

exit_fabric_init:
	return ret;
}

static int msm_bus_init_clk(struct device *bus_dev,
				struct msm_bus_node_device_type *pdata)
{
	unsigned int ctx;
	int ret = 0;
	struct msm_bus_node_device_type *node_dev = to_msm_bus_node(bus_dev);

	for (ctx = 0; ctx < NUM_CTX; ctx++) {
		if (!IS_ERR_OR_NULL(pdata->clk[ctx].clk)) {
			node_dev->clk[ctx].clk = pdata->clk[ctx].clk;
			node_dev->clk[ctx].enable = false;
			node_dev->clk[ctx].dirty = false;
			MSM_BUS_ERR("%s: Valid node clk node %d ctx %d",
				__func__, node_dev->node_info->id, ctx);
		}
	}

	if (!IS_ERR_OR_NULL(pdata->qos_clk.clk)) {
		node_dev->qos_clk.clk = pdata->qos_clk.clk;
		node_dev->qos_clk.enable = false;
		MSM_BUS_ERR("%s: Valid Iface clk node %d", __func__,
						node_dev->node_info->id);
	}

	return ret;
}

static int msm_bus_copy_node_info(struct msm_bus_node_device_type *pdata,
				struct device *bus_dev)
{
	int ret = 0;
	struct msm_bus_node_info_type *node_info = NULL;
	struct msm_bus_node_info_type *pdata_node_info = NULL;
	struct msm_bus_node_device_type *bus_node = NULL;

	bus_node = to_msm_bus_node(bus_dev);

	if (!bus_node || !pdata) {
		ret = -ENXIO;
		MSM_BUS_ERR("%s: Invalid pointers pdata %p, bus_node %p",
			__func__, pdata, bus_node);
		goto exit_copy_node_info;
	}

	node_info = bus_node->node_info;
	pdata_node_info = pdata->node_info;

	node_info->name = pdata_node_info->name;
	node_info->id =  pdata_node_info->id;
	node_info->bus_device_id = pdata_node_info->bus_device_id;
	node_info->mas_rpm_id = pdata_node_info->mas_rpm_id;
	node_info->slv_rpm_id = pdata_node_info->slv_rpm_id;
	node_info->num_connections = pdata_node_info->num_connections;
	node_info->num_blist = pdata_node_info->num_blist;
	node_info->num_qports = pdata_node_info->num_qports;
	node_info->num_aggports = pdata_node_info->num_aggports;
	node_info->buswidth = pdata_node_info->buswidth;
	node_info->virt_dev = pdata_node_info->virt_dev;
	node_info->is_fab_dev = pdata_node_info->is_fab_dev;
	node_info->qos_params.mode = pdata_node_info->qos_params.mode;
	node_info->qos_params.prio1 = pdata_node_info->qos_params.prio1;
	node_info->qos_params.prio0 = pdata_node_info->qos_params.prio0;
	node_info->qos_params.reg_prio1 = pdata_node_info->qos_params.reg_prio1;
	node_info->qos_params.reg_prio0 = pdata_node_info->qos_params.reg_prio0;
	node_info->qos_params.prio_lvl = pdata_node_info->qos_params.prio_lvl;
	node_info->qos_params.prio_rd = pdata_node_info->qos_params.prio_rd;
	node_info->qos_params.prio_wr = pdata_node_info->qos_params.prio_wr;
	node_info->qos_params.gp = pdata_node_info->qos_params.gp;
	node_info->qos_params.thmp = pdata_node_info->qos_params.thmp;
	node_info->qos_params.ws = pdata_node_info->qos_params.ws;
	node_info->qos_params.bw_buffer = pdata_node_info->qos_params.bw_buffer;
	node_info->util_fact = pdata_node_info->util_fact;
	node_info->vrail_comp = pdata_node_info->vrail_comp;

	node_info->dev_connections = devm_kzalloc(bus_dev,
			sizeof(struct device *) *
				pdata_node_info->num_connections,
			GFP_KERNEL);
	if (!node_info->dev_connections) {
		MSM_BUS_ERR("%s:Bus dev connections alloc failed\n", __func__);
		ret = -ENOMEM;
		goto exit_copy_node_info;
	}

	node_info->connections = devm_kzalloc(bus_dev,
			sizeof(int) * pdata_node_info->num_connections,
			GFP_KERNEL);
	if (!node_info->connections) {
		MSM_BUS_ERR("%s:Bus connections alloc failed\n", __func__);
		devm_kfree(bus_dev, node_info->dev_connections);
		ret = -ENOMEM;
		goto exit_copy_node_info;
	}

	memcpy(node_info->connections,
		pdata_node_info->connections,
		sizeof(int) * pdata_node_info->num_connections);

	node_info->black_connections = devm_kzalloc(bus_dev,
			sizeof(struct device *) *
				pdata_node_info->num_blist,
			GFP_KERNEL);
	if (!node_info->black_connections) {
		MSM_BUS_ERR("%s: Bus black connections alloc failed\n",
			__func__);
		devm_kfree(bus_dev, node_info->dev_connections);
		devm_kfree(bus_dev, node_info->connections);
		ret = -ENOMEM;
		goto exit_copy_node_info;
	}

	node_info->black_listed_connections = devm_kzalloc(bus_dev,
			pdata_node_info->num_blist * sizeof(int),
			GFP_KERNEL);
	if (!node_info->black_listed_connections) {
		MSM_BUS_ERR("%s:Bus black list connections alloc failed\n",
					__func__);
		devm_kfree(bus_dev, node_info->black_connections);
		devm_kfree(bus_dev, node_info->dev_connections);
		devm_kfree(bus_dev, node_info->connections);
		ret = -ENOMEM;
		goto exit_copy_node_info;
	}

	memcpy(node_info->black_listed_connections,
		pdata_node_info->black_listed_connections,
		sizeof(int) * pdata_node_info->num_blist);

	node_info->qport = devm_kzalloc(bus_dev,
			sizeof(int) * pdata_node_info->num_qports,
			GFP_KERNEL);
	if (!node_info->qport) {
		MSM_BUS_ERR("%s:Bus qport allocation failed\n", __func__);
		devm_kfree(bus_dev, node_info->dev_connections);
		devm_kfree(bus_dev, node_info->connections);
		devm_kfree(bus_dev, node_info->black_listed_connections);
		ret = -ENOMEM;
		goto exit_copy_node_info;
	}

	memcpy(node_info->qport,
		pdata_node_info->qport,
		sizeof(int) * pdata_node_info->num_qports);

exit_copy_node_info:
	return ret;
}

static struct device *msm_bus_device_init(
			struct msm_bus_node_device_type *pdata)
{
	struct device *bus_dev = NULL;
	struct msm_bus_node_device_type *bus_node = NULL;
	struct msm_bus_node_info_type *node_info = NULL;
	int ret = 0;

	/**
	* Init here so we can use devm calls
	*/

	bus_node = kzalloc(sizeof(struct msm_bus_node_device_type), GFP_KERNEL);
	if (!bus_node) {
		MSM_BUS_ERR("%s:Bus node alloc failed\n", __func__);
		kfree(bus_dev);
		bus_dev = NULL;
		goto exit_device_init;
	}
	bus_dev = &bus_node->dev;
	device_initialize(bus_dev);

	node_info = devm_kzalloc(bus_dev,
			sizeof(struct msm_bus_node_info_type), GFP_KERNEL);
	if (!node_info) {
		MSM_BUS_ERR("%s:Bus node info alloc failed\n", __func__);
		devm_kfree(bus_dev, bus_node);
		kfree(bus_dev);
		bus_dev = NULL;
		goto exit_device_init;
	}

	bus_node->node_info = node_info;
	bus_node->ap_owned = pdata->ap_owned;

	if (msm_bus_copy_node_info(pdata, bus_dev) < 0) {
		devm_kfree(bus_dev, bus_node);
		devm_kfree(bus_dev, node_info);
		kfree(bus_dev);
		bus_dev = NULL;
		goto exit_device_init;
	}

	bus_dev->bus = &msm_bus_type;
	dev_set_name(bus_dev, bus_node->node_info->name);

	ret = device_add(bus_dev);
	if (ret < 0) {
		MSM_BUS_ERR("%s: Error registering device %d",
				__func__, pdata->node_info->id);
		devm_kfree(bus_dev, bus_node);
		devm_kfree(bus_dev, node_info->dev_connections);
		devm_kfree(bus_dev, node_info->connections);
		devm_kfree(bus_dev, node_info->black_connections);
		devm_kfree(bus_dev, node_info->black_listed_connections);
		devm_kfree(bus_dev, node_info);
		kfree(bus_dev);
		bus_dev = NULL;
		goto exit_device_init;
	}
	device_create_file(bus_dev, &dev_attr_vrail);
	device_create_file(bus_dev, &dev_attr_bw);
	INIT_LIST_HEAD(&bus_node->devlist);

exit_device_init:
	return bus_dev;
}

static int msm_bus_setup_dev_conn(struct device *bus_dev, void *data)
{
	struct msm_bus_node_device_type *bus_node = NULL;
	int ret = 0;
	int j;
	struct msm_bus_node_device_type *fab;

	bus_node = to_msm_bus_node(bus_dev);
	if (!bus_node) {
		MSM_BUS_ERR("%s: Can't get device info", __func__);
		ret = -ENODEV;
		goto exit_setup_dev_conn;
	}

	/* Setup parent bus device for this node */
	if (!bus_node->node_info->is_fab_dev) {
		struct device *bus_parent_device =
			bus_find_device(&msm_bus_type, NULL,
				(void *)&bus_node->node_info->bus_device_id,
				msm_bus_device_match_adhoc);

		if (!bus_parent_device) {
			MSM_BUS_ERR("%s: Error finding parentdev %d parent %d",
				__func__,
				bus_node->node_info->id,
				bus_node->node_info->bus_device_id);
			ret = -ENXIO;
			goto exit_setup_dev_conn;
		}
		bus_node->node_info->bus_device = bus_parent_device;
		fab = to_msm_bus_node(bus_parent_device);
		list_add_tail(&bus_node->dev_link, &fab->devlist);
	}

	bus_node->node_info->is_traversed = false;

	for (j = 0; j < bus_node->node_info->num_connections; j++) {
		bus_node->node_info->dev_connections[j] =
			bus_find_device(&msm_bus_type, NULL,
				(void *)&bus_node->node_info->connections[j],
				msm_bus_device_match_adhoc);

		if (!bus_node->node_info->dev_connections[j]) {
			MSM_BUS_ERR("%s: Error finding conn %d for device %d",
				__func__, bus_node->node_info->connections[j],
				 bus_node->node_info->id);
			ret = -ENODEV;
			goto exit_setup_dev_conn;
		}
	}

	for (j = 0; j < bus_node->node_info->num_blist; j++) {
		bus_node->node_info->black_connections[j] =
			bus_find_device(&msm_bus_type, NULL,
				(void *)&bus_node->node_info->
				black_listed_connections[j],
				msm_bus_device_match_adhoc);

		if (!bus_node->node_info->black_connections[j]) {
			MSM_BUS_ERR("%s: Error finding conn %d for device %d\n",
				__func__, bus_node->node_info->
				black_listed_connections[j],
				bus_node->node_info->id);
			ret = -ENODEV;
			goto exit_setup_dev_conn;
		}
	}

exit_setup_dev_conn:
	return ret;
}

static int msm_bus_node_debug(struct device *bus_dev, void *data)
{
	int j;
	int ret = 0;
	struct msm_bus_node_device_type *bus_node = NULL;

	bus_node = to_msm_bus_node(bus_dev);
	if (!bus_node) {
		MSM_BUS_ERR("%s: Can't get device info", __func__);
		ret = -ENODEV;
		goto exit_node_debug;
	}

	MSM_BUS_DBG("Device = %d buswidth %u", bus_node->node_info->id,
				bus_node->node_info->buswidth);
	for (j = 0; j < bus_node->node_info->num_connections; j++) {
		struct msm_bus_node_device_type *bdev =
		to_msm_bus_node(bus_node->node_info->dev_connections[j]);
		MSM_BUS_DBG("\n\t Connection[%d] %d", j, bdev->node_info->id);
	}

	if (bus_node->node_info->is_fab_dev)
		msm_bus_floor_init(bus_dev);

exit_node_debug:
	return ret;
}

static int msm_bus_device_probe(struct platform_device *pdev)
{
	unsigned int i, ret;
	struct msm_bus_device_node_registration *pdata;

	/* If possible, get pdata from device-tree */
	if (pdev->dev.of_node)
		pdata = msm_bus_of_to_pdata(pdev);
	else {
		pdata = (struct msm_bus_device_node_registration *)pdev->
			dev.platform_data;
	}

	if (IS_ERR_OR_NULL(pdata)) {
		MSM_BUS_ERR("No platform data found");
		ret = -ENODATA;
		goto exit_device_probe;
	}

	for (i = 0; i < pdata->num_devices; i++) {
		struct device *node_dev = NULL;

		node_dev = msm_bus_device_init(&pdata->info[i]);

		if (!node_dev) {
			MSM_BUS_ERR("%s: Error during dev init for %d",
				__func__, pdata->info[i].node_info->id);
			ret = -ENXIO;
			goto exit_device_probe;
		}

		ret = msm_bus_init_clk(node_dev, &pdata->info[i]);
		/*Is this a fabric device ?*/
		if (pdata->info[i].node_info->is_fab_dev) {
			MSM_BUS_DBG("%s: %d is a fab", __func__,
						pdata->info[i].node_info->id);
			ret = msm_bus_fabric_init(node_dev, &pdata->info[i]);
			if (ret) {
				MSM_BUS_ERR("%s: Error intializing fab %d",
					__func__, pdata->info[i].node_info->id);
				goto exit_device_probe;
			}
		}
	}

	ret = bus_for_each_dev(&msm_bus_type, NULL, NULL,
						msm_bus_setup_dev_conn);
	if (ret) {
		MSM_BUS_ERR("%s: Error setting up dev connections", __func__);
		goto exit_device_probe;
	}

	ret = bus_for_each_dev(&msm_bus_type, NULL, NULL, msm_bus_dev_init_qos);
	if (ret) {
		MSM_BUS_ERR("%s: Error during qos init", __func__);
		goto exit_device_probe;
	}


	/* Register the arb layer ops */
	msm_bus_arb_setops_adhoc(&arb_ops);
	bus_for_each_dev(&msm_bus_type, NULL, NULL, msm_bus_node_debug);

	devm_kfree(&pdev->dev, pdata->info);
	devm_kfree(&pdev->dev, pdata);
exit_device_probe:
	return ret;
}

static int msm_bus_device_rules_probe(struct platform_device *pdev)
{
	struct bus_rule_type *rule_data = NULL;
	int num_rules = 0;

	num_rules = msm_bus_of_get_static_rules(pdev, &rule_data);

	if (!rule_data)
		goto exit_rules_probe;

	msm_rule_register(num_rules, rule_data, NULL);
	static_rules.num_rules = num_rules;
	static_rules.rules = rule_data;
	pdev->dev.platform_data = &static_rules;

exit_rules_probe:
	return 0;
}

int msm_bus_device_rules_remove(struct platform_device *pdev)
{
	struct static_rules_type *static_rules = NULL;

	static_rules = pdev->dev.platform_data;
	if (static_rules)
		msm_rule_unregister(static_rules->num_rules,
					static_rules->rules, NULL);
	return 0;
}

static int msm_bus_free_dev(struct device *dev, void *data)
{
	struct msm_bus_node_device_type *bus_node = NULL;

	bus_node = dev->platform_data;

	if (bus_node)
		MSM_BUS_ERR("\n%s: Removing device %d", __func__,
						bus_node->node_info->id);
	device_unregister(dev);
	return 0;
}

int msm_bus_device_remove(struct platform_device *pdev)
{
	bus_for_each_dev(&msm_bus_type, NULL, NULL, msm_bus_free_dev);
	return 0;
}

static struct of_device_id rules_match[] = {
	{.compatible = "qcom,msm-bus-static-bw-rules"},
	{}
};

static struct platform_driver msm_bus_rules_driver = {
	.probe = msm_bus_device_rules_probe,
	.remove = msm_bus_device_rules_remove,
	.driver = {
		.name = "msm_bus_rules_device",
		.owner = THIS_MODULE,
		.of_match_table = rules_match,
	},
};

static struct of_device_id fabric_match[] = {
	{.compatible = "qcom,msm-bus-device"},
	{}
};

static struct platform_driver msm_bus_device_driver = {
	.probe = msm_bus_device_probe,
	.remove = msm_bus_device_remove,
	.driver = {
		.name = "msm_bus_device",
		.owner = THIS_MODULE,
		.of_match_table = fabric_match,
	},
};

int __init msm_bus_device_init_driver(void)
{
	int rc;

	MSM_BUS_ERR("msm_bus_fabric_init_driver\n");
	rc =  platform_driver_register(&msm_bus_device_driver);

	if (rc) {
		MSM_BUS_ERR("Failed to register bus device driver");
		return rc;
	}
	return platform_driver_register(&msm_bus_rules_driver);
}
subsys_initcall(msm_bus_device_init_driver);
