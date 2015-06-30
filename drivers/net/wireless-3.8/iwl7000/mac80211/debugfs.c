/*
 * mac80211 debugfs for wireless PHYs
 *
 * Copyright 2007	Johannes Berg <johannes@sipsolutions.net>
 * Copyright 2013-2015  Intel Mobile Communications GmbH
 *
 * GPLv2
 *
 */

#include <linux/debugfs.h>
#include <linux/rtnetlink.h>
#include "ieee80211_i.h"
#include "driver-ops.h"
#include "rate.h"
#include "debugfs.h"

#define DEBUGFS_FORMAT_BUFFER_SIZE 100

#ifdef CPTCFG_MAC80211_LATENCY_MEASUREMENTS
#define TX_TIMING_STATS_BIN_DELIMTER_C ','
#define TX_TIMING_STATS_BIN_DELIMTER_S ","
#define TX_TIMING_STATS_BINS_DISABLED "enable(bins disabled)\n"
#define TX_TIMING_STATS_DISABLED "disable\n"


#ifdef CPTCFG_NL80211_TESTMODE
/*
 * Display Tx latency threshold (per interface per TID) for triggering
 * usniffer logs event.
 */
static ssize_t sta_tx_latency_threshold_read(struct file *file,
					char __user *userbuf,
					size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	struct ieee80211_tx_latency_threshold  *tx_thrshld;
	char buf[2 * IEEE80211_NUM_TIDS * 20 + 128];
	int bufsz = sizeof(buf);
	int pos = 0;
	u32 i;

	rcu_read_lock();
	tx_thrshld = rcu_dereference(local->tx_threshold);

	/* Tx threshold is not enabled or no thresholds were configured */
	if (!tx_thrshld || (!tx_thrshld->thresholds_bss &&
			    !tx_thrshld->thresholds_p2p)) {
		pos += scnprintf(buf + pos, bufsz - pos, "%s\n",
				TX_TIMING_STATS_DISABLED);
		goto unlock;
	}

	pos += scnprintf(buf + pos, bufsz - pos, "mode: %s\n",
			 tx_thrshld->monitor_record_mode ?
			 "Continuous Recording" : "Internal Buffer");
	pos += scnprintf(buf + pos, bufsz - pos, "window: %d\n",
			 tx_thrshld->monitor_collec_wind);

	if (tx_thrshld->thresholds_bss) {
		pos += scnprintf(buf + pos, bufsz - pos, "BSS thresholds:\n");
		for (i = 0; i < IEEE80211_NUM_TIDS; i++) {
			pos += scnprintf(buf + pos, bufsz - pos, "TID %u: %u\n",
					 i, tx_thrshld->thresholds_bss[i]);
		}
	}

	if (tx_thrshld->thresholds_p2p) {
		pos += scnprintf(buf + pos, bufsz - pos, "P2P thresholds:\n");
		for (i = 0; i < IEEE80211_NUM_TIDS; i++) {
			pos += scnprintf(buf + pos, bufsz - pos,
					 "TID %u: %u\n",
					 i, tx_thrshld->thresholds_p2p[i]);
		}
	}
unlock:
	rcu_read_unlock();
	return simple_read_from_buffer(userbuf, count, ppos, buf, pos);
}

/*
 * Configure Tx latency threshold for triggering usniffer logs event.
 * Configuration is done per interface per TID.
 * Enable input: <mode,window,iface,tid,threshold>
 * Disable input (for an interface):<mode,window,iface,tid,-1>
 *	mode - Internal buffer mode continuos recording mode (recording
 *	with MIPI)
 *	window - the size of the window before collecting the usniffer logs
 *	iface - 0 for BSS 1 for P2P
 *	tid - the tid to apply the threshold on
 *	threshold- the wanted threshold (-1 to disable for the interface 0 to
 *	disable only for the tid)
 */
static ssize_t sta_tx_latency_threshold_write(struct file *file,
					      const char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	struct ieee80211_tx_latency_threshold  *tx_thrshld;
	u32 alloc_size;
	int mode;
	int window_size;
	int thrshld;
	int tid;
	int iface;
	int ret;

	char buf[128] = {};

	if (sizeof(buf) <= count)
		return -EINVAL;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	ret = count;

	if (sscanf(buf, "%d,%d,%d,%d,%d", &mode, &window_size, &iface, &tid,
		   &thrshld) != 5)
		return -EINVAL;

	mutex_lock(&local->sta_mtx);

	/* cannot change config once we have stations */
	if (local->num_sta)
		goto unlock;

	tx_thrshld =
		rcu_dereference_protected(local->tx_threshold,
					  lockdep_is_held(&local->sta_mtx));
	if (tx_thrshld)
		goto config_thrshld;

	tx_thrshld = kzalloc(sizeof(struct ieee80211_tx_latency_threshold),
			     GFP_ATOMIC);
	if (!tx_thrshld) {
		ret = -ENOMEM;
		goto unlock;
	}

config_thrshld:
	/* Check if we need to disable the thresholds */
	if (iface == IEEE80211_TX_LATENCY_BSS &&
	    thrshld == IEEE80211_IF_DISABLE_THSHLD) {
		kfree(tx_thrshld->thresholds_bss);
		tx_thrshld->thresholds_bss = NULL;
		goto assign;
	} else if (iface == IEEE80211_TX_LATENCY_P2P &&
	    thrshld == IEEE80211_IF_DISABLE_THSHLD) {
		kfree(tx_thrshld->thresholds_p2p);
		tx_thrshld->thresholds_p2p = NULL;
		goto assign;
	}

	/* Check tid  && thrshld parameters are valid */
	if (tid >= IEEE80211_NUM_TIDS || tid < 0 ||
	    thrshld < IEEE80211_IF_DISABLE_THSHLD) {
		ret = -EINVAL;
		goto unlock;
	}

	alloc_size = sizeof(u32) * IEEE80211_NUM_TIDS;
	/* Check iface paramters is valid */
	if (iface == IEEE80211_TX_LATENCY_BSS) {
		if (!tx_thrshld->thresholds_bss) {
			tx_thrshld->thresholds_bss = kzalloc(alloc_size,
							     GFP_ATOMIC);
			if (!tx_thrshld->thresholds_bss) {
				ret = -ENOMEM;
				goto unlock;
			}
		}
		tx_thrshld->thresholds_bss[tid] = thrshld;
	} else if (iface == IEEE80211_TX_LATENCY_P2P) {
		if (!tx_thrshld->thresholds_p2p) {
			tx_thrshld->thresholds_p2p = kzalloc(alloc_size,
							     GFP_ATOMIC);
			if (!tx_thrshld->thresholds_p2p) {
				ret = -ENOMEM;
				goto unlock;
			}
		}
		tx_thrshld->thresholds_p2p[tid] = thrshld;
	} else { /* not a valid interface */
		ret = -EINVAL;
		goto unlock;
	}
	tx_thrshld->monitor_collec_wind  = window_size;
	tx_thrshld->monitor_record_mode = mode;
assign:
	rcu_assign_pointer(local->tx_threshold, tx_thrshld);
unlock:
	mutex_unlock(&local->sta_mtx);

	return ret;
}

static const struct file_operations stats_tx_latency_threshold_ops = {
	.write = sta_tx_latency_threshold_write,
	.read = sta_tx_latency_threshold_read,
	.open = simple_open,
	.llseek = generic_file_llseek,
};
#endif

static ssize_t sta_tx_latency_points_read(struct file *file,
					  char __user *userbuf,
					  size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	struct ieee80211_tx_latency_bin_ranges  *tx_latency;
	char buf[516];
	int bufsz = sizeof(buf);
	int pos = 0;

	rcu_read_lock();
	tx_latency = rcu_dereference(local->tx_latency);

	/* Tx latency is not enabled or no thresholds were configured */
	if (!tx_latency) {
		pos += scnprintf(buf + pos, bufsz - pos, "%s\n",
				TX_TIMING_STATS_DISABLED);
		goto unlock;
	}

	pos += scnprintf(buf + pos, bufsz - pos, "start: %u\n",
			 local->tx_msrmnt_points[0]);
	pos += scnprintf(buf + pos, bufsz - pos, "end: %u\n",
			 local->tx_msrmnt_points[1]);

unlock:
	rcu_read_unlock();
	return simple_read_from_buffer(userbuf, count, ppos, buf, pos);
}

/*
 * Configure the Tx latency measuring points.
 *
 * In order to analyze the Tx latency there are 4 points that are of interest:
 * 1. Tx packet Enter the Kernel
 * 2. Tx packet is written to the bus
 * 3. Tx packet is acked by the AP
 * 4. Tx packet is erased.
 *
 * By default the measurement is done between points 1 to 4 (measure the whole
 * Tx flow). If one sees that the there is some problem with the Tx  latency,
 * he can try to focus the analysis to different parts of the Tx flow.
 *
 * Valid input is: a,b where
 * 1. a < b
 * 2. IEEE80211_TX_LAT_ENTER<=a
 * 3. b < IEEE80211_TX_LAT_MAX_POINT
 * 4. a & b are values from enum ieee80211_tx_lat_msr_point (0-3)
 */
static ssize_t sta_tx_latency_points_write(struct file *file,
					   const char __user *userbuf,
					   size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	struct ieee80211_tx_latency_bin_ranges  *tx_latency;
	int ret;
	u32 start, end;

	char buf[32] = {};

	if (sizeof(buf) <= count)
		return -EINVAL;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	ret = count;

	if (sscanf(buf, "%u,%u", &start, &end) != 2 || end <= start ||
	    IEEE80211_TX_LAT_MAX_POINT <= end)
		return -EINVAL;

	mutex_lock(&local->sta_mtx);

	/* cannot change config once we have stations */
	if (local->num_sta)
		goto unlock;

	tx_latency =
		rcu_dereference_protected(local->tx_latency,
					  lockdep_is_held(&local->sta_mtx));
	/* Tx latency disabled */
	if (!tx_latency)
		goto unlock;

	local->tx_msrmnt_points[0] = start;
	local->tx_msrmnt_points[1] = end;

	rcu_assign_pointer(local->tx_latency, tx_latency);
unlock:
	mutex_unlock(&local->sta_mtx);

	return ret;
}

static const struct file_operations stats_tx_latency_points_ops = {
	.write = sta_tx_latency_points_write,
	.read = sta_tx_latency_points_read,
	.open = simple_open,
	.llseek = generic_file_llseek,
};
/*
 * Display if Tx latency statistics & bins are enabled/disabled
 */
static ssize_t sta_tx_latency_stat_read(struct file *file,
					char __user *userbuf,
					size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	struct ieee80211_tx_latency_bin_ranges  *tx_latency;
	char *buf;
	int bufsz, i, ret;
	int pos = 0;

	rcu_read_lock();

	tx_latency = rcu_dereference(local->tx_latency);

	if (tx_latency && tx_latency->n_ranges) {
		bufsz = tx_latency->n_ranges * 15;
		buf = kzalloc(bufsz, GFP_ATOMIC);
		if (!buf)
			goto err;

		for (i = 0; i < tx_latency->n_ranges; i++)
			pos += scnprintf(buf + pos, bufsz - pos, "%d,",
					 tx_latency->ranges[i]);
		pos += scnprintf(buf + pos, bufsz - pos, "\n");
	} else if (tx_latency) {
		bufsz = sizeof(TX_TIMING_STATS_BINS_DISABLED) + 1;
		buf = kzalloc(bufsz, GFP_ATOMIC);
		if (!buf)
			goto err;

		pos += scnprintf(buf + pos, bufsz - pos, "%s\n",
				 TX_TIMING_STATS_BINS_DISABLED);
	} else {
		bufsz = sizeof(TX_TIMING_STATS_DISABLED) + 1;
		buf = kzalloc(bufsz, GFP_ATOMIC);
		if (!buf)
			goto err;

		pos += scnprintf(buf + pos, bufsz - pos, "%s\n",
				 TX_TIMING_STATS_DISABLED);
	}

	rcu_read_unlock();

	ret = simple_read_from_buffer(userbuf, count, ppos, buf, pos);
	kfree(buf);

	return ret;
err:
	rcu_read_unlock();
	return -ENOMEM;
}

/*
 * Receive input from user regarding Tx latency statistics
 * The input should indicate if Tx latency statistics and bins are
 * enabled/disabled.
 * If bins are enabled input should indicate the amount of different bins and
 * their ranges. Each bin will count how many Tx frames transmitted within the
 * appropriate latency.
 * Legal input is:
 * a) "enable(bins disabled)" - to enable only general statistics
 * b) "a,b,c,d,...z" - to enable general statistics and bins, where all are
 * numbers and a < b < c < d.. < z
 * c) "disable" - disable all statistics
 * NOTE: must configure Tx latency statistics bins before stations connected.
 */

static ssize_t sta_tx_latency_stat_write(struct file *file,
					 const char __user *userbuf,
					 size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	char buf[128] = {};
	char *bins = buf;
	char *token;
	int buf_size, i, alloc_size;
	int prev_bin = 0;
	int n_ranges = 0;
	int ret = count;
	struct ieee80211_tx_latency_bin_ranges  *tx_latency;

	if (sizeof(buf) <= count)
		return -EINVAL;
	buf_size = count;
	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	mutex_lock(&local->sta_mtx);

	/* cannot change config once we have stations */
	if (local->num_sta)
		goto unlock;

	tx_latency =
		rcu_dereference_protected(local->tx_latency,
					  lockdep_is_held(&local->sta_mtx));

	/* disable Tx statistics */
	if (!strcmp(buf, TX_TIMING_STATS_DISABLED)) {
		if (!tx_latency)
			goto unlock;
		RCU_INIT_POINTER(local->tx_latency, NULL);
		synchronize_rcu();
		kfree(tx_latency);
		goto unlock;
	}

	/* Tx latency already enabled */
	if (tx_latency)
		goto unlock;

	if (strcmp(TX_TIMING_STATS_BINS_DISABLED, buf)) {
		/* check how many bins and between what ranges user requested */
		token = buf;
		while (*token != '\0') {
			if (*token == TX_TIMING_STATS_BIN_DELIMTER_C)
				n_ranges++;
			token++;
		}
		n_ranges++;
	}

	alloc_size = sizeof(struct ieee80211_tx_latency_bin_ranges) +
		     n_ranges * sizeof(u32);
	tx_latency = kzalloc(alloc_size, GFP_ATOMIC);
	if (!tx_latency) {
		ret = -ENOMEM;
		goto unlock;
	}
	tx_latency->n_ranges = n_ranges;
	for (i = 0; i < n_ranges; i++) { /* setting bin ranges */
		token = strsep(&bins, TX_TIMING_STATS_BIN_DELIMTER_S);
		sscanf(token, "%d", &tx_latency->ranges[i]);
		/* bins values should be in ascending order */
		if (prev_bin >= tx_latency->ranges[i]) {
			ret = -EINVAL;
			kfree(tx_latency);
			goto unlock;
		}
		prev_bin = tx_latency->ranges[i];
	}

	/* set default measurement points */
	local->tx_msrmnt_points[0] = IEEE80211_TX_LAT_ENTER;
	local->tx_msrmnt_points[1] = IEEE80211_TX_LAT_DEL;

	rcu_assign_pointer(local->tx_latency, tx_latency);

unlock:
	mutex_unlock(&local->sta_mtx);

	return ret;
}

static const struct file_operations stats_tx_latency_ops = {
	.write = sta_tx_latency_stat_write,
	.read = sta_tx_latency_stat_read,
	.open = simple_open,
	.llseek = generic_file_llseek,
};

/*
 * Display if Tx consecutive loss statistics are enabled/disabled
 */
static ssize_t sta_tx_consecutive_loss_read(struct file *file,
					    char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	struct ieee80211_tx_consec_loss_ranges  *tx_consec;
	char *buf;
	size_t bufsz, i;
	int ret;
	u32 pos = 0;

	rcu_read_lock();

	tx_consec = rcu_dereference(local->tx_consec);

	if (tx_consec && tx_consec->n_ranges) { /* enabled */
		bufsz = tx_consec->n_ranges * 16;
		buf = kzalloc(bufsz, GFP_ATOMIC);
		if (!buf)
			goto err;

		pos += scnprintf(buf, bufsz, "bins: ");
		for (i = 0; i < tx_consec->n_ranges; i++)
			pos += scnprintf(buf + pos, bufsz - pos, "%u,",
					 tx_consec->ranges[i]);
		pos += scnprintf(buf + pos, bufsz - pos,
				 "\nlate threshold: %d\n",
				 tx_consec->late_threshold);
	} else { /* disabled */
		bufsz = sizeof(TX_TIMING_STATS_DISABLED) + 1;
		buf = kzalloc(bufsz, GFP_ATOMIC);
		if (!buf)
			goto err;

		pos += scnprintf(buf + pos, bufsz - pos, "%s\n",
				 TX_TIMING_STATS_DISABLED);
	}

	rcu_read_unlock();

	ret = simple_read_from_buffer(userbuf, count, ppos, buf, pos);
	kfree(buf);

	return ret;
err:
	rcu_read_unlock();
	return -ENOMEM;
}

/*
 * Receive input from user regarding Tx consecutive loss statistics
 * The input should indicate if Tx consecutive loss statistics are
 * enabled/disabled.
 * This entry keep track of 2 statistics to do with Tx consecutive loss:
 * 1) How many consecutive packets were lost within the bin ranges
 * 2) How many consecutive packets were sent successfully but the transmit
 * latency is greater than the late threshold, therefor considered lost.
 *
 * Legal input is:
 * a) "a,b,c,d,...z,threshold" - to enable consecutive loss statistics,
 * where all are numbers and a < b < c < d.. < z
 * b) "disable" - disable all statistics
 * NOTE:
 * 1) Must supply at least 2 values.
 * 2) Last value is always the late threshold.
 * 3) Must configure Tx consecutive loss statistics bins before stations
 * connected.
 */

static ssize_t sta_tx_consecutive_loss_write(struct file *file,
					     const char __user *userbuf,
					     size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	char buf[128] = {};
	char *bins = buf;
	char *token;
	u32 i, alloc_size;
	u32 prev_bin = 0;
	int n_ranges = 0;
	int n_vals = 0;
	int ret = count;
	struct ieee80211_tx_consec_loss_ranges *tx_consec;

	if (sizeof(buf) <= count)
		return -EINVAL;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	mutex_lock(&local->sta_mtx);

	/* cannot change config once we have stations */
	if (local->num_sta)
		goto unlock;

	tx_consec =
		rcu_dereference_protected(local->tx_consec,
					  lockdep_is_held(&local->sta_mtx));

	/* disable Tx statistics */
	if (!strcmp(buf, TX_TIMING_STATS_DISABLED)) {
		if (!tx_consec)
			goto unlock;
		rcu_assign_pointer(local->tx_consec, NULL);
		synchronize_rcu();
		kfree(tx_consec);
		goto unlock;
	}

	/* Tx latency already enabled */
	if (tx_consec)
		goto unlock;

	/* check how many bins and between what ranges user requested */
	token = buf;
	while (*token != '\0') {
		if (*token == TX_TIMING_STATS_BIN_DELIMTER_C)
			n_vals++;
		token++;
	}
	n_vals++;
	/* last value is for setting the late threshold */
	n_ranges = n_vals - 1;

	/*
	 * user needs to enter at least 2 values, one for the threshold, and
	 * one for the range
	 */
	if (n_vals < 2) {
		ret = -EINVAL;
		goto unlock;
	}

	alloc_size = sizeof(struct ieee80211_tx_consec_loss_ranges) +
		     n_ranges * sizeof(u32);
	tx_consec = kzalloc(alloc_size, GFP_ATOMIC);
	if (!tx_consec) {
		ret = -ENOMEM;
		goto unlock;
	}
	tx_consec->n_ranges = n_ranges;
	for (i = 0; i < n_vals; i++) { /* setting bin ranges */
		token = strsep(&bins, TX_TIMING_STATS_BIN_DELIMTER_S);

		if (i == n_vals - 1) { /* last value - late threshold */
			ret = sscanf(token, "%d", &tx_consec->late_threshold);
			if (ret != 1) {
				ret = -EINVAL;
				kfree(tx_consec);
				goto unlock;
			}
			break;
		}
		ret = sscanf(token, "%d", &tx_consec->ranges[i]);
		if (ret != 1) {
			ret = -EINVAL;
			kfree(tx_consec);
			goto unlock;
		}

		/* bins values should be in ascending order */
		if (prev_bin >= tx_consec->ranges[i]) {
			ret = -EINVAL;
			kfree(tx_consec);
			goto unlock;
		}
		prev_bin = tx_consec->ranges[i];
	}

	/* set default measurement points */
	local->tx_msrmnt_points[0] = IEEE80211_TX_LAT_ENTER;
	local->tx_msrmnt_points[1] = IEEE80211_TX_LAT_DEL;

	rcu_assign_pointer(local->tx_consec, tx_consec);

unlock:
	mutex_unlock(&local->sta_mtx);

	return ret;
}

static const struct file_operations stats_tx_consecutive_loss_ops = {
	.write = sta_tx_consecutive_loss_write,
	.read = sta_tx_consecutive_loss_read,
	.open = simple_open,
	.llseek = generic_file_llseek,
};
#endif /* CPTCFG_MAC80211_LATENCY_MEASUREMENTS */

int mac80211_format_buffer(char __user *userbuf, size_t count,
				  loff_t *ppos, char *fmt, ...)
{
	va_list args;
	char buf[DEBUGFS_FORMAT_BUFFER_SIZE];
	int res;

	va_start(args, fmt);
	res = vscnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	return simple_read_from_buffer(userbuf, count, ppos, buf, res);
}

#define DEBUGFS_READONLY_FILE_FN(name, fmt, value...)			\
static ssize_t name## _read(struct file *file, char __user *userbuf,	\
			    size_t count, loff_t *ppos)			\
{									\
	struct ieee80211_local *local = file->private_data;		\
									\
	return mac80211_format_buffer(userbuf, count, ppos, 		\
				      fmt "\n", ##value);		\
}

#define DEBUGFS_READONLY_FILE_OPS(name)			\
static const struct file_operations name## _ops = {			\
	.read = name## _read,						\
	.open = simple_open,						\
	.llseek = generic_file_llseek,					\
};

#define DEBUGFS_READONLY_FILE(name, fmt, value...)		\
	DEBUGFS_READONLY_FILE_FN(name, fmt, value)		\
	DEBUGFS_READONLY_FILE_OPS(name)

#define DEBUGFS_ADD(name)						\
	debugfs_create_file(#name, 0400, phyd, local, &name## _ops);

#define DEBUGFS_ADD_MODE(name, mode)					\
	debugfs_create_file(#name, mode, phyd, local, &name## _ops);


DEBUGFS_READONLY_FILE(user_power, "%d",
		      local->user_power_level);
DEBUGFS_READONLY_FILE(power, "%d",
		      local->hw.conf.power_level);
DEBUGFS_READONLY_FILE(total_ps_buffered, "%d",
		      local->total_ps_buffered);
DEBUGFS_READONLY_FILE(wep_iv, "%#08x",
		      local->wep_iv & 0xffffff);
DEBUGFS_READONLY_FILE(rate_ctrl_alg, "%s",
	local->rate_ctrl ? local->rate_ctrl->ops->name : "hw/driver");

#ifdef CONFIG_PM
static ssize_t reset_write(struct file *file, const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;

	rtnl_lock();
	__ieee80211_suspend(&local->hw, NULL);
	__ieee80211_resume(&local->hw);
	rtnl_unlock();

	return count;
}

static const struct file_operations reset_ops = {
	.write = reset_write,
	.open = simple_open,
	.llseek = noop_llseek,
};
#endif

static const char *hw_flag_names[NUM_IEEE80211_HW_FLAGS + 1] = {
#define FLAG(F)	[IEEE80211_HW_##F] = #F
	FLAG(HAS_RATE_CONTROL),
	FLAG(RX_INCLUDES_FCS),
	FLAG(HOST_BROADCAST_PS_BUFFERING),
	FLAG(SIGNAL_UNSPEC),
	FLAG(SIGNAL_DBM),
	FLAG(NEED_DTIM_BEFORE_ASSOC),
	FLAG(SPECTRUM_MGMT),
	FLAG(AMPDU_AGGREGATION),
	FLAG(SUPPORTS_PS),
	FLAG(PS_NULLFUNC_STACK),
	FLAG(SUPPORTS_DYNAMIC_PS),
	FLAG(MFP_CAPABLE),
	FLAG(WANT_MONITOR_VIF),
	FLAG(NO_AUTO_VIF),
	FLAG(SW_CRYPTO_CONTROL),
	FLAG(SUPPORT_FAST_XMIT),
	FLAG(REPORTS_TX_ACK_STATUS),
	FLAG(CONNECTION_MONITOR),
	FLAG(QUEUE_CONTROL),
	FLAG(SUPPORTS_PER_STA_GTK),
	FLAG(AP_LINK_PS),
	FLAG(TX_AMPDU_SETUP_IN_HW),
	FLAG(SUPPORTS_RC_TABLE),
	FLAG(P2P_DEV_ADDR_FOR_INTF),
	FLAG(TIMING_BEACON_ONLY),
	FLAG(SUPPORTS_HT_CCK_RATES),
	FLAG(CHANCTX_STA_CSA),
	FLAG(SUPPORTS_CLONED_SKBS),
	FLAG(SINGLE_SCAN_ON_ALL_BANDS),
	FLAG(TDLS_WIDER_BW),
	FLAG(SUPPORTS_FTM_INITIATOR),

	/* keep last for the build bug below */
	(void *)0x1
#undef FLAG
};

static ssize_t hwflags_read(struct file *file, char __user *user_buf,
			    size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	size_t bufsz = 30 * NUM_IEEE80211_HW_FLAGS;
	char *buf = kzalloc(bufsz, GFP_KERNEL);
	char *pos = buf, *end = buf + bufsz - 1;
	ssize_t rv;
	int i;

	if (!buf)
		return -ENOMEM;

	/* fail compilation if somebody adds or removes
	 * a flag without updating the name array above
	 */
	BUILD_BUG_ON(hw_flag_names[NUM_IEEE80211_HW_FLAGS] != (void *)0x1);

	for (i = 0; i < NUM_IEEE80211_HW_FLAGS; i++) {
		if (test_bit(i, local->hw.flags))
			pos += scnprintf(pos, end - pos, "%s",
					 hw_flag_names[i]);
	}

	rv = simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
	kfree(buf);
	return rv;
}

static ssize_t queues_read(struct file *file, char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct ieee80211_local *local = file->private_data;
	unsigned long flags;
	char buf[IEEE80211_MAX_QUEUES * 20];
	int q, res = 0;

	spin_lock_irqsave(&local->queue_stop_reason_lock, flags);
	for (q = 0; q < local->hw.queues; q++)
		res += sprintf(buf + res, "%02d: %#.8lx/%d\n", q,
				local->queue_stop_reasons[q],
				skb_queue_len(&local->pending[q]));
	spin_unlock_irqrestore(&local->queue_stop_reason_lock, flags);

	return simple_read_from_buffer(user_buf, count, ppos, buf, res);
}

DEBUGFS_READONLY_FILE_OPS(hwflags);
DEBUGFS_READONLY_FILE_OPS(queues);

/* statistics stuff */

static ssize_t format_devstat_counter(struct ieee80211_local *local,
	char __user *userbuf,
	size_t count, loff_t *ppos,
	int (*printvalue)(struct ieee80211_low_level_stats *stats, char *buf,
			  int buflen))
{
	struct ieee80211_low_level_stats stats;
	char buf[20];
	int res;

	rtnl_lock();
	res = drv_get_stats(local, &stats);
	rtnl_unlock();
	if (res)
		return res;
	res = printvalue(&stats, buf, sizeof(buf));
	return simple_read_from_buffer(userbuf, count, ppos, buf, res);
}

#define DEBUGFS_DEVSTATS_FILE(name)					\
static int print_devstats_##name(struct ieee80211_low_level_stats *stats,\
				 char *buf, int buflen)			\
{									\
	return scnprintf(buf, buflen, "%u\n", stats->name);		\
}									\
static ssize_t stats_ ##name## _read(struct file *file,			\
				     char __user *userbuf,		\
				     size_t count, loff_t *ppos)	\
{									\
	return format_devstat_counter(file->private_data,		\
				      userbuf,				\
				      count,				\
				      ppos,				\
				      print_devstats_##name);		\
}									\
									\
static const struct file_operations stats_ ##name## _ops = {		\
	.read = stats_ ##name## _read,					\
	.open = simple_open,						\
	.llseek = generic_file_llseek,					\
};

#define DEBUGFS_STATS_ADD(name)					\
	debugfs_create_u32(#name, 0400, statsd, &local->name);
#define DEBUGFS_DEVSTATS_ADD(name)					\
	debugfs_create_file(#name, 0400, statsd, local, &stats_ ##name## _ops);

DEBUGFS_DEVSTATS_FILE(dot11ACKFailureCount);
DEBUGFS_DEVSTATS_FILE(dot11RTSFailureCount);
DEBUGFS_DEVSTATS_FILE(dot11FCSErrorCount);
DEBUGFS_DEVSTATS_FILE(dot11RTSSuccessCount);

void debugfs_hw_add(struct ieee80211_local *local)
{
	struct dentry *phyd = local->hw.wiphy->debugfsdir;
	struct dentry *statsd;

	if (!phyd)
		return;

	local->debugfs.keys = debugfs_create_dir("keys", phyd);

	DEBUGFS_ADD(total_ps_buffered);
	DEBUGFS_ADD(wep_iv);
	DEBUGFS_ADD(queues);
#ifdef CONFIG_PM
	DEBUGFS_ADD_MODE(reset, 0200);
#endif
	DEBUGFS_ADD(hwflags);
	DEBUGFS_ADD(user_power);
	DEBUGFS_ADD(power);

	statsd = debugfs_create_dir("statistics", phyd);

	/* if the dir failed, don't put all the other things into the root! */
	if (!statsd)
		return;

#ifdef CPTCFG_MAC80211_DEBUG_COUNTERS
	DEBUGFS_STATS_ADD(dot11TransmittedFragmentCount);
	DEBUGFS_STATS_ADD(dot11MulticastTransmittedFrameCount);
	DEBUGFS_STATS_ADD(dot11FailedCount);
	DEBUGFS_STATS_ADD(dot11RetryCount);
	DEBUGFS_STATS_ADD(dot11MultipleRetryCount);
	DEBUGFS_STATS_ADD(dot11FrameDuplicateCount);
	DEBUGFS_STATS_ADD(dot11ReceivedFragmentCount);
	DEBUGFS_STATS_ADD(dot11MulticastReceivedFrameCount);
	DEBUGFS_STATS_ADD(dot11TransmittedFrameCount);
	DEBUGFS_STATS_ADD(tx_handlers_drop);
	DEBUGFS_STATS_ADD(tx_handlers_queued);
	DEBUGFS_STATS_ADD(tx_handlers_drop_wep);
	DEBUGFS_STATS_ADD(tx_handlers_drop_not_assoc);
	DEBUGFS_STATS_ADD(tx_handlers_drop_unauth_port);
	DEBUGFS_STATS_ADD(rx_handlers_drop);
	DEBUGFS_STATS_ADD(rx_handlers_queued);
	DEBUGFS_STATS_ADD(rx_handlers_drop_nullfunc);
	DEBUGFS_STATS_ADD(rx_handlers_drop_defrag);
	DEBUGFS_STATS_ADD(tx_expand_skb_head);
	DEBUGFS_STATS_ADD(tx_expand_skb_head_cloned);
	DEBUGFS_STATS_ADD(rx_expand_skb_head_defrag);
	DEBUGFS_STATS_ADD(rx_handlers_fragments);
	DEBUGFS_STATS_ADD(tx_status_drop);
#endif
	DEBUGFS_DEVSTATS_ADD(dot11ACKFailureCount);
	DEBUGFS_DEVSTATS_ADD(dot11RTSFailureCount);
	DEBUGFS_DEVSTATS_ADD(dot11FCSErrorCount);
	DEBUGFS_DEVSTATS_ADD(dot11RTSSuccessCount);

#ifdef CPTCFG_MAC80211_LATENCY_MEASUREMENTS
	DEBUGFS_DEVSTATS_ADD(tx_latency);
	DEBUGFS_DEVSTATS_ADD(tx_latency_threshold);
	DEBUGFS_DEVSTATS_ADD(tx_latency_points);
	DEBUGFS_DEVSTATS_ADD(tx_consecutive_loss);
#endif /* CPTCFG_MAC80211_LATENCY_MEASUREMENTS */
}
