/*
 * Dynamic sync control driver V2
 *
 * by andip71 (alias Lord Boeffla)
 *
 * All credits for original implemenation to faux123
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/writeback.h>
#include <linux/dyn_sync_cntrl.h>
#include <linux/fb.h>

/* Declarations */

bool __read_mostly suspend_active = false;
bool __read_mostly dyn_fsync_active = DYN_FSYNC_ACTIVE_DEFAULT;
static struct notifier_block notif;
extern void sync_filesystems(int wait);
static struct kobject *dyn_fsync_kobj;

/* Functions */

static void dyn_fsync_force_flush(void)
{
	sync_filesystems(0);
	sync_filesystems(1);
}

static ssize_t dyn_fsync_active_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", (dyn_fsync_active ? 1 : 0));
}

static ssize_t dyn_fsync_active_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;

	if (sscanf(buf, "%u\n", &data))
	{
		if (data) {
			pr_info("%s: dynamic fsync enabled\n", __func__);
			dyn_fsync_active = true;
		} else {
			pr_info("%s: dynamic fsync disabled\n", __func__);
			dyn_fsync_active = false;
			/* force a flush */
			dyn_fsync_force_flush();
		}
	}

	return count;
}

static ssize_t dyn_fsync_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u.%u\n",
		DYN_FSYNC_VERSION_MAJOR,
		DYN_FSYNC_VERSION_MINOR);
}

static int dyn_fsync_panic_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	/* kernel panic, force flush now */
	suspend_active = false;
	dyn_fsync_force_flush();
	pr_warn("dynamic fsync: panic - force flush!\n");

	return NOTIFY_DONE;
}

static int dyn_fsync_notify_sys(struct notifier_block *this, unsigned long code,
				void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT || code == SYS_POWER_OFF)
	{
		/*
		 * system shutdown or reboot, disable dynamic
		 * fsync and force flush
		 */
		suspend_active = false;
		dyn_fsync_active = false;
		dyn_fsync_force_flush();
		pr_warn("dynamic fsync: reboot - force flush!\n");
	}

	return NOTIFY_DONE;
}

static int fb_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	/* Parse framebuffer events as soon as they occur */
	if (action != FB_EARLY_EVENT_BLANK)
		return NOTIFY_OK;

	switch (*blank)
	{
		case FB_BLANK_UNBLANK:
			suspend_active = false;
			if (dyn_fsync_active)
				dyn_fsync_force_flush();
			break;
		case FB_BLANK_POWERDOWN:
			suspend_active = true;
			break;
		default:
			break;
	}

	return NOTIFY_OK;
}

/* Module structures */

static struct notifier_block dyn_fsync_notifier =
{
	.notifier_call = dyn_fsync_notify_sys,
};

static struct kobj_attribute dyn_fsync_active_attribute =
	__ATTR(Dyn_fsync_active, 0666,
		dyn_fsync_active_show,
		dyn_fsync_active_store);

static struct kobj_attribute dyn_fsync_version_attribute =
	__ATTR(Dyn_fsync_version, 0444, dyn_fsync_version_show, NULL);

static struct attribute *dyn_fsync_active_attrs[] =
{
	&dyn_fsync_active_attribute.attr,
	&dyn_fsync_version_attribute.attr,
	NULL,
};

static struct attribute_group dyn_fsync_active_attr_group =
{
	.attrs = dyn_fsync_active_attrs,
};

static struct notifier_block dyn_fsync_panic_block =
{
	.notifier_call  = dyn_fsync_panic_event,
	.priority       = INT_MAX,
};

/* Module init/exit */

static void dyn_fsync_exit(void);

static int dyn_fsync_init(void)
{
	int sysfs_result;

	register_reboot_notifier(&dyn_fsync_notifier);

	atomic_notifier_chain_register(&panic_notifier_list,
		&dyn_fsync_panic_block);

	dyn_fsync_kobj = kobject_create_and_add("dyn_fsync", kernel_kobj);

	if (!dyn_fsync_kobj)
	{
		pr_err("%s dyn_fsync_kobj create failed!\n", __func__);
		return -ENOMEM;
    	}

	sysfs_result = sysfs_create_group(dyn_fsync_kobj,
			&dyn_fsync_active_attr_group);

	if (sysfs_result)
	{
		pr_err("%s dyn_fsync sysfs create failed!\n", __func__);
		kobject_put(dyn_fsync_kobj);
	}

	notif.notifier_call = fb_notifier_callback;
	if (fb_register_client(&notif))
		dyn_fsync_exit();

	pr_info("%s dynamic fsync initialisation complete\n", __func__);

	return sysfs_result;
}


static void dyn_fsync_exit(void)
{
	unregister_reboot_notifier(&dyn_fsync_notifier);

	atomic_notifier_chain_unregister(&panic_notifier_list,
		&dyn_fsync_panic_block);

	if (dyn_fsync_kobj != NULL)
		kobject_put(dyn_fsync_kobj);

	if (fb_register_client(&notif))
		fb_unregister_client(&notif);

	pr_info("%s dynamic fsync unregistration complete\n", __func__);
}

module_init(dyn_fsync_init);
module_exit(dyn_fsync_exit);

MODULE_AUTHOR("andip71");
MODULE_DESCRIPTION("dynamic fsync - automatic fs sync optimization for msm8974");
MODULE_LICENSE("GPL v2");
