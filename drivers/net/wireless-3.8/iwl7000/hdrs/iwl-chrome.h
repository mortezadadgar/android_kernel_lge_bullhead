#ifndef __IWL_CHROME
#define __IWL_CHROME
/* This file is pre-included from the Makefile (cc command line) */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/vmalloc.h>
#include <net/genetlink.h>
#include <linux/crypto.h>
#include <linux/moduleparam.h>
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
#include <crypto/algapi.h>
#include <linux/pci.h>

/* get the CPTCFG_* preprocessor symbols */
#include <hdrs/config.h>

/* cfg80211 version specific backward compat code follows */
#ifdef CONFIG_WIRELESS_38
#define CFG80211_VERSION KERNEL_VERSION(3,8,0)
#else
#define CFG80211_VERSION LINUX_VERSION_CODE
#endif

/* mac80211 & backport */
#include <hdrs/mac80211-exp.h>
#include <hdrs/ieee80211.h>
#include <hdrs/mac80211-bp.h>
/* need to include mac80211 here, otherwise we get the regular kernel one */
#include <hdrs/mac80211.h>

/* include rhashtable this way to get our copy if another exists */
#include <linux/list_nulls.h>
#ifndef NULLS_MARKER
#define NULLS_MARKER(value) (1UL | (((long)value) << 1))
#endif
#include "linux/rhashtable.h"

/* artifacts of backports - never in upstream */
#define genl_info_snd_portid(__genl_info) (__genl_info->snd_portid)
#define NETLINK_CB_PORTID(__skb) NETLINK_CB(cb->skb).portid
#define netlink_notify_portid(__notify) __notify->portid

/* things that may or may not be upstream depending on the version */
#ifndef ETH_P_802_3_MIN
#define ETH_P_802_3_MIN 0x0600
#endif

#ifndef U32_MAX
#define U32_MAX		((u32)~0U)
#endif

#ifndef U8_MAX
#define U8_MAX		((u8)~0U)
#endif

#ifndef S8_MAX
#define S8_MAX		((s8)(U8_MAX>>1))
#endif

#ifndef S8_MIN
#define S8_MIN		((s8)(-S8_MAX - 1))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
/* backport IDR APIs */
static inline void iwl7000_idr_destroy(struct idr *idp)
{
	idr_remove_all(idp);
	idr_destroy(idp);
}
#define idr_destroy(idp) iwl7000_idr_destroy(idp)

static inline int idr_alloc(struct idr *idr, void *ptr, int start, int end,
			    gfp_t gfp_mask)
{
	int id, ret;

	do {
		if (!idr_pre_get(idr, gfp_mask))
			return -ENOMEM;
		ret = idr_get_new_above(idr, ptr, start, &id);
		if (!ret && id > end) {
			idr_remove(idr, id);
			ret = -ENOSPC;
		}
	} while (ret == -EAGAIN);

	return ret ? ret : id;
}

static inline void idr_preload(gfp_t gfp_mask)
{
}

static inline void idr_preload_end(void)
{
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
#define netdev_notifier_info_to_dev(ndev)	ndev
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0)
/* PCIe device capabilities flags have been renamed in (upstream)
 * commit d2ab1fa68c61f01b28ab0859a972c892d81f5d32 (PCI: Rename PCIe
 * capability definitions to follow convention).  This was just a
 * clean rename, without any functional changes.  We use one of the
 * renamed flags, so define it to the old one.
 */
#define PCI_EXP_DEVCTL2_LTR_EN PCI_EXP_LTR_EN

#define PTR_ERR_OR_ZERO(p) PTR_RET(p)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
#define __genl_const
static inline int
_genl_register_family_with_ops_grps(struct genl_family *family,
				    struct genl_ops *ops, size_t n_ops,
				    struct genl_multicast_group *mcgrps,
				    size_t n_mcgrps)
{
	int ret, i;

	ret = genl_register_family_with_ops(family, ops, n_ops);
	if (ret)
		return ret;
	for (i = 0; i < n_mcgrps; i++) {
		ret = genl_register_mc_group(family, &mcgrps[i]);
		if (ret) {
			genl_unregister_family(family);
			return ret;
		}
	}

	return 0;
}
#define genl_register_family_with_ops_groups(family, ops, grps)		\
	_genl_register_family_with_ops_grps((family),			\
					    (ops), ARRAY_SIZE(ops),	\
					    (grps), ARRAY_SIZE(grps))
#else
#define __genl_const const
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
#define ether_addr_equal_unaligned __iwl7000_ether_addr_equal_unaligned
static inline bool ether_addr_equal_unaligned(const u8 *addr1, const u8 *addr2)
{
	return memcmp(addr1, addr2, ETH_ALEN) == 0;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
#define kvfree __iwl7000_kvfree
static inline void kvfree(const void *addr)
{
	if (is_vmalloc_addr(addr))
		vfree(addr);
	else
		kfree(addr);
}

static inline u64 ktime_get_boot_ns(void)
{
	return ktime_to_ns(ktime_get_boottime());
}

/* interface name assignment types (sysfs name_assign_type attribute) */
#define NET_NAME_UNKNOWN	0	/* unknown origin (not exposed to userspace) */
#define NET_NAME_ENUM		1	/* enumerated by kernel */
#define NET_NAME_PREDICTABLE	2	/* predictably named by the kernel */
#define NET_NAME_USER		3	/* provided by user-space */
#define NET_NAME_RENAMED	4	/* renamed by user-space */

static inline struct net_device *
backport_alloc_netdev_mqs(int sizeof_priv, const char *name,
			  unsigned char name_assign_type,
			  void (*setup)(struct net_device *),
			  unsigned int txqs, unsigned int rxqs)
{
	return alloc_netdev_mqs(sizeof_priv, name, setup, txqs, rxqs);
}

#define alloc_netdev_mqs backport_alloc_netdev_mqs

#undef alloc_netdev
static inline struct net_device *
backport_alloc_netdev(int sizeof_priv, const char *name,
		      unsigned char name_assign_type,
		      void (*setup)(struct net_device *))
{
	return backport_alloc_netdev_mqs(sizeof_priv, name, name_assign_type,
					 setup, 1, 1);
}
#define alloc_netdev backport_alloc_netdev
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,2,0)
#include <crypto/scatterwalk.h>
#include <crypto/aead.h>

static inline struct scatterlist *scatterwalk_ffwd(struct scatterlist dst[2],
					    struct scatterlist *src,
					    unsigned int len)
{
	for (;;) {
		if (!len)
			return src;

		if (src->length > len)
			break;

		len -= src->length;
		src = sg_next(src);
	}

	sg_init_table(dst, 2);
	sg_set_page(dst, sg_page(src), src->length - len, src->offset + len);
	scatterwalk_crypto_chain(dst, sg_next(src), 0, 2);

	return dst;
}



struct aead_old_request {
	struct scatterlist srcbuf[2];
	struct scatterlist dstbuf[2];
	struct aead_request subreq;
};

static inline unsigned int iwl7000_crypto_aead_reqsize(struct crypto_aead *tfm)
{
	return crypto_aead_crt(tfm)->reqsize + sizeof(struct aead_old_request);
}
#define crypto_aead_reqsize iwl7000_crypto_aead_reqsize

static inline struct aead_request *
crypto_backport_convert(struct aead_request *req)
{
	struct aead_old_request *nreq = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct scatterlist *src, *dst;

	src = scatterwalk_ffwd(nreq->srcbuf, req->src, req->assoclen);
	dst = req->src == req->dst ?
	      src : scatterwalk_ffwd(nreq->dstbuf, req->dst, req->assoclen);

	aead_request_set_tfm(&nreq->subreq, aead);
	aead_request_set_callback(&nreq->subreq, aead_request_flags(req),
				  req->base.complete, req->base.data);
	aead_request_set_crypt(&nreq->subreq, src, dst, req->cryptlen,
			       req->iv);
	aead_request_set_assoc(&nreq->subreq, req->src, req->assoclen);

	return &nreq->subreq;
}

static inline int iwl7000_crypto_aead_encrypt(struct aead_request *req)
{
	return crypto_aead_encrypt(crypto_backport_convert(req));
}
#define crypto_aead_encrypt iwl7000_crypto_aead_encrypt

static inline int iwl7000_crypto_aead_decrypt(struct aead_request *req)
{
	return crypto_aead_decrypt(crypto_backport_convert(req));
}
#define crypto_aead_decrypt iwl7000_crypto_aead_decrypt

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(4,2,0) */

/* Note: this stuff is included in in chromeos-3.14 and 3.18.
 * Additionally, we check for <4.2, since that's when it was
 * added upstream.
 */
#if (LINUX_VERSION_CODE != KERNEL_VERSION(3,14,0)) &&	\
    (LINUX_VERSION_CODE != KERNEL_VERSION(3,18,0)) &&	\
    (LINUX_VERSION_CODE < KERNEL_VERSION(4,2,0))
static inline void aead_request_set_ad(struct aead_request *req,
				       unsigned int assoclen)
{
	req->assoclen = assoclen;
}

static inline void kernel_param_lock(struct module *mod)
{
	__kernel_param_lock();
}

static inline void kernel_param_unlock(struct module *mod)
{
	__kernel_param_unlock();
}
#endif /* !3.14 && <4.2 */

#ifndef list_first_entry_or_null
#define list_first_entry_or_null(ptr, type, member) \
	(!list_empty(ptr) ? list_first_entry(ptr, type, member) : NULL)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
#ifdef CONFIG_DEBUG_FS
struct dentry *iwl_debugfs_create_bool(const char *name, umode_t mode,
				       struct dentry *parent, bool *value);
#else
static inline struct dentry *
iwl_debugfs_create_bool(const char *name, umode_t mode,
			struct dentry *parent, bool *value)
{
	return ERR_PTR(-ENODEV);
}
#endif /* CONFIG_DEBUG_FS */
#define debugfs_create_bool iwl_debugfs_create_bool

#define tso_t __iwl7000_tso_t
struct tso_t {
	int next_frag_idx;
	void *data;
	size_t size;
	u16 ip_id;
	bool ipv6;
	u32 tcp_seq;
};

int tso_count_descs(struct sk_buff *skb);
void tso_build_hdr(struct sk_buff *skb, char *hdr, struct tso_t *tso,
		   int size, bool is_last);
void tso_start(struct sk_buff *skb, struct tso_t *tso);
void tso_build_data(struct sk_buff *skb, struct tso_t *tso, int size);

#endif /* < 4.4.0 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0))
static inline int
skb_ensure_writable(struct sk_buff *skb, int write_len)
{
	if (!pskb_may_pull(skb, write_len))
		return -ENOMEM;

	if (!skb_cloned(skb) || skb_clone_writable(skb, write_len))
		return 0;

	return pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
}
#endif

#ifndef NETIF_F_CSUM_MASK
#define NETIF_F_CSUM_MASK (NETIF_F_V4_CSUM | NETIF_F_V6_CSUM)
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0))
static inline int
pci_enable_msix_range(struct pci_dev *dev, struct msix_entry *entries,
		      int minvec, int maxvec)
{
	return -EOPNOTSUPP;
}
#endif

#if CFG80211_VERSION < KERNEL_VERSION(4, 1, 0) && \
	CFG80211_VERSION >= KERNEL_VERSION(3, 14, 0)
static inline struct sk_buff *
backport_cfg80211_vendor_event_alloc(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
				     int approxlen, int event_idx, gfp_t gfp)
{
	return cfg80211_vendor_event_alloc(wiphy, approxlen, event_idx, gfp);
}

#define cfg80211_vendor_event_alloc backport_cfg80211_vendor_event_alloc
#endif

#endif /* __IWL_CHROME */
