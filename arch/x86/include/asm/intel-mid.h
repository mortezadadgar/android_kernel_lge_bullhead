/*
 * intel-mid.h: Intel MID specific setup code
 *
 * (C) Copyright 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_INTEL_MID_H
#define _ASM_X86_INTEL_MID_H

#include <linux/sfi.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

/*
 * Access to message bus through these 2 registers
 * in CUNIT(0:0:0) PCI configuration space.
 * MSGBUS_CTRL_REG(0xD0):
 *   31:24	= message bus opcode
 *   23:16	= message bus port
 *   15:8	= message bus address
 *   7:4	= message bus byte enables
 * MSGBUS_DTAT_REG(0xD4):
 *   hold the data for write or read
 */
#define PCI_ROOT_MSGBUS_CTRL_REG	0xD0
#define PCI_ROOT_MSGBUS_DATA_REG	0xD4
#define PCI_ROOT_MSGBUS_READ		0x10
#define PCI_ROOT_MSGBUS_WRITE		0x11
#define PCI_ROOT_MSGBUS_DWORD_ENABLE	0xf0

extern u32 board_id;
extern struct sfi_soft_platform_id spid;
extern int pci_mrst_init(void);
extern int get_gpio_by_name(const char *name);
extern void *get_oem0_table(void);
extern void intel_delayed_device_register(void *dev,
			void (*delayed_callback)(void *dev_desc));
extern int sdhci_pci_request_regulators(void);
extern void install_irq_resource(struct platform_device *pdev, int irq);
extern void intel_scu_device_register(struct platform_device *pdev);
extern struct devs_id *get_device_id(u8 type, char *name);
extern int __init sfi_parse_mrtc(struct sfi_table_header *table);
extern int sfi_mrtc_num;
extern struct sfi_rtc_table_entry sfi_mrtc_array[];
extern int intel_mid_create_property(const struct attribute *attr);
extern u32 intel_mid_msgbus_read32_raw(u32 cmd);
extern void intel_mid_msgbus_write32_raw(u32 cmd, u32 data);
extern u32 intel_mid_msgbus_read32(u8 port, u8 addr);
extern void intel_mid_msgbus_write32(u8 port, u8 addr, u32 data);

/*
 * Here defines the array of devices platform data that IAFW would export
 * through SFI "DEVS" table, we use name and type to match the device and
 * its platform data.
 */
struct devs_id {
	char name[SFI_NAME_LEN + 1];
	u8 type;
	u8 delay;
	void *(*get_platform_data)(void *info);
	void (*device_handler)(struct sfi_device_table_entry *pentry,
				struct devs_id *dev);
	/* Custom handler for devices */
	u8 trash_itp;/* true if this driver uses pin muxed with XDB connector */
};

#define SD_NAME_SIZE 16
/**
 * struct sd_board_info - template for device creation
 * @name: Initializes sdio_device.name; identifies the driver.
 * @bus_num: board-specific identifier for a given SDIO controller.
 * @board_ref_clock: Initializes sd_device.board_ref_clock;
 * @platform_data: Initializes sd_device.platform_data; the particular
 *      data stored there is driver-specific.
 *
 */
struct sd_board_info {
	char            name[SD_NAME_SIZE];
	int             bus_num;
	unsigned short  addr;
	u32             board_ref_clock;
	void            *platform_data;
};


/*
 * Medfield is the follow-up of Moorestown, it combines two chip solution into
 * one. Other than that it also added always-on and constant tsc and lapic
 * timers. Medfield is the platform name, and the chip name is called Penwell
 * we treat Medfield/Penwell as a variant of Moorestown. Penwell can be
 * identified via MSRs.
 */
enum intel_mid_cpu_type {
	INTEL_MID_CPU_CHIP_LINCROFT = 1,
	INTEL_MID_CPU_CHIP_PENWELL,
	INTEL_MID_CPU_CHIP_CLOVERVIEW,
	INTEL_MID_CPU_CHIP_TANGIER,
};

extern enum intel_mid_cpu_type __intel_mid_cpu_chip;

#ifdef CONFIG_X86_INTEL_MID

static inline enum intel_mid_cpu_type intel_mid_identify_cpu(void)
{
	return __intel_mid_cpu_chip;
}

#else /* !CONFIG_X86_INTEL_MID */

#define intel_mid_identify_cpu()    (0)

#endif /* !CONFIG_X86_INTEL_MID */

enum intel_mid_timer_options {
	INTEL_MID_TIMER_DEFAULT,
	INTEL_MID_TIMER_APBT_ONLY,
	INTEL_MID_TIMER_LAPIC_APBT,
};

enum {
	/* 0 is "unknown" so that you can write if (!mrst_platform_id()) */
	MRST_PLATFORM_LANFORD = 1,
	MRST_PLATFORM_SHCDK,
	MRST_PLATFORM_AAVA_SC,
};
extern int mrst_platform_id(void);

extern enum intel_mid_timer_options intel_mid_timer_options;

enum {
	MFLD_BOARD_UNKNOWN = 0,
	MFLD_BOARD_CDK     = 1,
	MFLD_BOARD_PR2     = 2,
};
extern int mfld_board_type(void);

/*
    MSIC Pin, Pin#, ID,     CDK     BKB      BKB      BKB    BKB   BKB
    --------  ----- ---   (Fab-B) Pr2Proto Pr2Volume PR2PnP  PR3  PR3PnP
    GPIO0LV0  L13   BID_0    0       1        1        0      1     0
    GPIO0LV1  H12   BID_1    1       0        0        0      0     0
    GPIO0LV2  K13   BID_2    0       1        1        0      1     0
    GPIO0LV3  J11   BID_3    0       0        0        1      0     1
    GPIO0LV4  K14   FAB_ID0  1       1        0        0      1     1
    GPIO0LV5  J15   FAB_ID1  0       0        1        0      1     0


    MSIC Pin, Pin#, ID,     Aava     RR    RR    RR    Joki
    --------  ----- ---   FF-Proto  DV10  DV20  DV21   EV20
    GPIO0LV0  L13   BID_0    0       1     1     1      0
    GPIO0LV1  H12   BID_1    1       0     0     0      1
    GPIO0LV2  K13   BID_2    1       1     1     1      1
    GPIO0LV3  J11   BID_3    0       0     0     0      0
    GPIO0LV4  K14   FAB_ID0  x       0     1     0      1
    GPIO0LV5  J15   FAB_ID1  0       1     0     0      0


 */

enum {
	MFLD_BID_UNKNOWN     = 0,
	MFLD_BID_CDK         = 0x12,
	MFLD_BID_AAVA        = 0x06,
	MFLD_BID_JOKI_EV20   = 0x16,
	MFLD_BID_PR2_PROTO   = 0x15,
	MFLD_BID_PR2_PNP     = 0x08,
	MFLD_BID_PR2_VOLUME  = 0x25,
	MFLD_BID_PR3         = 0x35,
	MFLD_BID_PR3_PNP     = 0x18,
	MFLD_BID_RR_DV10     = 0x25,
	MFLD_BID_RR_DV20     = 0x15,
	MFLD_BID_RR_DV21     = 0x05,
	MFLD_BID_LEX   = 0xe0000025,
	CLVT_BID_VV          = 0x08,
	CLVT_BID_PR0         = 0x04
};
extern u32 mfld_board_id(void);

/* Right now board id information is updated from sfi table print*/

enum {
	CTP_BID_UNKNOWN = 0,
	CTP_BID_VV	= 0x08,
	CTP_BID_PR0	= 0x04
};
extern u32 ctp_board_id(void);

#define spid_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr   = {				\
		.name = __stringify(_name),	\
		.mode = 0444,			\
	},					\
	.show   = _name##_show,			\
}

/* Customer_ID table */
enum {
	CUSTOMER_INTEL,
	CUSTOMER_AT,
	CUSTOMER_OR,
	CUSTOMER_TK,
	CUSTOMER_RSVD,
	CUSTOMER_UNKNOWN = 0xFFFF
};

/* Vendor_ID table */
enum {
	VENDOR_INTEL,
	VENDOR_LN,
	VENDOR_MO,
	VENDOR_RSVD,
	VENDOR_UNKNOWN = 0xFFFF
};

/* Manufacturer_ID table for Vendor_ID == VENDOR_INTEL */
enum {
	MANUFACTURER_FC1,
	MANUFACTURER_FC2,
	MANUFACTURER_FC3,
	MANUFACTURER_RSVD,
	MANUFACTURER_UNKNOWN = 0xFFFF
};

/* Platform_Family_ID table for Vendor_ID == VENDOR_INTEL */
enum {
	INTEL_MFLD_PHONE  = 0x0000,
	INTEL_MFLD_TABLET = 0x8000,
	INTEL_CLVTP_PHONE = 0x0002,
	INTEL_CLVT_TABLET = 0x8002,
	INTEL_MRFL_PHONE  = 0x0004,
	INTEL_MRFL_TABLET = 0x8004,
	INTEL_LFLD_PHONE  = 0x0006,
	INTEL_LFLD_TABLET = 0x8006,
	INTEL_PLATFORM_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_MFLD_PHONE */
enum {
	INTEL_MFLDP_BB15_PRO = 0x0000,
	INTEL_MFLDP_BB15_ENG = 0x8000,
	INTEL_MFLDP_BB20_PRO = 0x0001,
	INTEL_MFLDP_BB20_ENG = 0x8001,
	INTEL_MFLDP_OR_PRO   = 0x0002,
	INTEL_MFLDP_OR_ENG   = 0x8002,
	INTEL_MFLDP_AT_PRO   = 0x0003,
	INTEL_MFLDP_AT_ENG   = 0x8003,
	INTEL_MFLDP_LEX_PRO  = 0x0004,
	INTEL_MFLDP_LEX_ENG  = 0x8004,
	INTEL_MFLDP_UNKNOWN  = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_MFLD_TABLET */
enum {
	INTEL_MFLDT_RR_PRO  = 0x0000,
	INTEL_MFLDT_RR_ENG  = 0x8000,
	INTEL_MFLDT_JK_PRO  = 0x0001,
	INTEL_MFLDT_JK_ENG  = 0x8001,
	INTEL_MFLDT_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_CLVTP_PHONE */
enum {
	INTEL_CLVTPP_RHB_PRO = 0x0000,
	INTEL_CLVTPP_RHB_ENG = 0x8000,
	INTEL_CLVTPP_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_CLVT_TABLET */
enum {
	INTEL_CLVTT_TBD_PRO = 0x0000,
	INTEL_CLVTT_TBD_ENG = 0x8000,
	INTEL_CLVTT_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_MRFL_PHONE */
enum {
	INTEL_MRFLP_TBD_PRO = 0x0000,
	INTEL_MRFLP_TBD_ENG = 0x8000,
	INTEL_MRFLP_UNKNOWN = 0xFFFF
};

/* Product_Line_ID table for Platform_Family_ID == INTEL_MRFL_TABLET */
enum {
	INTEL_MRFLT_TBD_PRO = 0x0000,
	INTEL_MRFLT_TBD_ENG = 0x8000,
	INTEL_MRFLT_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLDP_BB15 */
enum {
	MFLDP_BB15_PR20, /* CRAK C0 */
	MFLDP_BB15_PR31, /* CRAK D0 */
	MFLDP_BB15_PR32, /* CRAK D0 */
	MFLDP_BB15_PR33, /* CRAK D1 - 1.6GHz */
	MFLDP_BB15_PR34, /* CRAK D1 - 1.6GHz, alt eMMC, DDR2 */
	MFLDP_BB15_PR35, /* CRAK D1 - 1.6GHz, alt eMMC, DDR2 */
	MFLDP_BB15_PR36, /* CRAK D1 - 1.6GHz, alt eMMC, DDR2, MSIC C2 */
	MFLDP_BB15_PR40, /* CRAK D1 - 2.0GHz */
	MFLDP_BB15_PR2A,
	MFLDP_BB15_PR3A,
	MFLDP_BB15_PR3B,
	MFLDP_BB15_4MVV,
	MFLDP_BB15_4MSV,
	MFLDP_BB15_ICDK,
	MFLDP_BB15_RSVD,
	MFLDP_BB15_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLDP_BB20 */
enum {
	MFLDP_BB20_TBD,
	MFLDP_BB20_RSVD,
	MFLDP_BB20_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLDP_OR */
enum {
	MFLDP_OR_NHDV1,    /* CRAK D0 - 1.6G */
	MFLDP_OR_NHDV2,    /* CRAK D1 - 1.6G */
	MFLDP_OR_NHDV3,    /* CRAK D1 - 1.6G */
	MFLDP_OR_NHDV31R,  /* CRAK D1 - 1.6G */
	MFLDP_OR_NHDV31A,  /* CAAK D1 - 1.6G */
	MFLDP_OR_NHDV30F,  /* CRAK D1 - 2.0G */
	MFLDP_OR_NHDV31A1, /* CAAK D1 - 2.0G */
	MFLDP_OR_NHDV30D,  /* CRAK D2 - 1.6G */
	MFLDP_OR_NHDV30G,  /* CRAK D2 - 2.0G */
	MFLDP_OR_NHDV31A2, /* CAAK D2 - 1.6G */
	MFLDP_OR_NHDV31A3, /* CAAK D2 - 2.0G */
	MFLDP_OR_NHDV30E,  /* CRAK D1, Samsung eMMC for part quals */
	MFLDP_OR_RSVD,
	MFLDP_OR_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLDP_AT */
enum {
	MFLDP_AT_LA, /* CAAK D1 */
	MFLDP_AT_LA_RSVD,
	MFLDP_AT_LA_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLDP_LEX */
enum {
	MFLDP_LEX_PR11, /* RYS/PNW 1GHz CREK D1 */
	MFLDP_LEX_PR1M, /* RYS/PNW 1GHz CREK D1 */
	MFLDP_LEX_PR21, /* RYS/PNW 1GHz CSEK D1 */
	MFLDP_LEX_PR2M, /* RYS/PNW 1GHz CSEK D1 */
	MFLDP_LEX_PR30, /* RYS/PNW 1GHz CSEK D1 */
	MFLDP_LEX_RSVD,
	MFLDP_LEX_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLDT_RR */
enum {
	MFLDT_RR_DV10, /* CRAK D0 */
	MFLDT_RR_DV15, /* CRAK D0/D1 */
	MFLDT_RR_DV20, /* CRAK D1 */
	MFLDT_RR_DV21, /* CRAK D1 */
	MFLDT_RR_RSVD,
	MFLDT_RR_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MFLDT_JK */
enum {
	MFLDT_JK_EV10, /* CRAK D0 */
	MFLDT_JK_EV20, /* CRAK D0 */
	MFLDT_JK_DV10, /* CRAK D1 */
	MFLDT_JK_RSVD,
	MFLDT_JK_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_CLVTPP_RHB */
enum {
	CLVTPP_RHB_CCVV0,  /* Clover City VV0 FAB A CLV/CLV+ A0*/
	CLVTPP_RHB_CCVV1,  /* Clover City VV1 FAB B CLV+ A0*/
	CLVTPP_RHB_CCVV2,  /* Clover City VV2 FAB C CLV+ A0*/
	CLVTPP_RHB_CLEV,   /* Clover Lake CRB EV */
	CLVTPP_RHB_PR01,   /* RHB PR0.1 CLV A0 C-CLASS */
	CLVTPP_RHB_PR02,   /* RHB PR0.2 CLV A0 C-CLASS */
	CLVTPP_RHB_PMPR10, /* CLV+ A0 */
	CLVTPP_RHB_CCPVV1, /* Clover City Pre-VV1 Fab B CLV+ A0 */
	CLVTPP_RHB_PPR10,  /* RHB Pre-PR1.0 CLV A0 C- CLASS */
	CLVTPP_RHB_MPR10,  /* RHB Macro PR1.0 CLV+ A0 */
	CLVTPP_RHB_PR10,   /* RHB PR1.0 CLV+ A0 C-CLASS */
	CLVTPP_RHB_MPR15,  /* RHB Macro PR1.5 CLV+ A0 */
	CLVTPP_RHB_PR15,   /* RHB PR1.5 CLV+ A0 C-CLASS */
	CLVTPP_RHB_MPR20,  /* RHB Macro PR2.0 CLV+ B0 */
	CLVTPP_RHB_PR20,   /* RHB PR2.0 CLV+ B0 C-CLASS */
	CLVTPP_RHB_MPR30,  /* RHB Macro PR3.0 CLV+ B0 */
	CLVTPP_RHB_CCVV3,  /* Clover City VV3 FAB D CLV+ A0 */
	CLVTPP_RHB_PR30,   /* RHB PR3.0 CLV+ B0 C-CLASS */
	CLVTPP_RHB_DV1,    /* RHB Dv1 */
	CLVTPP_RHB_RSVD,
	CLVTPP_RHB_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_CLVTT_TBD */
enum {
	CLVTT_TBD_CLEVA, /* Clover Lake EV - CRB - FAB A */
	CLVTT_TBD_CLEVB, /* Clover Lake EV - CRB - FAB B */
	CLVTT_TBD_CLEVC, /* Clover Lake EV - CRB - FAB C */
	CLVTT_TBD_CLEVD, /* Clover Lake EV - CRB - FAB D */
	CLVTT_TBD_RSVD,
	CLVTT_TBD_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MRFLP_TBD */
enum {
	MRFLP_TBD_SRVVA, /* SilverRidge VV FAB A */
	MRFLP_TBD_SRVVB, /* SilverRidge VV FAB B */
	MRFLP_TBD_SRVVC, /* SilverRidge VV FAB C */
	MRFLP_TBD_SRVVD, /* SilverRidge VV FAB D */
	MRFLP_TBD_SRSVA, /* SilverRidge SV FAB A */
	MRFLP_TBD_SRSVB, /* SilverRidge SV FAB B */
	MRFLP_TBD_SRSVC, /* SilverRidge SV FAB C */
	MRFLP_TBD_SRSVD, /* SilverRidge SV FAB D */
	MRFLP_TBD_RSVD,
	MRFLP_TBD_UNKNOWN = 0xFFFF
};

/* Hardware_ID table for Product_Line_ID == INTEL_MRFLT_TBD */
enum {
	MRFLT_TBD_TBD,
	MRFLT_TBD_RSVD,
	MRFLT_TBD_UNKNOWN = 0xFFFF
};

/*
 * Penwell uses spread spectrum clock, so the freq number is not exactly
 * the same as reported by MSR based on SDM.
 * CLVP A0 has 100MHz FSB and CLVP B0 has 133MHz FSB.
 */
#define FSB_FREQ_83SKU		83200
#define FSB_FREQ_100SKU		99840
#define FSB_FREQ_133SKU		133000

/* Bus Select SoC Fuse value */
#define BSEL_SOC_FUSE_MASK	0x7
#define BSEL_SOC_FUSE_001	0x1 /* FSB 133MHz */
#define BSEL_SOC_FUSE_101	0x5 /* FSB 100MHz */
#define BSEL_SOC_FUSE_111	0x7 /* FSB 83MHz */

#define SFI_MTMR_MAX_NUM 8
#define SFI_MRTC_MAX	8

extern unsigned char hsu_dma_enable;

extern struct console early_mrst_console;
extern void mrst_early_console_init(void);
extern struct console early_mrfld_console;
extern void mrfld_early_console_init(void);

#ifdef CONFIG_SERIAL_MFD_HSU_CONSOLE_PORT
extern struct console early_hsu_console;
extern void hsu_early_console_init(void);
#endif

extern struct console early_pti_console;

extern void intel_scu_devices_create(int bus_id);
extern void intel_scu_devices_destroy(int bus_id);
extern void *cloverview_usb_otg_get_pdata(void);

/* VRTC timer */
#define MRST_VRTC_MAP_SZ	(1024)
/*#define MRST_VRTC_PGOFFSET	(0xc00) */

extern void intel_mid_rtc_init(void);

extern int mrst_pmu_pci_set_power_state(struct pci_dev *pdev,
					pci_power_t state);

#ifdef CONFIG_X86_MDFLD
extern int pmu_pci_set_power_state(struct pci_dev *pdev, pci_power_t state);
extern pci_power_t pmu_pci_choose_state(struct pci_dev *pdev);
#endif /* !CONFIG_X86_MDFLD */

#ifdef CONFIG_INTEL_MID_RAM_CONSOLE
extern void ram_consle_reserve_memory(void);
#else
static inline void ram_consle_reserve_memory(void){ }
#endif/* !CONFIG_INTEL_MID_RAM_CONSOLE*/

#ifdef CONFIG_X86_MRST
extern int mrst_pmu_s0i3_entry(void);
extern void mrst_pmu_disable_msi(void);
extern u32 mrst_pmu_msi_is_disabled(void);
extern void mrst_pmu_enable_msi(void);
extern void mrst_reserve_memory(void);
#else
static inline void mrst_reserve_memory(void) { }
#endif	/* !CONFIG_X86_MRST */

#include <linux/cpuidle.h>
extern int mrst_s0i3(struct cpuidle_device *dev, struct cpuidle_state *state);
extern void mrst_s0i3_resume(void);
extern int mrst_pmu_invalid_cstates(void);
extern const char s0i3_trampoline_data[], s0i3_trampoline_data_end[];

extern int get_force_shutdown_occured(void);

extern const struct atomisp_platform_data *intel_get_v4l2_subdev_table(void);

#ifdef CONFIG_X86_MRFLD
enum intel_mrfl_sim_type {
	INTEL_MRFL_CPU_SIMULATION_NONE = 0,
	INTEL_MRFL_CPU_SIMULATION_VP,
	INTEL_MRFL_CPU_SIMULATION_SLE,
};

extern enum intel_mrfl_sim_type __intel_mrfl_sim_platform;

static inline enum intel_mrfl_sim_type intel_mrfl_identify_sim(void)
{
	return __intel_mrfl_sim_platform;
}

#else /* !CONFIG_X86_MRFLD */
#define intel_mrfl_identify_sim()	(0)
#endif /* !CONFIG_X86_MRFLD */
#define INTEL_MID_IRQ_OFFSET 0x100
#endif /* _ASM_X86_INTEL_MID_H */
