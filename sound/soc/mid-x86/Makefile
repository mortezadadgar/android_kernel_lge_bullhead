snd-soc-sst-platform-objs := sst_platform.o
snd-soc-mfld-machine-objs := mfld_machine.o 

# Medfield board
obj-$(CONFIG_SND_SST_PLATFORM) += snd-soc-sst-platform.o
ifeq ($(CONFIG_SND_BYT_AK4614),y)
obj-$(CONFIG_SND_SST_PLATFORM) += mid_ssp.o
endif
obj-$(CONFIG_SND_MFLD_MACHINE) += snd-soc-mfld-machine.o

# Baytrail board
snd-soc-byt-ak-machine-objs := byt_bin_ak4614.o byt_ia_ak4614.o
obj-$(CONFIG_SND_BYT_AK4614) += snd-soc-byt-ak-machine.o
snd-soc-byt-rt-machine-objs := byt_bl_rt5642.o
obj-$(CONFIG_SND_BYT_RT5642) += snd-soc-byt-rt-machine.o
snd-soc-byt-rambi-machine-objs := byt_rambi_max98090.o
obj-$(CONFIG_SND_BYT_MAX98090) += snd-soc-byt-rambi-machine.o

# SST Compress framework driver
snd-compress-sst-objs := compress_sst.o
obj-$(CONFIG_SND_COMPRESS_SST) += snd-compress-sst.o

# DSP driver
obj-$(CONFIG_SND_INTEL_SST) += sst/
ifeq ($(CONFIG_SND_BYT_AK4614),y)
obj-$(CONFIG_SND_INTEL_SST) += ssp/
endif

# Audio Comms
snd-soc-comms-machine-objs := mfld_comms_machine.o
snd-soc-ssp-objs := mfld_ssp.o
obj-$(CONFIG_SND_MFLD_COMMS_MACHINE) += mfld_comms_machine.o
obj-$(CONFIG_SND_SOC_SSP) += mid_ssp.o
