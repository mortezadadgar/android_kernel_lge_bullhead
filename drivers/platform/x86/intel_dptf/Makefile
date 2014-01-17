################################################################################
## This file is provided under a dual BSD/GPLv2 license.  When using or
## redistributing this file, you may do so under either license.
##
## GPL LICENSE SUMMARY
##
## Copyright (c) 2013 Intel Corporation All Rights Reserved
##
## This program is free software; you can redistribute it and/or modify it under
## the terms of version 2 of the GNU General Public License as published by the
## Free Software Foundation.
##
## This program is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
## FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
## details.
##
## You should have received a copy of the GNU General Public License along with
## this program; if not, write to the Free Software  Foundation, Inc.,
## 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
## The full GNU General Public License is included in this distribution in the
## file called LICENSE.GPL.
##
## BSD LICENSE
##
## Copyright (c) 2013 Intel Corporation All Rights Reserved
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright notice, this
##   list of conditions and the following disclaimer.
## * Redistributions in binary form must reproduce the above copyright notice,
##   this list of conditions and the following disclaimer in the documentation
##   and/or other materials provided with the distribution.
## * Neither the name of Intel Corporation nor the names of its contributors may
##   be used to endorse or promote products derived from this software without
##   specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
##
################################################################################

obj-$(CONFIG_INTEL_DPTF)      += esif_lf_driver.o
obj-$(CONFIG_INTEL_DPTF_ACPI) += dptf_acpi.o
obj-$(CONFIG_INTEL_DPTF_CPU)  += dptf_cpu.o
obj-$(CONFIG_INTEL_DPTF_PCH)  += dptf_pch.o

ccflags-y += -g
ccflags-y += -Wno-multichar -Wall
ccflags-y += -DESIF_ATTR_KERNEL
ccflags-y += -DESIF_ATTR_OS_LINUX
ccflags-y += -DESIF_ATTR_IOSF

# Participants (NATIVE CODE ONLY)
dptf_acpi-objs	:= dptf_acpi_os_lin.o
dptf_acpi-objs	+= esif_debug.o
dptf_cpu-objs   := dptf_cpu_os_lin.o
dptf_cpu-objs   += esif_debug.o
dptf_pch-objs   := dptf_pch_os_lin.o
dptf_pch-objs   += esif_debug.o

# OS Specific
esif_lf_driver-objs := esif_lf_os_lin.o
esif_lf_driver-objs += esif_ipc_os_lin.o

# Lower Framework R0
esif_lf_driver-objs += esif_lf.o
esif_lf_driver-objs += esif_lf_action.o
esif_lf_driver-objs += esif_lf_action_acpi.o
esif_lf_driver-objs += esif_lf_action_code.o
esif_lf_driver-objs += esif_lf_action_const.o
esif_lf_driver-objs += esif_lf_action_mbi.o
esif_lf_driver-objs += esif_lf_action_mmio.o
esif_lf_driver-objs += esif_lf_action_msr.o
esif_lf_driver-objs += esif_lf_action_systemio.o
esif_lf_driver-objs += esif_lf_action_var.o
esif_lf_driver-objs += esif_lf_command.o
esif_lf_driver-objs += esif_lf_cpc.o
esif_lf_driver-objs += esif_lf_data.o
esif_lf_driver-objs += esif_lf_dsp.o
esif_lf_driver-objs += esif_lf_event.o
esif_lf_driver-objs += esif_lf_pm.o
esif_lf_driver-objs += esif_lf_poll.o
esif_lf_driver-objs += esif_lf_primitive.o

# Common Framework
esif_lf_driver-objs += esif_debug.o
esif_lf_driver-objs += esif_hash_table.o
esif_lf_driver-objs += esif_ipc.o
esif_lf_driver-objs += esif_link_list.o
esif_lf_driver-objs += esif_queue.o
