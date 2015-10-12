#
# Makefile for linux kernel
#

# ARM64 maps efi runtime services in userspace addresses
# which don't have KASAN shadow. So dereference of these addresses
# in efi_call_virt() will cause crash if this code instrumented.
#
KASAN_SANITIZE_runtime-wrappers.o	:= n

obj-y					+= efi.o vars.o
obj-$(CONFIG_EFI_VARS)			+= efivars.o
obj-$(CONFIG_EFI_VARS_PSTORE)		+= efi-pstore.o
