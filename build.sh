#!/bin/sh

# Variables
branch_name=$(git rev-parse --abbrev-ref HEAD)
last_commit=$(git rev-parse --verify --short=8 HEAD)
export LOCALVERSION="-Shadow-Kernel-${branch_name}/${last_commit}"
cpus=$(nproc --all)
Red='\033[0;31m'
Blue='\033[0;34m'
Color_Off='\033[0m'
defconfig="shadow_defconfig"
spaces="  "

die()
{
	echo "$spaces | $Red${1} |"
	exit 1
}

# fire it!
echo "\n$spaces| ${Blue}Kernel Compilation Started!${Color_Off} |"

# make defconfig
if ! make -s "$defconfig"; then
	die "Defconfig Generation Failed!"
fi

# make kernel
if ! make CROSS_COMPILE=aarch64-linux-gnu- -j "${cpus}"; then
	die "Kernel Compilation failed!"
fi

# set version
date=$(date +%m%d%H%M)
version="Shadow-${date}-${last_commit}"

# create ramdisk
cp arch/arm64/boot/Image.gz-dtb "$HOME"/bullhead/AnyKernel
cd "$HOME"/bullhead/AnyKernel || die "cd to anykernel dir failed"
echo "$spaces| ${Blue}Zipping!${Color_Off} |"
zip "$version".zip -r ./* -x \*.zip
echo "$spaces| ${Blue}Done! -> ""$version"".zip${Color_Off} |"
