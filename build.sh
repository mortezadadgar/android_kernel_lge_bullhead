#!/bin/sh

# Variables
branch_name=$(git rev-parse --abbrev-ref HEAD)
last_commit=$(git rev-parse --verify --short=8 HEAD)
export LOCALVERSION="-Shadow-Kernel-${branch_name}/${last_commit}"
cpus=$(nproc --all)
Red='\033[0;31m'
Blue='\033[0;34m'
color_Off='\033[0m'
defconfig="shadow_defconfig"
spaces="  "
cross_compile="aarch64-linux-gnu-"
ccache="$(which ccache)"

die() {
	echo "$spaces | $Red${1} |"
	exit 1
}

# fire it!
echo "\n$spaces| ${Blue}Kernel Compilation Started!${color_Off} |"

# make defconfig
if ! make -s "$defconfig"; then
	die "Defconfig Generation Failed!"
fi

# make kernel
if ! make -j "${cpus}" \
	CROSS_COMPILE="$cross_compile" \
	CC="$ccache ${cross_compile}gcc"; then
	die "Kernel Compilation failed!"
fi

# set version
date=$(date +%m%d%H%M)
version="Shadow-${date}-${last_commit}"

# create ramdisk
cp arch/arm64/boot/Image.gz-dtb "$HOME"/bullhead/AnyKernel
cd "$HOME"/bullhead/AnyKernel || die "cd to anykernel dir failed"
echo "$spaces| ${Blue}Zipping!${color_Off} |"
zip "$version".zip -r ./* -x \*.zip
echo "$spaces| ${Blue}Done! -> ""$version"".zip${color_Off} |"

notify-send "Kernel" "Compilation is done!"
