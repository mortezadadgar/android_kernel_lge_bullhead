#!/bin/zsh

# Variables
branch_name=$(git rev-parse --abbrev-ref HEAD)
last_commit=$(git rev-parse --verify --short=8 HEAD)
export LOCALVERSION="${branch_name}/${last_commit}"
cpus=$(nproc --all)
color_Off='\033[0m'
defconfig="bullhead_defconfig"
spaces="  "
cross_compile="aarch64-linux-gnu-"
ccache="$(which ccache)"

die() {
	print -P "$spaces | %F{red}${1} |$f"
	exit 1
}

# fire it!
print -P "\n$spaces| %F{blue}Kernel Compilation Started! |%f"

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
print -P "$spaces| %F{blue}Zipping!%f |"
zip "$version".zip -r ./* -x \*.zip
print -P "$spaces| %F{green}Done!%f -> %F{blue}""$version"".zip%f |"

notify-send "Kernel" "Compilation is done!"
