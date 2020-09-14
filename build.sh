#!/bin/bash
date=$(date +%m%d%H%M)
branch_name=$(git rev-parse --abbrev-ref HEAD)
last_commit=$(git rev-parse --verify --short=8 HEAD)
version="msm44-${date}-${last_commit}"
cpus=$(nproc --all)
export LOCALVERSION="-Shadow-Kernel-44-${branch_name}/${last_commit}"
RED='\033[0;31m'
Green='\033[0;32m'
Yellow='\033[0;33m'
Blue='\033[0;34m'
Color_Off='\033[0m'

function die()
{
	echo -e ${RED}
	echo "${1}" | boxes -d stone
	exit
}

cd $HOME/bullhead
echo -e ${Blue}
echo "Kernel Compilation Started!" |boxes -d stone
echo -e ${Color_Off}
# making defconfig
make bullhead_defconfig
if [ ! $? -eq 0 ]; then
	die "Defconfig Generation Failed!"
fi

# making kernel
make CROSS_COMPILE=aarch64-linux-gnu- -j ${cpus}
if [ ! $? -eq 0 ]; then
	die "Kernel Compilation failed!"
fi

# AnyKernel
cp arch/arm64/boot/Image.gz-dtb $HOME/AnyKernel
cd $HOME/AnyKernel
echo -e ${Blue}
echo "Zipping!" |boxes -d stone
echo -e ${Color_Off}
zip ${version}.zip -r * -x \*.zip
echo -e ${Blue}
echo "Done! -> ${version}.zip" |boxes -d stone
echo -e ${Color_Off}
exit
