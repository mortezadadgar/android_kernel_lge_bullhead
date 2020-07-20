#!/usr/bin/sh
date=$(date +%m%d%H%M)
cd $HOME/bullhead
echo 'Kernel Compilation Started!' |boxes -d stone
make shadow_defconfig
make CROSS_COMPILE=aarch64-linux-gnu- -j 4
cp arch/arm64/boot/Image.gz-dtb $HOME/AnyKernel
cd $HOME/AnyKernel
echo 'Zipping!' |boxes -d stone
zip Shadow-${date}.zip -r * -x \*.zip
echo 'Done!' |boxes -d stone
