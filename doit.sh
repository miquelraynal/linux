#!/usr/bin/env bash

set -e

TV=2020.02.3-083
BV=2020.02.3-307
sudo /usr/local/bin/mscc-install-pkg -t toolchains/$TV-toolchain mscc-toolchain-bin-$TV
sudo /usr/local/bin/mscc-install-pkg -t brsdk/$BV-brsdk mscc-brsdk-arm64-$BV
sudo /usr/local/bin/mscc-install-pkg -t brsdk/$BV-brsdk mscc-brsdk-mipsel-$BV

BUILD=build
# BSP=/opt/mscc/mscc-brsdk-arm64-$BV/arm64-armv8_a-linux-gnu
BSP=/opt/mscc/mscc-brsdk-mipsel-$BV/mipsel-mips32r2-linux-gnu
# export PATH=$BSP/x86_64-linux/bin:$PATH
export PATH=$BSP/smb/x86_64-linux/bin:$PATH

export ARCH=mips
# export CROSS_COMPILE=/opt/mscc/mscc-toolchain-bin-$TV/arm64-armv8_a-linux-gnu/bin/aarch64-linux-
export CROSS_COMPILE=/opt/mscc/mscc-toolchain-bin-$TV/mipsel-mips32r2-linux-gnu/bin/mipsel-linux-

mkdir -p $BUILD
if [ $# -gt 0 ]; then
    make O=$BUILD $*
else
    [ ! -e $BUILD/.config ] && make O=$BUILD mscc_vcoreiii_defconfig
    make O=$BUILD -j 8 -l 10
    [ ! -e $BUILD/rootfs.squashfs ] && cp $BSP/smb/rootfs.squashfs $BUILD/rootfs.squashfs
    mkimage -f vcoreiii.its $BUILD/vcoreiii.fit
    [ -d /tftpboot/$USER ] && cp -v $BUILD/vcoreiii.fit /tftpboot/$USER
fi
/bin/true
