#!/bin/bash

sudo qemu-system-arm \
    -M vexpress-a9 \
    -m 512M \
    -kernel ./out_vexpress_3_14/arch/arm/boot/zImage \
    -nographic \
    -append "root=/dev/mmcblk0  console=ttyAMA0" \
    -sd ./a9rootfs.ext3
