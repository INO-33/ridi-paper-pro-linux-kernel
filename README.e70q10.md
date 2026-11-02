# RIDI PAPER PRO E70Q10 kernel

This tree is based on NXP `o8.0.0_1.0.0-ga` and contains the Linux 4.9
bring-up for the i.MX6SL-based RIDI PAPER PRO (E70Q10).

The port currently includes:

- the E70Q10 device tree;
- appended-DTB and ATAG compatibility for the stock U-Boot;
- ED078KH3-compatible EPDC/PxP support and TPS65185 power sequencing;
- RC5T619/RN5T618 PMIC support;
- CYTTSP5 touch support;
- the stock-compatible i.MX I2C transfer behavior used during bring-up.

## Reproducible build

Use the Android 8 ARM GCC 4.9 toolchain:

```sh
export CROSS_COMPILE="$ANDROID_TOP/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.9/bin/arm-linux-androideabi-"
make ARCH=arm HOSTCFLAGS=-fcommon e70q10_defconfig
make ARCH=arm CROSS_COMPILE="$CROSS_COMPILE" HOSTCFLAGS=-fcommon -j6 \
    zImage imx6sl-e70q10.dtb
```

The stock U-Boot supplies ATAGs but no runtime DTB. Append
`arch/arm/boot/dts/imx6sl-e70q10.dtb` to `arch/arm/boot/zImage` when producing
the RAM-test kernel.

`HOSTCFLAGS=-fcommon` is required when the kernel's older in-tree DTC is built
with a modern host compiler.

## RAM-only safety policy

Development images must be tested with `fastboot boot`, never `fastboot
flash`. The internal eMMC must remain unmounted, or read-only when inspection
is necessary. Keep the external SD inserted for logs and recovery inputs.

The E70Q10 defconfig enables the recovery and diagnosis features required by
the current test environment:

- `CONFIG_ARM_APPENDED_DTB` and ATAG-to-DTB compatibility;
- i.MX watchdog support;
- pstore console capture;
- Magic SysRq and debugfs;
- USB configfs with CDC ACM support;
- `/proc/config.gz` for configuration recovery.

RAM-test boot images should add these runtime command-line settings:

```text
panic=10 oops=panic loglevel=7
```

The diagnostic initramfs must expose a USB CDC ACM shell, copy bounded logs to
the external SD, and provide `reboot -f`. A userspace watchdog should reboot
when Android or the diagnostic payload fails to reach its ready marker.

## Current limitations

The stock boot partition is about 6 MiB and cannot contain the current
untrimmed kernel. The stock system partition is also too small for a normal
Oreo system image. Do not repartition or write the internal eMMC during
bring-up; use a RAM-booted kernel and SD-backed Android partitions.

Extracted stock firmware, waveform data, Wi-Fi firmware, test logs, boot
images, object files, and local host tools are intentionally not tracked in
this source repository.
