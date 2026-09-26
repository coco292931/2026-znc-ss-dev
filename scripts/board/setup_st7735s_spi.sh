#!/bin/sh
set -eu

SPI_DEVICE="${ST7735S_SPI_DEVICE:-}"
SPI_NODE="${ST7735S_SPI_NODE:-}"
SYSFS_DEVICE="/sys/bus/spi/devices/${SPI_DEVICE}"
PINMUX_TOOL="${ST7735S_PINMUX_TOOL:-/home/root/smartcar_pinmux}"
TFT_MODULE="${ST7735S_TFT_MODULE:-/home/root/key/TFT18_dev.ko}"

if [ ! -x "${PINMUX_TOOL}" ]; then
    echo "Missing pinmux tool: ${PINMUX_TOOL}" >&2
    exit 1
fi

# GPIO60/61/62 are SPI1 CLK/MISO/MOSI. GPIO63 is TFT CS and GPIO25 is
# IMU CS; both remain GPIO outputs and are mutually selected by rewrite.
"${PINMUX_TOOL}" spi1-screen

if [ -z "${SPI_DEVICE}" ]; then
    if [ -d /sys/bus/spi/devices/spi1.0 ]; then
        SPI_DEVICE="spi1.0"
    elif [ -d /sys/bus/spi/devices/spi1.2 ]; then
        SPI_DEVICE="spi1.2"
    else
        echo "Missing SPI device: spi1.0 or spi1.2" >&2
        exit 1
    fi
fi
if [ -z "${SPI_NODE}" ]; then
    SPI_NODE="/dev/spidev${SPI_DEVICE#spi}"
fi
SYSFS_DEVICE="/sys/bus/spi/devices/${SPI_DEVICE}"

if [ ! -d "${SYSFS_DEVICE}" ]; then
    echo "Missing SPI device: ${SYSFS_DEVICE}" >&2
    exit 1
fi
if [ ! -d /sys/bus/spi/drivers/spidev ]; then
    echo "The kernel spidev driver is unavailable" >&2
    exit 1
fi

CURRENT_DRIVER=""
if [ -L "${SYSFS_DEVICE}/driver" ]; then
    CURRENT_DRIVER="$(basename "$(readlink "${SYSFS_DEVICE}/driver")")"
fi
if [ "${CURRENT_DRIVER}" = "loongson,tft-driver" ]; then
    if [ "${SPI_DEVICE}" != "spi1.2" ]; then
        echo "Refusing to replace ${SPI_DEVICE} bound to ${CURRENT_DRIVER}" >&2
        exit 1
    fi
    if [ ! -f "${TFT_MODULE}" ]; then
        echo "Missing TFT module source: ${TFT_MODULE}" >&2
        exit 1
    fi
    rmmod TFT18_dri 2>/dev/null || true
    rmmod TFT18_dev 2>/dev/null || true
    PATCHED_MODULE="/run/TFT18_dev_spidev.ko"
    cp -f "${TFT_MODULE}" "${PATCHED_MODULE}"
    PATCH_OFFSET="${ST7735S_TFT_MODALIAS_OFFSET:-1616}"
    EXPECTED="/tmp/tft_modalias_expected.$$"
    printf '%s' 'loongson,tft-driver' > "${EXPECTED}"
    if ! dd if="${PATCHED_MODULE}" bs=1 skip="${PATCH_OFFSET}" count=19 2>/dev/null |
        tr -d '\000' | cmp -s "${EXPECTED}" -; then
        rm -f "${EXPECTED}" "${PATCHED_MODULE}"
        echo "Unexpected TFT module layout; refusing modalias patch" >&2
        exit 1
    fi
    printf 'spidev' | dd of="${PATCHED_MODULE}" bs=1 seek="${PATCH_OFFSET}" conv=notrunc 2>/dev/null
    dd if=/dev/zero of="${PATCHED_MODULE}" bs=1 seek=$((PATCH_OFFSET + 6)) count=13 conv=notrunc 2>/dev/null
    rm -f "${EXPECTED}"
    insmod "${PATCHED_MODULE}"
    CURRENT_DRIVER=""
    if [ -L "${SYSFS_DEVICE}/driver" ]; then
        CURRENT_DRIVER="$(basename "$(readlink "${SYSFS_DEVICE}/driver")")"
    fi
fi
if [ -n "${CURRENT_DRIVER}" ] && [ "${CURRENT_DRIVER}" != "spidev" ]; then
    echo "${SPI_DEVICE} is already bound to ${CURRENT_DRIVER}; refusing to replace it" >&2
    exit 1
fi

bind_with_legacy_match_patch() {
    # This vendor 4.19 kernel has neither driver_override nor SPI new_id.
    # Temporarily replace the unused final spidev OF match entry, bind, then
    # restore the original kernel bytes immediately.
    SPIDEV_VADDR="$(awk '$3 == "spidev_dt_ids" { print $1; exit }' /proc/kallsyms)"
    case "${SPIDEV_VADDR}" in
        90000000*) SPIDEV_PHYS_HEX="${SPIDEV_VADDR#90000000}" ;;
        *) echo "Unexpected spidev_dt_ids address: ${SPIDEV_VADDR}" >&2; return 1 ;;
    esac

    # struct of_device_id is 200 bytes on this 64-bit 4.19 kernel. The fourth
    # entry starts at 3*200 and its compatible[128] member starts at +64.
    MATCH_PHYS=$((0x${SPIDEV_PHYS_HEX} + 3 * 200 + 64))
    BACKUP="/tmp/spidev_semtech_match.$$"
    EXPECTED="/tmp/spidev_semtech_expected.$$"
    COMPATIBLE="/tmp/spidev_st7735_compatible.$$"
    dd if=/dev/mem of="${BACKUP}" bs=1 skip="${MATCH_PHYS}" count=128 2>/dev/null
    printf '%s' 'semtech,sx1301' > "${EXPECTED}"
    dd if="${BACKUP}" of="${BACKUP}.prefix" bs=1 count=14 2>/dev/null
    if ! cmp -s "${EXPECTED}" "${BACKUP}.prefix"; then
        rm -f "${BACKUP}" "${BACKUP}.prefix" "${EXPECTED}"
        echo "Refusing kernel match patch: expected semtech,sx1301 entry not found" >&2
        return 1
    fi

    restore_match() {
        dd if="${BACKUP}" of=/dev/mem bs=1 seek="${MATCH_PHYS}" count=128 conv=notrunc 2>/dev/null || true
        rm -f "${BACKUP}" "${BACKUP}.prefix" "${EXPECTED}" "${COMPATIBLE}"
    }
    trap restore_match EXIT INT TERM
    dd if=/dev/zero of=/dev/mem bs=1 seek="${MATCH_PHYS}" count=128 conv=notrunc 2>/dev/null
    printf '%s' 'sitronix,st7735r' > "${COMPATIBLE}"
    dd if="${COMPATIBLE}" of=/dev/mem bs=1 seek="${MATCH_PHYS}" conv=notrunc 2>/dev/null
    echo "${SPI_DEVICE}" > /sys/bus/spi/drivers/spidev/bind
    restore_match
    trap - EXIT INT TERM
}

if [ "${CURRENT_DRIVER}" != "spidev" ]; then
    if [ -e "${SYSFS_DEVICE}/driver_override" ]; then
        echo spidev > "${SYSFS_DEVICE}/driver_override"
        echo "${SPI_DEVICE}" > /sys/bus/spi/drivers/spidev/bind
    else
        bind_with_legacy_match_patch
    fi
fi

ATTEMPTS=20
while [ ! -c "${SPI_NODE}" ] && [ "${ATTEMPTS}" -gt 0 ]; do
    sleep 0.1
    ATTEMPTS=$((ATTEMPTS - 1))
done
if [ ! -c "${SPI_NODE}" ]; then
    echo "spidev bound but device node was not created: ${SPI_NODE}" >&2
    exit 1
fi

# The application defaults to spidev1.0. Keep that path compatible when the
# board's TFT device is physically registered at chip-select 2.
if [ "${SPI_NODE}" != "/dev/spidev1.0" ]; then
    ln -sfn "${SPI_NODE}" /dev/spidev1.0
fi

echo "ST7735S SPI ready: ${SPI_DEVICE} -> ${SPI_NODE}"
