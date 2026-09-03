#!/usr/bin/env bash
set -euo pipefail

# Build the standalone show image and install it as BOOT.BIN on a mounted FAT
# microSD card. This never partitions or formats the card.
#
# Usage:
#   Firmware/Scripts/install_show_sd.sh
#   Firmware/Scripts/install_show_sd.sh /path/to/mounted/card

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BOOT="${ROOT}/Firmware/AmpWorkspace/RPGD_AMP/BOOT.BIN"
LABEL="${SHOW_SD_LABEL:-SD_Zybo}"

if [ "$#" -gt 1 ]; then
  echo "Usage: $0 [mounted-card-directory]" >&2
  exit 2
fi

if [ "$#" -eq 1 ]; then
  MOUNT="$1"
else
  MOUNT="$(findmnt -rn -S "LABEL=${LABEL}" -o TARGET | head -n 1)"
fi

if [ -z "${MOUNT}" ] || [ ! -d "${MOUNT}" ]; then
  echo "ERROR: mounted FAT card '${LABEL}' not found." >&2
  echo "Pass its mount directory explicitly: $0 /media/user/${LABEL}" >&2
  exit 1
fi

SOURCE="$(findmnt -rn -T "${MOUNT}" -o SOURCE)"
FSTYPE="$(findmnt -rn -T "${MOUNT}" -o FSTYPE)"
OPTIONS="$(findmnt -rn -T "${MOUNT}" -o OPTIONS)"
case "${FSTYPE}" in
  vfat|fat|msdos) ;;
  *)
    echo "ERROR: ${MOUNT} (${SOURCE}) is ${FSTYPE}, not a FAT filesystem." >&2
    exit 1
    ;;
esac
case ",${OPTIONS}," in
  *,rw,*) ;;
  *)
    echo "ERROR: ${MOUNT} (${SOURCE}) is not mounted read/write." >&2
    exit 1
    ;;
esac

echo "Building standalone show BOOT.BIN..."
SHOW_QSPI_BUILD_ONLY=1 "${ROOT}/Firmware/Scripts/program_show_qspi.sh"

boot_size="$(stat -c%s "${BOOT}")"
available="$(df -B1 --output=avail "${MOUNT}" | tail -n 1 | tr -d ' ')"
if [ "${available}" -lt "${boot_size}" ]; then
  echo "ERROR: ${MOUNT} has ${available} bytes free; need ${boot_size}." >&2
  exit 1
fi

TMP="${MOUNT}/BOOT.BIN.tmp"
DEST="${MOUNT}/BOOT.BIN"
trap 'rm -f "${TMP}"' EXIT

echo "Installing ${BOOT} -> ${DEST} (${boot_size} bytes)"
cp "${BOOT}" "${TMP}"
sync "${TMP}"
mv -f "${TMP}" "${DEST}"
sync "${DEST}"

if ! cmp -s "${BOOT}" "${DEST}"; then
  echo "ERROR: SD copy verification failed: ${DEST}" >&2
  exit 1
fi

echo "SD_INSTALL_OK ${DEST}"
echo "Safely eject the card, insert it in the Zybo, set JP5 to SD, and power-cycle."
