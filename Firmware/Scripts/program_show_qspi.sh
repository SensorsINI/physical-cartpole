#!/usr/bin/env bash
set -euo pipefail

# Pack and flash the show AMP image to Zybo QSPI.
# BOOT.BIN = secloc2026 FSBL + short-pole BIT + AMP CPU0 (CPU1 blob inside).
# CPU0 starts CPU1 after FSBL handoff. Never uses destination_cpu / --accept-dual-qspi.
# JP5 must be JTAG while flashing. Then power off, JP5 = QSPI (two center pins), power on.
# Image-range erase only. Hanging at 0xFD0000 / 0xFFF000 is preserved.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${ROOT}/Firmware/AmpWorkspace/RPGD_AMP"
CPU0="${OUT}/CartPoleFirmware_rpgd_amp_cpu0.elf"
BIT="${ROOT}/FPGA/bitstreams/cartpole_short_pole_secloc.bit"
BOOT="${OUT}/BOOT.BIN"
BIF="${OUT}/show_qspi.generated.bif"
HANGING_OFF=$((0xFD0000))

FSBL_EXPORT="${ROOT}/Firmware/VitisProjects/cartpole_zybo_secloc2026/export/cartpole_zybo_secloc2026/sw/cartpole_zybo_secloc2026/boot/fsbl.elf"
FSBL_ZYNQ="${ROOT}/Firmware/VitisProjects/cartpole_zybo_secloc2026/zynq_fsbl/executable.elf"

if [ -f /tools/Xilinx/Vivado/2020.1/settings64.sh ]; then
  # shellcheck source=/dev/null
  source /tools/Xilinx/Vivado/2020.1/settings64.sh
fi
export PATH="/tools/Xilinx/Vitis/2020.1/bin:${PATH}"

if [ "${SHOW_QSPI_SKIP_BUILD:-0}" != "1" ]; then
  echo "Building production AMP ELFs..."
  RPGD_AMP_PRODUCTION=1 "${ROOT}/Firmware/Scripts/build_rpgd_amp_elfs.sh"
fi
if [ ! -f "${CPU0}" ]; then
  echo "ERROR: missing ${CPU0}" >&2
  exit 1
fi
if [ ! -f "${BIT}" ]; then
  echo "ERROR: missing ${BIT}" >&2
  exit 1
fi

FSBL=""
if [ -f "${FSBL_EXPORT}" ]; then
  FSBL="${FSBL_EXPORT}"
elif [ -f "${FSBL_ZYNQ}" ]; then
  FSBL="${FSBL_ZYNQ}"
else
  echo "ERROR: secloc2026 FSBL not found. Generate the cartpole_zybo_secloc2026 platform." >&2
  echo "  looked for ${FSBL_EXPORT}" >&2
  echo "  looked for ${FSBL_ZYNQ}" >&2
  exit 1
fi

mkdir -p "${OUT}"
cat > "${BIF}" <<EOF
the_ROM_image:
{
	[bootloader] ${FSBL}
	${BIT}
	${CPU0}
}
EOF

if ! command -v bootgen >/dev/null 2>&1; then
  echo "ERROR: bootgen not in PATH" >&2
  exit 1
fi

echo "bootgen -> ${BOOT}"
bootgen -arch zynq -image "${BIF}" -w on -o i "${BOOT}"

boot_size="$(stat -c%s "${BOOT}")"
if [ "${boot_size}" -ge "${HANGING_OFF}" ]; then
  echo "ERROR: ${BOOT} is ${boot_size} bytes; must stay below hanging slot 0xFD0000 (${HANGING_OFF})" >&2
  exit 1
fi
echo "BOOT.BIN ${boot_size} bytes (hanging reserved at 0xFD0000 / 0xFFF000)"

mkdir -p /tmp/xsct_home_pl_buttons
export _JAVA_OPTIONS="-Duser.home=/tmp/xsct_home_pl_buttons"
if [ ! -S /tmp/.X11-unix/X115 ]; then
  Xvfb :115 -screen 0 1024x768x24 >/tmp/xvfb_slider.log 2>&1 &
  sleep 0.4
fi
export DISPLAY=:115

if ! pgrep -x hw_server >/dev/null; then
  echo "Starting hw_server..."
  hw_server >/tmp/hw_server_slider.log 2>&1 &
  sleep 5
else
  echo "hw_server already running"
fi

echo "Keep Vitis closed (it steals JTAG). JP5 must be JTAG to program flash."
xsct "${ROOT}/Firmware/Scripts/program_qspi_boot.tcl" -boot "${BOOT}" -fsbl "${FSBL}"
