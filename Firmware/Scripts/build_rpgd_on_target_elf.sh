#!/usr/bin/env bash
set -euo pipefail

# Build a motor-safe RPGD on-target ELF. Does not program or start the board.
# Usage: Firmware/Scripts/build_rpgd_on_target_elf.sh

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
APP="${ROOT}/Firmware/VitisProjects/CartPoleFirmware"
DEBUG="${APP}/Debug"
OUT="${APP}/RPGD_OnTarget"
FIRMWARE_SRC="${ROOT}/Firmware/Src"
DEFAULT_BSP_ROOT="${ROOT}/Firmware/VitisProjects/cartpole_zybo_secloc2026/export/cartpole_zybo_secloc2026/sw/cartpole_zybo_secloc2026/standalone_domain"
BSP_ROOT="${RPGD_BSP_ROOT:-${DEFAULT_BSP_ROOT}}"
BSP_INC="${BSP_ROOT}/bspinclude/include"
BSP_LIB="${BSP_ROOT}/bsplib/lib"
XILINX_SPEC="${RPGD_XILINX_SPEC:-${DEBUG}/Xilinx.spec}"
LINKER_SCRIPT="${RPGD_LINKER_SCRIPT:-${APP}/src/lscript.ld}"
CC="${RPGD_ARM_CC:-arm-none-eabi-gcc}"
SIZE="${RPGD_ARM_SIZE:-arm-none-eabi-size}"

if ! command -v "${CC}" >/dev/null 2>&1; then
  echo "ERROR: ${CC} not in PATH" >&2
  exit 1
fi
for required in "${BSP_INC}" "${BSP_LIB}" "${XILINX_SPEC}" "${LINKER_SCRIPT}"; do
  if [ ! -e "${required}" ]; then
    echo "ERROR: required Vitis/BSP input missing: ${required}" >&2
    echo "Set RPGD_BSP_ROOT, RPGD_XILINX_SPEC, or RPGD_LINKER_SCRIPT if your platform uses another path." >&2
    exit 1
  fi
done
if ! command -v rg >/dev/null 2>&1; then
  echo "ERROR: rg (ripgrep) is required to enumerate versioned firmware sources" >&2
  exit 1
fi

mkdir -p "${OUT}/obj"

COMMON=(
  -std=gnu11 -Wall -Wextra -O3
  -ffunction-sections -fdata-sections
  -fmessage-length=0
  -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard
  -DRPGD_BAREMETAL -DRPGD_ON_TARGET_TEST -DNDEBUG
  "-I${FIRMWARE_SRC}"
  "-I${FIRMWARE_SRC}/CartPoleFirmware"
  "-I${FIRMWARE_SRC}/General"
  "-I${FIRMWARE_SRC}/Zynq"
  "-I${BSP_INC}"
)

mapfile -t SRCS < <(
  rg --files -L \
    "${FIRMWARE_SRC}/CartPoleFirmware" \
    "${FIRMWARE_SRC}/General" \
    "${FIRMWARE_SRC}/Zynq" \
    -g '*.c' | sort
)
if [ "${#SRCS[@]}" -eq 0 ]; then
  echo "ERROR: no firmware C sources found" >&2
  exit 1
fi

OBJS=()
for src in "${SRCS[@]}"; do
  rel="${src#"${FIRMWARE_SRC}/"}"
  obj="${OUT}/obj/${rel%.c}.o"
  mkdir -p "$(dirname "${obj}")"
  echo "Compiling ${rel}"
  "${CC}" "${COMMON[@]}" -c -MT"${obj}" -MMD -MP -MF"${obj%.o}.d" -o "${obj}" "${src}"
  OBJS+=("${obj}")
done

echo "Linking CartPoleFirmware_rpgd_on_target.elf"
"${CC}" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -Wl,-build-id=none "-specs=${XILINX_SPEC}" \
  -Wl,-T -Wl,"${LINKER_SCRIPT}" \
  -L"${BSP_LIB}" \
  -o "${OUT}/CartPoleFirmware_rpgd_on_target.elf" \
  "${OBJS[@]}" \
  -lm -Wl,--start-group,-lxil,-lgcc,-lc,--end-group -Wl,--gc-sections

"${SIZE}" "${OUT}/CartPoleFirmware_rpgd_on_target.elf" | tee "${OUT}/CartPoleFirmware_rpgd_on_target.elf.size"
{
  "${CC}" --version
  printf 'BSP_ROOT=%s\nLINKER_SCRIPT=%s\n' "${BSP_ROOT}" "${LINKER_SCRIPT}"
} > "${OUT}/CartPoleFirmware_rpgd_on_target.build-info"
echo "Built ${OUT}/CartPoleFirmware_rpgd_on_target.elf"
echo "Do not program the board from this script."
