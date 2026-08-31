#!/usr/bin/env bash
set -euo pipefail

# Build motor-safe CPU0 dual-core timing ELF + CPU1 RPGD worker ELF.
# Does not program the board or enable the motor.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
APP="${ROOT}/Firmware/VitisProjects/CartPoleFirmware"
DEBUG="${APP}/Debug"
OUT="${ROOT}/Firmware/AmpWorkspace/RPGD_AMP"
FIRMWARE_SRC="${ROOT}/Firmware/Src"
CPU0_LINKER="${ROOT}/Firmware/Src/Zynq/lscript_cpu0_amp.ld"
CPU1_LINKER="${ROOT}/Firmware/Src/RPGDWorker/lscript_cpu1_amp.ld"
XILINX_SPEC="${RPGD_XILINX_SPEC:-${DEBUG}/Xilinx.spec}"
CC="${RPGD_ARM_CC:-arm-none-eabi-gcc}"
SIZE="${RPGD_ARM_SIZE:-arm-none-eabi-size}"
OBJCOPY="${RPGD_ARM_OBJCOPY:-arm-none-eabi-objcopy}"

DEFAULT_CPU0_BSP="${ROOT}/Firmware/AmpWorkspace/cartpole_rpgd_amp/export/cartpole_rpgd_amp/sw/cartpole_rpgd_amp/standalone_domain"
DEFAULT_CPU1_BSP="${ROOT}/Firmware/AmpWorkspace/cartpole_rpgd_amp/export/cartpole_rpgd_amp/sw/cartpole_rpgd_amp/standalone_domain_cpu1"
FALLBACK_CPU0_BSP="${ROOT}/Firmware/VitisProjects/cartpole_zybo_secloc2026/export/cartpole_zybo_secloc2026/sw/cartpole_zybo_secloc2026/standalone_domain"

CPU0_BSP="${RPGD_CPU0_BSP_ROOT:-}"
CPU1_BSP="${RPGD_CPU1_BSP_ROOT:-}"
bsp_ok() { [ -f "${1}/bspinclude/include/xsysmon.h" ] && [ -d "${1}/bsplib/lib" ]; }
if [ -z "${CPU0_BSP}" ]; then
  if bsp_ok "${DEFAULT_CPU0_BSP}"; then
    CPU0_BSP="${DEFAULT_CPU0_BSP}"
  else
    CPU0_BSP="${FALLBACK_CPU0_BSP}"
  fi
fi
if [ -z "${CPU1_BSP}" ]; then
  if bsp_ok "${DEFAULT_CPU1_BSP}"; then
    CPU1_BSP="${DEFAULT_CPU1_BSP}"
  else
    CPU1_BSP="${CPU0_BSP}"
  fi
fi

if ! command -v "${CC}" >/dev/null 2>&1; then
  echo "ERROR: ${CC} not in PATH" >&2
  exit 1
fi
for required in "${CPU0_BSP}/bspinclude/include" "${CPU0_BSP}/bsplib/lib" "${XILINX_SPEC}" "${CPU0_LINKER}" "${CPU1_LINKER}"; do
  if [ ! -e "${required}" ]; then
    echo "ERROR: required input missing: ${required}" >&2
    echo "Run Firmware/Scripts/setup_rpgd_amp_platform.tcl if the AMP BSP is absent." >&2
    exit 1
  fi
done
if ! command -v rg >/dev/null 2>&1; then
  echo "ERROR: rg (ripgrep) is required" >&2
  exit 1
fi

mkdir -p "${OUT}/obj_cpu0" "${OUT}/obj_cpu1"

COMMON_CPU0=(
  -std=gnu11 -Wall -Wextra -O3
  -ffunction-sections -fdata-sections
  -fmessage-length=0
  -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard
  -DRPGD_BAREMETAL -DRPGD_DUAL_CORE -DRPGD_ON_TARGET_TEST -DUSE_AMP=1 -DNDEBUG
  "-I${FIRMWARE_SRC}"
  "-I${FIRMWARE_SRC}/CartPoleFirmware"
  "-I${FIRMWARE_SRC}/General"
  "-I${FIRMWARE_SRC}/Zynq"
  "-I${CPU0_BSP}/bspinclude/include"
)

COMMON_CPU1=(
  -std=gnu11 -Wall -Wextra -O3
  -ffunction-sections -fdata-sections
  -fmessage-length=0
  -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard
  -DRPGD_BAREMETAL -DRPGD_DUAL_CORE -DRPGD_WORKER_ONLY -DUSE_AMP=1 -DNDEBUG
  "-I${FIRMWARE_SRC}"
  "-I${FIRMWARE_SRC}/General"
  "-I${FIRMWARE_SRC}/Zynq"
  "-I${FIRMWARE_SRC}/RPGDWorker"
  "-I${CPU1_BSP}/bspinclude/include"
)

mapfile -t CPU0_SRCS < <(
  rg --files -L \
    "${FIRMWARE_SRC}/CartPoleFirmware" \
    "${FIRMWARE_SRC}/General" \
    "${FIRMWARE_SRC}/Zynq" \
    -g '*.c' | sort
)

CPU1_SRCS=(
  "${FIRMWARE_SRC}/RPGDWorker/main.c"
  "${FIRMWARE_SRC}/Zynq/amp_ipc.c"
  "${FIRMWARE_SRC}/General/rpgd_c/rpgd_cartpole.c"
  "${FIRMWARE_SRC}/General/rpgd_c/cartpole_model.c"
  "${FIRMWARE_SRC}/General/rpgd_c/cartpole_cost.c"
)

CPU0_OBJS=()
for src in "${CPU0_SRCS[@]}"; do
  rel="${src#"${FIRMWARE_SRC}/"}"
  obj="${OUT}/obj_cpu0/${rel%.c}.o"
  mkdir -p "$(dirname "${obj}")"
  echo "Compiling cpu0 ${rel}"
  "${CC}" "${COMMON_CPU0[@]}" -c -MT"${obj}" -MMD -MP -MF"${obj%.o}.d" -o "${obj}" "${src}"
  CPU0_OBJS+=("${obj}")
done

CPU1_OBJS=()
for src in "${CPU1_SRCS[@]}"; do
  rel="${src#"${FIRMWARE_SRC}/"}"
  obj="${OUT}/obj_cpu1/${rel%.c}.o"
  mkdir -p "$(dirname "${obj}")"
  echo "Compiling cpu1 ${rel}"
  "${CC}" "${COMMON_CPU1[@]}" -c -MT"${obj}" -MMD -MP -MF"${obj%.o}.d" -o "${obj}" "${src}"
  CPU1_OBJS+=("${obj}")
done

CPU0_ELF="${OUT}/CartPoleFirmware_rpgd_amp_cpu0.elf"
CPU1_ELF="${OUT}/RPGDWorker_cpu1.elf"

echo "Linking ${CPU0_ELF}"
"${CC}" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -Wl,-build-id=none "-specs=${XILINX_SPEC}" \
  -Wl,-T -Wl,"${CPU0_LINKER}" \
  -L"${CPU0_BSP}/bsplib/lib" \
  -o "${CPU0_ELF}" \
  "${CPU0_OBJS[@]}" \
  -lm -Wl,--start-group,-lxil,-lgcc,-lc,--end-group -Wl,--gc-sections

echo "Linking ${CPU1_ELF}"
"${CC}" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -Wl,-build-id=none "-specs=${XILINX_SPEC}" \
  -Wl,-T -Wl,"${CPU1_LINKER}" \
  -L"${CPU1_BSP}/bsplib/lib" \
  -o "${CPU1_ELF}" \
  "${CPU1_OBJS[@]}" \
  -lm -Wl,--start-group,-lxil,-lgcc,-lc,--end-group -Wl,--gc-sections

{
  echo "=== cpu0 ==="
  "${SIZE}" "${CPU0_ELF}"
  echo "=== cpu1 ==="
  "${SIZE}" "${CPU1_ELF}"
  "${CC}" --version | head -1
  printf 'CPU0_BSP=%s\nCPU1_BSP=%s\n' "${CPU0_BSP}" "${CPU1_BSP}"
  printf 'CPU0_LINKER=%s\nCPU1_LINKER=%s\n' "${CPU0_LINKER}" "${CPU1_LINKER}"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${CPU0_ELF}" "${CPU1_ELF}" "${CPU0_LINKER}" "${CPU1_LINKER}"
    find "${FIRMWARE_SRC}/General/rpgd_c" "${FIRMWARE_SRC}/RPGDWorker" "${FIRMWARE_SRC}/Zynq/amp_ipc.c" \
      "${FIRMWARE_SRC}/Zynq/rpgd_amp_dispatch.c" -type f | sort | xargs sha256sum
  fi
} | tee "${OUT}/rpgd_amp.build-info"

echo "Built:"
echo "  ${CPU0_ELF}"
echo "  ${CPU1_ELF}"
echo "Motor remains disabled. Do not program QSPI from this script."
