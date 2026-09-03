#!/usr/bin/env bash
set -euo pipefail

# Build CPU0 dual-core ELF + CPU1 RPGD worker ELF.
# Default: motor-safe on-target timing harness (-DRPGD_ON_TARGET_TEST).
# Production cartpole (CPU0 show mux, CPU1 waits until SW0 / RPGD):
#   RPGD_AMP_PRODUCTION=1 Firmware/Scripts/build_rpgd_amp_elfs.sh
# Does not program the board.

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
READELF="${RPGD_ARM_READELF:-arm-none-eabi-readelf}"

# Same hardware platform as Vitis Debug LSTM (PL buttons / slider).
DEFAULT_CPU0_BSP="${ROOT}/Firmware/VitisProjects/cartpole_zybo_secloc2026/export/cartpole_zybo_secloc2026/sw/cartpole_zybo_secloc2026/standalone_domain"
DEFAULT_CPU1_BSP="${ROOT}/Firmware/VitisProjects/cartpole_zybo_secloc2026/export/cartpole_zybo_secloc2026/sw/cartpole_zybo_secloc2026/standalone_domain_cpu1"

CPU0_BSP="${RPGD_CPU0_BSP_ROOT:-${DEFAULT_CPU0_BSP}}"
CPU1_BSP="${RPGD_CPU1_BSP_ROOT:-${DEFAULT_CPU1_BSP}}"
bsp_ok() { [ -f "${1}/bspinclude/include/xsysmon.h" ] && [ -d "${1}/bsplib/lib" ]; }
if ! bsp_ok "${CPU0_BSP}"; then
  echo "ERROR: CPU0 BSP missing or incomplete: ${CPU0_BSP}" >&2
  echo "Refresh Firmware/VitisProjects/cartpole_zybo_secloc2026 from the PL-buttons XSA." >&2
  exit 1
fi
if ! bsp_ok "${CPU1_BSP}"; then
  echo "ERROR: CPU1 AMP BSP missing: ${CPU1_BSP}" >&2
  echo "Run: xsct Firmware/Scripts/setup_rpgd_amp_platform.tcl" >&2
  exit 1
fi
if ! rg -q "XPAR_PL_BUTTONS_GPIO_DEVICE_ID" "${CPU0_BSP}/bspinclude/include/xparameters.h"; then
  echo "ERROR: CPU0 BSP has no PL_BUTTONS_GPIO. Use the cartpole_zybo_pl_buttons platform, not a stale XSA." >&2
  exit 1
fi

if ! command -v "${CC}" >/dev/null 2>&1; then
  echo "ERROR: ${CC} not in PATH" >&2
  exit 1
fi
for required in "${CPU0_BSP}/bspinclude/include" "${CPU0_BSP}/bsplib/lib" "${XILINX_SPEC}" "${CPU0_LINKER}" "${CPU1_LINKER}"; do
  if [ ! -e "${required}" ]; then
    echo "ERROR: required input missing: ${required}" >&2
    echo "Run: xsct Firmware/Scripts/setup_rpgd_amp_platform.tcl" >&2
    exit 1
  fi
done
if ! command -v rg >/dev/null 2>&1; then
  echo "ERROR: rg (ripgrep) is required" >&2
  exit 1
fi

mkdir -p "${OUT}/obj_cpu0" "${OUT}/obj_cpu1"

PRODUCTION="${RPGD_AMP_PRODUCTION:-0}"
CPU0_DEFS=( -DRPGD_BAREMETAL -DRPGD_DUAL_CORE -DUSE_AMP=1 -DNDEBUG )
if [ "${PRODUCTION}" != "1" ]; then
  CPU0_DEFS+=( -DRPGD_ON_TARGET_TEST )
fi
if [ -n "${RPGD_CPU0_EXTRA_CFLAGS:-}" ]; then
  # Word-split is intentional: callers pass extra -D/-f flags.
  read -r -a _cpu0_extra <<< "${RPGD_CPU0_EXTRA_CFLAGS}"
  CPU0_DEFS+=( "${_cpu0_extra[@]}" )
fi

COMMON_CPU0=(
  -std=gnu11 -Wall -Wextra -O3
  -ffunction-sections -fdata-sections
  -fmessage-length=0
  -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard
  "${CPU0_DEFS[@]}"
  "-I${OUT}"
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

CPU0_ELF="${OUT}/CartPoleFirmware_rpgd_amp_cpu0.elf"
CPU1_ELF="${OUT}/RPGDWorker_cpu1.elf"
CPU1_BIN="${OUT}/RPGDWorker_cpu1.bin"
CPU1_BLOB_S="${OUT}/cpu1_blob.S"
CPU1_BLOB_META="${OUT}/cpu1_blob_meta.h"
CPU1_LOAD_ADDR=0x10000000

CPU1_OBJS=()
for src in "${CPU1_SRCS[@]}"; do
  rel="${src#"${FIRMWARE_SRC}/"}"
  obj="${OUT}/obj_cpu1/${rel%.c}.o"
  mkdir -p "$(dirname "${obj}")"
  echo "Compiling cpu1 ${rel}"
  "${CC}" "${COMMON_CPU1[@]}" -c -MT"${obj}" -MMD -MP -MF"${obj%.o}.d" -o "${obj}" "${src}"
  CPU1_OBJS+=("${obj}")
done

echo "Linking ${CPU1_ELF}"
"${CC}" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -Wl,-build-id=none "-specs=${XILINX_SPEC}" \
  -Wl,-T -Wl,"${CPU1_LINKER}" \
  -L"${CPU1_BSP}/bsplib/lib" \
  -o "${CPU1_ELF}" \
  "${CPU1_OBJS[@]}" \
  -lm -Wl,--start-group,-lxil,-lgcc,-lc,--end-group -Wl,--gc-sections

if ! command -v "${READELF}" >/dev/null 2>&1; then
  echo "ERROR: ${READELF} not in PATH" >&2
  exit 1
fi

entry_raw="$("${READELF}" -h "${CPU1_ELF}" | awk '/Entry point address/ { print $NF; exit }')"
load_vma="$("${READELF}" -lW "${CPU1_ELF}" | awk '$1 == "LOAD" { print $3; exit }')"
load_lma="$("${READELF}" -lW "${CPU1_ELF}" | awk '$1 == "LOAD" { print $4; exit }')"
load_filesz="$("${READELF}" -lW "${CPU1_ELF}" | awk '$1 == "LOAD" { print $5; exit }')"
nload="$("${READELF}" -lW "${CPU1_ELF}" | awk '$1 == "LOAD" { n++ } END { print n+0 }')"
entry_norm="$(printf '0x%08x' "$((entry_raw))")"
vma_norm="$(printf '0x%08x' "$((load_vma))")"
lma_norm="$(printf '0x%08x' "$((load_lma))")"
if [ "${nload}" -ne 1 ]; then
  echo "ERROR: CPU1 ELF has ${nload} PT_LOAD segments; objcopy -O binary needs exactly one" >&2
  exit 1
fi
if [ "${entry_norm}" != "${CPU1_LOAD_ADDR}" ] || [ "${vma_norm}" != "${CPU1_LOAD_ADDR}" ] || [ "${lma_norm}" != "${CPU1_LOAD_ADDR}" ]; then
  echo "ERROR: CPU1 ELF must load and enter at ${CPU1_LOAD_ADDR} (entry=${entry_raw} vma=${load_vma} lma=${load_lma})" >&2
  exit 1
fi

echo "objcopy ${CPU1_BIN}"
"${OBJCOPY}" -O binary "${CPU1_ELF}" "${CPU1_BIN}"
blob_size="$(stat -c%s "${CPU1_BIN}")"
if [ "${blob_size}" -le 0 ] || [ "${blob_size}" -ge 65536 ]; then
  echo "ERROR: CPU1 blob size ${blob_size} must be in (0, 64 KiB)" >&2
  exit 1
fi
if [ "${blob_size}" -ne "$((load_filesz))" ]; then
  echo "ERROR: CPU1 blob size ${blob_size} != PT_LOAD FileSiz $((load_filesz))" >&2
  exit 1
fi

cat > "${CPU1_BLOB_META}" <<EOF
#ifndef CPU1_BLOB_META_H
#define CPU1_BLOB_META_H
#define CPU1_ENTRY_ADDR ${CPU1_LOAD_ADDR}u
#define CPU1_BLOB_SIZE ${blob_size}u
#endif
EOF

cat > "${CPU1_BLOB_S}" <<EOF
	.section .rodata.cpu1_blob, "a"
	.global cpu1_blob_start
	.global cpu1_blob_end
	.type cpu1_blob_start, %object
	.type cpu1_blob_end, %object
	.balign 8
cpu1_blob_start:
	.incbin "${CPU1_BIN}"
cpu1_blob_end:
EOF

CPU0_OBJS=()
for src in "${CPU0_SRCS[@]}"; do
  rel="${src#"${FIRMWARE_SRC}/"}"
  obj="${OUT}/obj_cpu0/${rel%.c}.o"
  mkdir -p "$(dirname "${obj}")"
  echo "Compiling cpu0 ${rel}"
  "${CC}" "${COMMON_CPU0[@]}" -c -MT"${obj}" -MMD -MP -MF"${obj%.o}.d" -o "${obj}" "${src}"
  CPU0_OBJS+=("${obj}")
done

blob_obj="${OUT}/obj_cpu0/cpu1_blob.o"
echo "Assembling cpu1 blob"
"${CC}" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -c -o "${blob_obj}" "${CPU1_BLOB_S}"
CPU0_OBJS+=("${blob_obj}")

echo "Linking ${CPU0_ELF}"
"${CC}" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -Wl,-build-id=none "-specs=${XILINX_SPEC}" \
  -Wl,-T -Wl,"${CPU0_LINKER}" \
  -L"${CPU0_BSP}/bsplib/lib" \
  -o "${CPU0_ELF}" \
  "${CPU0_OBJS[@]}" \
  -lm -Wl,--start-group,-lxil,-lgcc,-lc,--end-group -Wl,--gc-sections

{
  echo "=== cpu0 ==="
  "${SIZE}" "${CPU0_ELF}"
  echo "=== cpu1 ==="
  "${SIZE}" "${CPU1_ELF}"
  printf 'CPU1_BLOB_SIZE=%s\nCPU1_ENTRY_ADDR=%s\n' "${blob_size}" "${CPU1_LOAD_ADDR}"
  "${CC}" --version | head -1
  printf 'PRODUCTION=%s\nCPU0_BSP=%s\nCPU1_BSP=%s\n' "${PRODUCTION}" "${CPU0_BSP}" "${CPU1_BSP}"
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
echo "  ${CPU1_BIN} (${blob_size} bytes)"
if [ "${PRODUCTION}" = "1" ]; then
  echo "Production AMP: CPU0 copies CPU1 into DDR and starts it. JTAG: Firmware/Scripts/program_rpgd_amp_production.sh  QSPI: Firmware/Scripts/program_show_qspi.sh"
else
  echo "Motor-safe on-target harness. Do not program QSPI from this script."
fi
