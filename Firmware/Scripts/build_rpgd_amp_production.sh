#!/usr/bin/env bash
set -euo pipefail

# Build the live dual-core RPGD cartpole ELFs (no on-target harness).
# Does not program the board.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [ -f /tools/Xilinx/Vitis/2020.1/settings64.sh ]; then
  # shellcheck source=/dev/null
  source /tools/Xilinx/Vitis/2020.1/settings64.sh
fi
export RPGD_AMP_PRODUCTION=1
exec "${ROOT}/Firmware/Scripts/build_rpgd_amp_elfs.sh"
