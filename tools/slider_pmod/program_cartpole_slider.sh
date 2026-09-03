#!/bin/bash
set -e
# JTAG: short-pole SecLoc bitstream + AMP CPU0 (show mux).
# CPU0 starts the CPU1 RPGD worker (same path as QSPI). CPU1 WFE-sleeps until SW0.
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
exec "${ROOT}/Firmware/Scripts/program_rpgd_amp_production.sh"
