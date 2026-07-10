#!/usr/bin/env bash
# Synthesize and export all three SecLoc chain HLS IPs.
#
# Usage (from anywhere):
#   FPGA/CustomIPs/build_secloc_chain.sh
#
# Only nn_marshal must be re-run after a network swap (nn_marshal_config.h
# change). shell and gate are network-agnostic.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VITIS_HLS="${VITIS_HLS:-/media/santiago/Data/tools/Xilinx/Vitis/2020.1/bin/vitis_hls}"

build_ip() {
    local dir="$1"
    echo "=== HLS csynth: $(basename "$dir") ==="
    (cd "$dir" && "$VITIS_HLS" -f run_hls.tcl)
}

build_ip "${HERE}/secloc_shell_hls"
build_ip "${HERE}/secloc_gate_hls"
build_ip "${HERE}/nn_marshal_hls"

echo "SecLoc chain HLS IPs exported to:"
echo "  secloc_shell_hls/secloc_shell_hls/solution1/impl/ip"
echo "  secloc_gate_hls/secloc_gate_hls/solution1/impl/ip"
echo "  nn_marshal_hls/nn_marshal_hls/solution1/impl/ip"
