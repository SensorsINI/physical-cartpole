#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${HERE}"

XILINX_HLS_INCLUDE="${XILINX_HLS_INCLUDE:-/media/santiago/Data/tools/Xilinx/Vitis/2020.1/include}"
COMMON_FLAGS=(
    -std=c++14 -O2 -Wall -Wno-unused-label
    -I"${XILINX_HLS_INCLUDE}"
    -I..
    -I../../../Firmware/Src/General
    -I../../NeuralNetworks/hls4ml_dense_1out_8_07_07_2026
)

g++ "${COMMON_FLAGS[@]}" -DSECLOC_INTEGRATION_TB \
    -o /tmp/secloc_three_block_tb \
    secloc_three_block_tb.cpp secloc_shell.cpp \
    ../secloc_gate_hls/secloc_gate.cpp \
    ../nn_marshal_hls/nn_marshal.cpp
/tmp/secloc_three_block_tb

g++ "${COMMON_FLAGS[@]}" \
    -o /tmp/secloc_shell_tb \
    secloc_shell_tb.cpp secloc_shell.cpp
/tmp/secloc_shell_tb

g++ "${COMMON_FLAGS[@]}" \
    -o /tmp/nn_marshal_tb \
    ../nn_marshal_hls/nn_marshal_tb.cpp ../nn_marshal_hls/nn_marshal.cpp
/tmp/nn_marshal_tb
