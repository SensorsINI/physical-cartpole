#!/usr/bin/env bash
# JTAG-program the short-pole SecLoc bitstream + Debug ELF.
# Do NOT use program_cartpole_slider.sh (that loads AMP RPGD).
set -euo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
source /tools/Xilinx/Vivado/2020.1/settings64.sh
export PATH="/tools/Xilinx/Vitis/2020.1/bin:${PATH}"
mkdir -p /tmp/xsct_home_pl_buttons
export _JAVA_OPTIONS="-Duser.home=/tmp/xsct_home_pl_buttons"
if [ ! -S /tmp/.X11-unix/X115 ]; then
  Xvfb :115 -screen 0 1024x768x24 >/tmp/xvfb_slider.log 2>&1 &
  sleep 0.4
fi
export DISPLAY=:115
exec xsct "${ROOT}/tools/slider_pmod/program_cartpole_short_pole.tcl"
