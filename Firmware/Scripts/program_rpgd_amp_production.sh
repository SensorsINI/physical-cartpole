#!/bin/bash
set -e
# JTAG-load dual-core RPGD AMP (CPU1 worker, then CPU0 cartpole).
# Same bitstream/PS7 as the slider programmer. Does not write QSPI.
# Close Vitis first (it steals JTAG). After this, hang + BTN0.

source /tools/Xilinx/Vivado/2020.1/settings64.sh
export PATH="/tools/Xilinx/Vitis/2020.1/bin:${PATH}"
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

echo "Keep Vitis closed (it steals JTAG)."
xsct "$(dirname "$(readlink -f "$0")")/program_rpgd_amp_production.tcl"
