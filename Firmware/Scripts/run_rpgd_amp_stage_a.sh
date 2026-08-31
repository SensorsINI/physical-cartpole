#!/usr/bin/env bash
set -euo pipefail

# Stage A hardware validation: motor-safe dual-core JTAG run.
# Stages C-E (constrained motor, balancing, QSPI) are not executed here.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${ROOT}/Firmware/AmpWorkspace/RPGD_AMP"
UART="${RPGD_UART:-/dev/ttyUSB1}"
XSCT="${RPGD_XSCT:-/tools/Xilinx/Vitis/2020.1/bin/xsct}"
BAUD="${RPGD_UART_BAUD:-230400}"

mkdir -p "${OUT}"
"${ROOT}/Firmware/Scripts/build_rpgd_amp_elfs.sh"

if [ ! -e "${UART}" ]; then
  echo "UART ${UART} not present; ELFs built but Stage A capture skipped."
  exit 0
fi
if [ ! -x "${XSCT}" ]; then
  echo "xsct not found at ${XSCT}; ELFs built but Stage A capture skipped."
  exit 0
fi

LOG="${OUT}/stage_a_uart.log"
python3 - <<PY &
import serial, sys, time
port = "${UART}"
log_path = "${LOG}"
ser = serial.Serial(port, ${BAUD}, timeout=0.2)
end = time.time() + 180
tail = ""
with open(log_path, "w") as log:
    log.write("RPGD AMP Stage A UART capture\n")
    while time.time() < end:
        data = ser.read(4096)
        if data:
            text = data.decode("utf-8", errors="replace")
            sys.stdout.write(text)
            sys.stdout.flush()
            log.write(text)
            tail = (tail + text)[-4096:]
            if "RPGD_ON_TARGET idle" in tail:
                break
ser.close()
PY
UART_PID=$!
sleep 1
"${XSCT}" "${ROOT}/Firmware/Scripts/launch_rpgd_amp_jtag.tcl" || true
wait "${UART_PID}" || true
echo "Stage A UART log: ${LOG}"
if grep -q "RPGD_DUAL_PARITY PASS" "${LOG}" && grep -q "dual_gate=1" "${LOG}"; then
  echo "Stage A dual-core gate: PASS"
else
  echo "Stage A dual-core gate: incomplete or FAIL (see log). Motor stays disabled."
fi
