"""One-shot dead-zone firmware capture (no set_angle_filter; boot default filter).

Usage:
  python tests/run_firmware_deadzone_capture.py [--duration 16] [--rep 1]

Prints a GO countdown, records streamed state, saves .npz, runs analyzer.
"""

import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DRIVER = os.path.join(REPO, "Driver")
SIM = os.path.join(DRIVER, "CartPoleSimulation")
sys.path.insert(0, DRIVER)
sys.path.insert(0, SIM)

from DriverFunctions.interface import Interface  # noqa: E402
from CartPoleSimulation.Control_Toolkit.serial_interface_helper import get_serial_port  # noqa: E402
from globals import ANGLE_360_DEG_IN_ADC_UNITS, CHIP, SERIAL_BAUD, SERIAL_PORT_NUMBER  # noqa: E402

OUTPUT_DIR = os.path.join(REPO, "Driver", "DataAnalysis", "HardwareFilterTest", "output")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=16.0)
    p.add_argument("--rep", type=int, default=1)
    p.add_argument("--skip-countdown", action="store_true", help="Start recording immediately")
    args = p.parse_args()

    port = get_serial_port(chip_type=CHIP, serial_port_number=SERIAL_PORT_NUMBER)
    if port is None:
        raise RuntimeError("No serial port found")
    print(f"Opening {port} at {SERIAL_BAUD} baud (boot-default trimmed-mean 63/7 filter)")
    iface = Interface()
    iface.open(port, SERIAL_BAUD)
    iface.pc_control_mode(False)
    iface.control_mode(False)
    iface.set_motor(0)
    iface.stream_output(False)
    time.sleep(0.2)

    print("\n=== FIRMWARE dead-zone capture ===")
    print("Lift the pole somewhat ABOVE the dead-zone side (gap near horizontal, +1.3 rad).")
    if not args.skip_countdown:
        print("Recording starts after countdown — release on GO.\n")
        for count in (5, 4, 3, 2, 1):
            print(f"  {count}...", flush=True)
            time.sleep(1.0)
        print("  GO — release now!", flush=True)
    else:
        print("Recording NOW — swing the pole through the gap if not already moving.\n")

    angles, angleDs, invalids, chip_times = [], [], [], []
    iface.stream_output(True)
    t_start = time.time()
    try:
        while time.time() - t_start < args.duration:
            (angle, angleD, _pos, _tgt, _cmd, invalid_steps,
             _dt, chip_time, _lat, _latv) = iface.read_state()
            angles.append(angle)
            angleDs.append(angleD)
            invalids.append(invalid_steps)
            chip_times.append(chip_time)
    finally:
        iface.stream_output(False)
        iface.close()

    angles = np.asarray(angles, dtype=np.float64)
    angleDs = np.asarray(angleDs, dtype=np.float64)
    invalids = np.asarray(invalids, dtype=np.int64)
    chip_times = np.asarray(chip_times, dtype=np.float64)
    flagged = int(np.count_nonzero(invalids))
    print(f"\nRecorded {len(angles)} messages over {chip_times[-1] - chip_times[0]:.1f} s; "
          f"flagged polls: {flagged}")
    if flagged == 0:
        print("WARNING: invalid_steps never pulsed — reflash dead-zone firmware or release higher.")

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    path = os.path.join(OUTPUT_DIR, f"firmware_swing_{stamp}_rep{args.rep}.npz")
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    np.savez_compressed(
        path,
        angle=angles,
        angleD=angleDs,
        invalid_steps=invalids,
        chip_time=chip_times,
        config=np.array([63, 7, 2], dtype=np.int32),
        meta_phase="firmware",
        meta_duration_s=args.duration,
        meta_timestamp=stamp,
        meta_repetition=args.rep,
        meta_angle_360_adc=float(ANGLE_360_DEG_IN_ADC_UNITS),
    )
    print(f"Saved {path}")

    analyzer = os.path.join(REPO, "Driver", "DataAnalysis", "HardwareFilterTest", "analyze_filter_experiment.py")
    print("\n--- Analyzer ---")
    os.system(f"{sys.executable} {analyzer} {path}")
    return path


if __name__ == "__main__":
    main()
