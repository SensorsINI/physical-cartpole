"""Hardware test runner for the FPGA angle filter block (median/trimmed mean).

Collects paired (filtered16, raw16) streams from the running cartpole via the
serial link, sweeping filter configurations at runtime (no reflash needed).
Results are saved as .npz files for analyze_filter_experiment.py.

Phases:
  static   Pole at rest. Sweeps all filter configurations automatically.
           Zero user interaction once started.
  dynamic  Records long paired streams while the user excites the pole
           (one swing release per repetition). One interaction per repetition.
  deadzone Records streams incl. the hardware dead-zone tracking registers
           during a decaying free swing released above the dead-zone side:
           early swings cross the zone, later ones naturally turn around
           inside it. One release per repetition, no precision needed.
  firmware Records the STREAMED STATE (angle, angleD, invalid_steps) during a
           dead-zone swing to verify the firmware consumes the hardware flags:
           invalid_steps must pulse during zone episodes, angle must glide
           through (extrapolated), angleD must stay spike-free.
  check    30-second sanity check of the serial commands and filter block.
  set      Only reconfigure the filter and exit (persists until board reset).
           For A/B tests with the normal control software.

Usage (from repo root, cartpole powered and firmware running):
  python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py check
  python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py static
  python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py dynamic --repetitions 3
  python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py deadzone --repetitions 4
  python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py firmware --repetitions 2
  python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py set --window 63 --trim 0 --mode 1
"""

import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DRIVER_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
CARTPOLE_SIM_ROOT = os.path.join(DRIVER_ROOT, "CartPoleSimulation")

sys.path.insert(0, DRIVER_ROOT)
sys.path.insert(1, CARTPOLE_SIM_ROOT)

from DriverFunctions.interface import (  # noqa: E402
    Interface,
    ANGLE_FILTER_MODE_RAW,
    ANGLE_FILTER_MODE_MEDIAN,
    ANGLE_FILTER_MODE_TRIMMED_MEAN,
)
from CartPoleSimulation.Control_Toolkit.serial_interface_helper import get_serial_port  # noqa: E402
from globals import CHIP, SERIAL_BAUD, SERIAL_PORT_NUMBER  # noqa: E402

OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

# Values the firmware restores as boot defaults (goniometer_zynq.h).
DEFAULT_CONFIG = ("trimmed63_t7_default", 63, 7, ANGLE_FILTER_MODE_TRIMMED_MEAN)

# (label, window_size, trim_count, filter_mode)
# Every collection also records the raw stream, so the sweep focuses on the
# filtered output of each configuration.
STATIC_CONFIGS = [
    ("raw_passthrough",      1, 0, ANGLE_FILTER_MODE_RAW),
    ("median63_old_design", 63, 0, ANGLE_FILTER_MODE_MEDIAN),
    ("average63_trim0",     63, 0, ANGLE_FILTER_MODE_TRIMMED_MEAN),
    ("trimmed63_t3",        63, 3, ANGLE_FILTER_MODE_TRIMMED_MEAN),
    ("trimmed63_t7_default", 63, 7, ANGLE_FILTER_MODE_TRIMMED_MEAN),
    ("trimmed63_t15",       63, 15, ANGLE_FILTER_MODE_TRIMMED_MEAN),
    ("trimmed63_t31_max",   63, 31, ANGLE_FILTER_MODE_TRIMMED_MEAN),
    ("median31",            31, 0, ANGLE_FILTER_MODE_MEDIAN),
    ("trimmed31_t3",        31, 3, ANGLE_FILTER_MODE_TRIMMED_MEAN),
    ("trimmed15_t1",        15, 1, ANGLE_FILTER_MODE_TRIMMED_MEAN),
]

MODE_NAMES = {0: "raw", 1: "median", 2: "trimmed_mean"}


def open_interface(port=None, baud=None):
    port = port or get_serial_port(chip_type=CHIP, serial_port_number=SERIAL_PORT_NUMBER)
    if port is None:
        raise RuntimeError("No serial port found. Pass --port explicitly.")
    baud = baud or SERIAL_BAUD
    interface = Interface()
    print(f"Opening {port} at {baud} baud")
    interface.open(port, baud)
    interface.pc_control_mode(False)
    interface.control_mode(False)
    interface.set_motor(0)
    interface.stream_output(False)
    time.sleep(0.2)
    return interface


def restore_defaults(interface):
    _, window, trim, mode = DEFAULT_CONFIG
    interface.set_angle_filter(window, trim, mode)
    print(f"Filter restored to boot default: window={window}, trim={trim}, mode={MODE_NAMES[mode]}")


def collect_config(interface, label, window, trim, mode, samples, interval_us):
    interface.set_angle_filter(window, trim, mode)
    # Window refills in window*2.2us; generous settle also absorbs serial jitter.
    time.sleep(0.1)
    t0 = time.time()
    filtered, raw = interface.collect_angle_pairs(length=samples, interval_us=interval_us)
    dt = time.time() - t0
    filtered = np.asarray(filtered, dtype=np.uint16)
    raw = np.asarray(raw, dtype=np.uint16)
    print(f"  {label:24s} window={window:2d} trim={trim:2d} mode={MODE_NAMES[mode]:12s} "
          f"{samples} pairs in {dt:.1f}s | filtered std={filtered.std():7.2f} raw std={raw.std():7.2f} "
          f"(16-bit codes, /16 for 12-bit LSB)")
    return filtered, raw


def save_npz(path, arrays, meta):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez_compressed(path, **arrays, **{f"meta_{k}": v for k, v in meta.items()})
    print(f"Saved {path}")


def phase_check(interface, args):
    print("\n=== CHECK: serial commands and filter block sanity ===")
    ok = True

    # 1. Raw passthrough: filtered output must equal raw output exactly.
    filtered, raw = collect_config(interface, "raw_passthrough", 1, 0, ANGLE_FILTER_MODE_RAW, 2000, args.interval_us)
    mismatches = int(np.count_nonzero(filtered != raw))
    # The two register reads are a few hundred ns apart while the filter
    # updates every ~2.2 us, so a small fraction of pairs can straddle an
    # update and differ by one sample.
    frac = mismatches / len(raw)
    print(f"  raw mode: filtered==raw for {100 * (1 - frac):.2f}% of pairs (expect >80%)")
    ok &= frac < 0.2

    # 2. Informational: how many of the 4 LSBs of the raw XADC word toggle.
    lsb_nonzero = int(np.count_nonzero(raw & 0xF))
    print(f"  raw LSB nibble nonzero in {lsb_nonzero}/{len(raw)} samples "
          f"(0 if the XADC pads the 12-bit code with zeros)")

    # 3. Averaging must reduce noise; pure average of 63 should give a clearly
    #    smaller std than raw on a static signal.
    filtered, raw = collect_config(interface, "average63_trim0", 63, 0, ANGLE_FILTER_MODE_TRIMMED_MEAN, 2000, args.interval_us)
    if raw.std() > 0:
        gain = raw.std() / max(filtered.std(), 1e-9)
        print(f"  average63: noise std reduced {gain:.1f}x (expect roughly 4-8x for white noise)")
        ok &= gain > 1.5
    else:
        print("  raw std is zero (sensor unnaturally quiet); skipping gain check")

    # 4. Dead-zone registers: reply must parse; with the pole hanging on-track
    #    the rails are untouched between two snapshots, and dz_age advances.
    try:
        s1 = interface.get_dead_zone()
        time.sleep(0.2)
        s2 = interface.get_dead_zone()
        print(f"  dead zone: status={s2[0]}, window={s2[1]}, age={s2[2]}, "
              f"counters low={s2[3]} high={s2[4]}")
        counters_ok = (s2[3] >= s1[3]) and (s2[4] >= s1[4])
        # Pole hangs mid-track: latest sample must not be at a rail and the
        # window must be clean (unless it was left inside the dead zone).
        if s2[0] == 0 and s2[1] == 0:
            print("  dead zone: pole on-track, no rail contact (as expected at rest)")
        else:
            print("  dead zone: rail contact at rest — pole parked in the dead zone, "
                  "or thresholds too tight; not a failure by itself")
        ok &= counters_ok
    except Exception as exc:
        print(f"  dead zone readout FAILED: {exc} (old firmware/bitstream without 0xD3?)")
        ok = False

    # 5. Config echo path already validated by set_angle_filter itself.
    restore_defaults(interface)
    print("CHECK PASSED" if ok else "CHECK FAILED — inspect output above")
    return 0 if ok else 1


def phase_static(interface, args):
    print("\n=== STATIC: filter configuration sweep (pole must hang still) ===")
    print("Do not touch the pole or the cart during this phase (~1 min).\n")
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    arrays = {}
    labels = []
    for label, window, trim, mode in STATIC_CONFIGS:
        filtered, raw = collect_config(interface, label, window, trim, mode, args.samples, args.interval_us)
        arrays[f"{label}__filtered"] = filtered
        arrays[f"{label}__raw"] = raw
        arrays[f"{label}__config"] = np.array([window, trim, mode], dtype=np.int32)
        labels.append(label)

    restore_defaults(interface)
    path = os.path.join(OUTPUT_DIR, f"static_sweep_{stamp}.npz")
    save_npz(path, arrays, {
        "phase": "static",
        "labels": np.array(labels),
        "interval_us": args.interval_us,
        "samples": args.samples,
        "timestamp": stamp,
    })
    print("\nStatic phase done. Next: run 'dynamic', then analyze_filter_experiment.py.")
    return 0


def phase_dynamic(interface, args):
    print("\n=== DYNAMIC: pole swing recordings ===")
    print("For each repetition: lift the pole to roughly horizontal, hold it still,")
    print("and release it exactly when the countdown says GO. Then hands off until")
    print("the recording finishes (LED blinks fast while sampling).\n")
    # Hardware runs the configuration under test; the raw stream recorded
    # alongside allows offline replay of every other configuration.
    _, window, trim, mode = DEFAULT_CONFIG
    interface.set_angle_filter(window, trim, mode)

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    duration_s = args.samples * args.interval_us * 1e-6
    for rep in range(args.repetitions):
        print(f"\nRepetition {rep + 1}/{args.repetitions}: recording window is {duration_s:.1f} s.")
        input("  Lift the pole to horizontal, hold it, and press Enter (release on GO)...")
        for count in (3, 2, 1):
            print(f"  {count}...")
            time.sleep(1.0)
        print("  GO — release now!")
        filtered, raw = interface.collect_angle_pairs(length=args.samples, interval_us=args.interval_us)
        filtered = np.asarray(filtered, dtype=np.uint16)
        raw = np.asarray(raw, dtype=np.uint16)
        swing_range = (int(raw.min()), int(raw.max()))
        print(f"  Recorded {len(raw)} pairs; raw range {swing_range} "
              f"({(swing_range[1] - swing_range[0]) / 16:.0f} ADC codes span)")
        if swing_range[1] - swing_range[0] < 16 * 100:
            print("  WARNING: very small angle span; the release may have been missed.")

        path = os.path.join(OUTPUT_DIR, f"dynamic_swing_{stamp}_rep{rep + 1}.npz")
        save_npz(path, {
            "filtered": filtered,
            "raw": raw,
            "config": np.array([window, trim, mode], dtype=np.int32),
        }, {
            "phase": "dynamic",
            "interval_us": args.interval_us,
            "samples": args.samples,
            "timestamp": stamp,
            "repetition": rep + 1,
        })

    restore_defaults(interface)
    print("\nDynamic phase done. Run analyze_filter_experiment.py next.")
    return 0


def phase_deadzone(interface, args):
    print("\n=== DEADZONE: hardware rail-detection test ===")
    print("The potentiometer dead zone sits to the side, at the far end of the")
    print("swing range. No precise release is needed: lift the pole a MODERATE")
    print("amount above that side and let it swing freely. The first swings CROSS")
    print("the zone; as the swing decays, the pole naturally TURNS AROUND inside")
    print("the zone for a while, then stops reaching it — one release produces")
    print("crossings, turnarounds, and near-misses in a single recording.\n")
    _, window, trim, mode = DEFAULT_CONFIG
    interface.set_angle_filter(window, trim, mode)

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    duration_s = args.samples * args.interval_us * 1e-6
    for rep in range(args.repetitions):
        maneuver = "swing"
        print(f"\nRepetition {rep + 1}/{args.repetitions} "
              f"(recording window {duration_s:.1f} s).")
        print("  Lift the pole somewhat above the dead-zone side — high enough that")
        print("  the first swings pass through it, but not much higher, so the")
        print("  turnaround band is reached within the recording.")
        input("  Press Enter, then release on GO...")
        for count in (3, 2, 1):
            print(f"  {count}...")
            time.sleep(1.0)
        print("  GO — release now!")

        before = interface.get_dead_zone()
        filtered, raw, dz_window, dz_status, dz_age = interface.collect_angle_deadzone(
            length=args.samples, interval_us=args.interval_us)
        after = interface.get_dead_zone()

        filtered = np.asarray(filtered, dtype=np.uint16)
        raw = np.asarray(raw, dtype=np.uint16)
        dz_window = np.asarray(dz_window, dtype=np.uint8)
        dz_status = np.asarray(dz_status, dtype=np.uint8)
        dz_age = np.asarray(dz_age, dtype=np.uint16)

        low_hits = after[3] - before[3]
        high_hits = after[4] - before[4]
        touched = "both rails" if (low_hits and high_hits) else \
                  ("low rail only" if low_hits else ("high rail only" if high_hits else "NO RAIL"))
        print(f"  Recorded {len(raw)} samples; rail contact during recording: {touched} "
              f"(low {low_hits}, high {high_hits} XADC samples)")
        print(f"  Samples flagged at rail: {int(np.count_nonzero(dz_status))}, "
              f"max contaminated window: {int(dz_window.max())}/{window}")
        if not (low_hits or high_hits):
            print("  WARNING: the pole never reached the dead zone; repeat this repetition.")

        path = os.path.join(OUTPUT_DIR, f"deadzone_{maneuver}_{stamp}_rep{rep + 1}.npz")
        save_npz(path, {
            "filtered": filtered,
            "raw": raw,
            "dz_window": dz_window,
            "dz_status": dz_status,
            "dz_age": dz_age,
            "config": np.array([window, trim, mode], dtype=np.int32),
            "counters_before": np.array(before, dtype=np.int64),
            "counters_after": np.array(after, dtype=np.int64),
        }, {
            "phase": "deadzone",
            "maneuver": maneuver,
            "interval_us": args.interval_us,
            "samples": args.samples,
            "timestamp": stamp,
            "repetition": rep + 1,
        })

    restore_defaults(interface)
    print("\nDeadzone phase done. Run analyze_filter_experiment.py next.")
    return 0


def phase_firmware(interface, args):
    """Validate the firmware-side use of the hardware dead-zone flags.

    Unlike the other phases (which read the filter block directly), this one
    records the normal STREAMED STATE messages, i.e. the angle after
    process_angle()/treat_deadangle_with_derivative() — exactly what the
    controller sees. During a dead-zone episode the firmware must extrapolate
    the angle (no flip to the opposite side), hold the derivative, and pulse
    invalid_steps.
    """
    from globals import ANGLE_360_DEG_IN_ADC_UNITS  # deferred: not needed by other phases

    print("\n=== FIRMWARE: dead-zone handling in the control path ===")
    print("Same maneuver as the 'deadzone' phase: one moderate release above the")
    print("dead-zone side; early swings cross the zone, later ones turn around in it.\n")
    _, window, trim, mode = DEFAULT_CONFIG
    interface.set_angle_filter(window, trim, mode)

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    for rep in range(args.repetitions):
        print(f"\nRepetition {rep + 1}/{args.repetitions} (recording ~{args.duration_s:.0f} s of state stream).")
        print("  Lift the pole somewhat above the dead-zone side.")
        input("  Press Enter, then release on GO...")
        for count in (3, 2, 1):
            print(f"  {count}...")
            time.sleep(1.0)
        print("  GO — release now!")

        interface.stream_output(True)
        angles, angleDs, invalids, chip_times = [], [], [], []
        t_start = time.time()
        try:
            while time.time() - t_start < args.duration_s:
                (angle, angleD, _position, _target, _cmd, invalid_steps,
                 _dt, chip_time, _lat, _latv) = interface.read_state()
                angles.append(angle)
                angleDs.append(angleD)
                invalids.append(invalid_steps)
                chip_times.append(chip_time)
        finally:
            interface.stream_output(False)

        angles = np.asarray(angles, dtype=np.float64)
        angleDs = np.asarray(angleDs, dtype=np.float64)
        invalids = np.asarray(invalids, dtype=np.int64)
        chip_times = np.asarray(chip_times, dtype=np.float64)

        flagged = int(np.count_nonzero(invalids))
        print(f"  Recorded {len(angles)} state messages; polls flagged contaminated: {flagged}")
        if flagged == 0:
            print("  WARNING: invalid_steps never pulsed — either the pole missed the")
            print("  dead zone (release higher) or the firmware on the board predates")
            print("  the dead-zone handling (reflash).")

        path = os.path.join(OUTPUT_DIR, f"firmware_swing_{stamp}_rep{rep + 1}.npz")
        save_npz(path, {
            "angle": angles,
            "angleD": angleDs,
            "invalid_steps": invalids,
            "chip_time": chip_times,
            "config": np.array([window, trim, mode], dtype=np.int32),
        }, {
            "phase": "firmware",
            "duration_s": args.duration_s,
            "timestamp": stamp,
            "repetition": rep + 1,
            "angle_360_adc": float(ANGLE_360_DEG_IN_ADC_UNITS),
        })

    restore_defaults(interface)
    print("\nFirmware phase done. Run analyze_filter_experiment.py next.")
    return 0


def phase_set(interface, args):
    interface.set_angle_filter(args.window, args.trim, args.mode)
    print(f"Filter set: window={args.window}, trim={args.trim}, mode={MODE_NAMES[args.mode]}")
    print("This setting persists until the board is reset or 'set' is run again.")
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("phase", choices=["check", "static", "dynamic", "deadzone", "firmware", "set"])
    parser.add_argument("--port", default=None, help="Serial port, e.g. /dev/ttyUSB0. Auto-detected if omitted.")
    parser.add_argument("--baud", type=int, default=None, help=f"Baud rate (default from globals: {SERIAL_BAUD}).")
    parser.add_argument("--samples", type=int, default=None,
                        help="Pairs per collection (max 16384). Default: 10000 static, 16384 dynamic.")
    parser.add_argument("--interval-us", type=int, default=None,
                        help="Sampling interval in us. Default: 100 static, 500 dynamic.")
    parser.add_argument("--repetitions", type=int, default=3, help="Dynamic phase repetitions. Default: 3.")
    parser.add_argument("--duration-s", type=float, default=16.0,
                        help="'firmware' phase: seconds of state stream per repetition. Default: 16.")
    parser.add_argument("--window", type=int, default=63, help="'set' phase: window size (1-64).")
    parser.add_argument("--trim", type=int, default=7, help="'set' phase: samples trimmed per side.")
    parser.add_argument("--mode", type=int, default=2, help="'set' phase: 0=raw, 1=median, 2=trimmed mean.")
    args = parser.parse_args()

    if args.samples is None:
        args.samples = 16384 if args.phase in ("dynamic", "deadzone") else 10000
    if args.interval_us is None:
        # deadzone records longer (~16 s) to capture the swing decaying from
        # crossings through the turnaround band; dz_age still catches rail
        # contacts between the sparser samples.
        args.interval_us = 1000 if args.phase == "deadzone" else (500 if args.phase == "dynamic" else 100)
    if args.samples > 16384:
        parser.error("--samples is capped at 16384 by the firmware buffer")
    return args


def main():
    args = parse_args()
    interface = open_interface(args.port, args.baud)
    try:
        if args.phase == "check":
            return phase_check(interface, args)
        if args.phase == "static":
            return phase_static(interface, args)
        if args.phase == "set":
            return phase_set(interface, args)
        if args.phase == "deadzone":
            return phase_deadzone(interface, args)
        if args.phase == "firmware":
            return phase_firmware(interface, args)
        return phase_dynamic(interface, args)
    finally:
        if args.phase != "set":
            try:
                restore_defaults(interface)
            except Exception:
                pass
        interface.set_motor(0)
        interface.close()


if __name__ == "__main__":
    sys.exit(main())
