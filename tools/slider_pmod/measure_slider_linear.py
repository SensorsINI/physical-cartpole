#!/usr/bin/env python3
"""Check CartPoleFirmware target_position against the affine JB slider map.

Close the cartpole GUI. Leave on-chip control OFF. Run from anywhere:

  python3 /path/to/physical-cartpole/tools/slider_pmod/measure_slider_linear.py
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Driver"))
sys.path.insert(0, str(REPO / "Driver" / "DriverFunctions"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from interface import Interface
from slider_curve import (
    SLIDER_ADC_LEFT,
    SLIDER_ADC_RIGHT,
    SLIDER_TARGET_HALF_LENGTH,
    adc_from_normed,
    firmware_constants_snippet,
    normed_from_adc,
    normed_to_from_left,
)
from zynq_serial import prefer_zynq_uart_port

TRACK_HALF = SLIDER_TARGET_HALF_LENGTH
BAUD = 230400
POSES = (
    ("LEFT stop (min ADC)", -1.0, 0.0),
    ("RIGHT stop (max ADC)", 1.0, 1.0),
)
CLAMP_EPS_M = 0.002


def target_to_from_left(target: float) -> float:
    return normed_to_from_left(target / TRACK_HALF)


def led_name(target: float) -> str:
    if abs(target) < 0.004:
        return "near-zero"
    return "green (+)" if target > 0 else "blue (-)"


def _read_state_no_flush(iface: Interface):
    """Interface.read_state() calls clear_read_buffer() first, which cuts a live
    STATE packet in half and prints CRC Failed / Missed CMD until the next SOF.
    Skip that flush while we are already locked on the stream."""
    saved = iface.clear_read_buffer
    iface.clear_read_buffer = lambda: None
    try:
        return iface.read_state()
    finally:
        iface.clear_read_buffer = saved


def sample(iface: Interface, seconds: float = 1.2) -> tuple[float, float, float, int]:
    vals = []
    t0 = time.time()
    end = t0 + seconds
    while time.time() < end:
        try:
            _a, _ad, _p, target, *_rest = _read_state_no_flush(iface)
        except Exception as e:
            print(f"  read failed: {e}")
            continue
        vals.append(float(target))
    if not vals:
        raise RuntimeError("no state packets")
    vals.sort()
    return vals[len(vals) // 2], vals[0], vals[-1], len(vals)


def print_reading(label: str, med: float, lo: float, hi: float) -> None:
    fl = target_to_from_left(med)
    print(
        f"  {label:28s}  target={med:+.4f} m  "
        f"travel={fl * 100:5.1f}%  jitter {lo:+.4f}..{hi:+.4f}  LED~{led_name(med)}"
    )


def watch(iface: Interface, seconds: float) -> None:
    print(f"Live target for {seconds:.0f}s. Move the slider; Ctrl+C to stop.")
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < seconds:
        try:
            _a, _ad, _p, target, *_rest = _read_state_no_flush(iface)
        except Exception as e:
            print(f"  read failed: {e}")
            time.sleep(0.2)
            continue
        now = time.time() - t0
        if now - last >= 0.25:
            fl = target_to_from_left(float(target))
            print(
                f"  t={now:5.1f}s  target={float(target):+.4f} m  "
                f"travel={fl * 100:5.1f}%  LED~{led_name(float(target))}",
                flush=True,
            )
            last = now


def poses(iface: Interface) -> None:
    print("Park still at each rail, then press Enter.")
    print(
        f"Expect LEFT ≈ −{TRACK_HALF:.3f} m, RIGHT ≈ +{TRACK_HALF:.3f} m "
        "if the pot reaches the firmware rails. Zero is (LEFT+RIGHT)/2."
    )
    rows = []
    for label, _exp_norm, _exp_fl in POSES:
        try:
            iface.stream_output(False)
        except Exception:
            pass
        input(f"\nPark at {label}, then press Enter...")
        iface.stream_output(True)
        time.sleep(0.25)
        try:
            _read_state_no_flush(iface)
        except Exception:
            pass
        med, lo, hi, n = sample(iface)
        print_reading(label, med, lo, hi)
        if hi - lo > 0.02:
            print(
                f"  jitter {hi - lo:.3f} m is large — park still, then Enter again "
                "if this was during a move."
            )
        got_fl = target_to_from_left(med)
        inferred_adc = adc_from_normed(med / TRACK_HALF)
        rows.append((label, med, got_fl, n, inferred_adc, lo, hi))
    print("\n==== min / max (zero will be the electrical mid) ====")
    print(f"{'pose':28s}  target      travel    inferred ADC")
    for label, med, got_fl, n, inferred_adc, lo, hi in rows:
        print(
            f"{label:28s}  {med:+.4f} m  {got_fl * 100:5.1f}%  "
            f"{inferred_adc:7.1f}   n={n}  jitter {lo:+.4f}..{hi:+.4f}"
        )
    left = next(r for r in rows if r[0].startswith("LEFT"))
    right = next(r for r in rows if r[0].startswith("RIGHT"))

    left_clamped = abs(left[1] + TRACK_HALF) < CLAMP_EPS_M
    right_clamped = abs(right[1] - TRACK_HALF) < CLAMP_EPS_M
    adc_l, adc_r = left[4], right[4]
    at_adc_floor = left_clamped and SLIDER_ADC_LEFT <= 0.5
    at_adc_ceil = right_clamped and SLIDER_ADC_RIGHT >= 4094.5
    if left_clamped and not at_adc_floor:
        print(
            f"LEFT is clamped at ADC {SLIDER_ADC_LEFT:.1f} "
            f"(target −{TRACK_HALF:.3f} m). True min may be lower."
        )
    elif at_adc_floor:
        print("LEFT saturates the 12-bit ADC (0). That is the rail.")
    if right_clamped and not at_adc_ceil:
        print(
            f"RIGHT is clamped at ADC {SLIDER_ADC_RIGHT:.1f} "
            f"(target +{TRACK_HALF:.3f} m). True max may be higher."
        )
    elif at_adc_ceil:
        print("RIGHT saturates the 12-bit ADC (4095). That is the rail.")
    if adc_r > adc_l:
        mid_adc = 0.5 * (adc_l + adc_r)
        print(f"\nElectrical mid ADC = {mid_adc:.1f}  (target 0 after we install these rails).")
        print("Suggested firmware constants:")
        print(firmware_constants_snippet(adc_l, adc_r).rstrip())
        print("Paste that printout and I will put the rails in.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", action="store_true", help="print live target")
    parser.add_argument("--seconds", type=float, default=90.0)
    args = parser.parse_args()
    port = prefer_zynq_uart_port("/dev/ttyUSB1")
    print(f"Opening {port} at {BAUD}. Close the GUI if this fails.")
    mid_adc = 0.5 * (SLIDER_ADC_LEFT + SLIDER_ADC_RIGHT)
    print(
        f"affine  ADC {SLIDER_ADC_LEFT:.1f}…{SLIDER_ADC_RIGHT:.1f}  "
        f"electrical mid {mid_adc:.1f} → "
        f"normed {normed_from_adc(mid_adc):+.3f}"
    )
    iface = Interface()
    iface.open(port, BAUD)
    try:
        if not iface.ping():
            print("PING failed. Is CartPoleFirmware running? GUI still open?")
            sys.exit(1)
        print("PING ok. Streaming (leave control off).")
        iface.stream_output(True)
        if args.watch:
            watch(iface, args.seconds)
        else:
            poses(iface)
    finally:
        try:
            iface.stream_output(False)
        except Exception:
            pass
        try:
            iface.device.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
