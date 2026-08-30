#!/usr/bin/env python3
"""Capture 3× left→right and 3× right→left sweeps to check the slider is affine.

Use the 50 MHz Slider_test bitstream (115200) so decoded ADC is printed.
Plot the CSVs, then copy the suggested rails into
Firmware/Src/Zynq/external_interface.c and slider_curve.py.

  python3 tools/slider_pmod/log_slider_curve.py
  python3 tools/slider_pmod/log_slider_curve.py --out tools/slider_pmod/data/YYYY-MM-DD

At each sweep: park, Enter, move at an even pace (~15–20 s), Enter when still
at the other stop. The UART buffer is flushed at the start of every sweep so
leftover samples from the previous park are not written into the CSV.
"""
from __future__ import annotations

import argparse
import select
import sys
import time
from datetime import date
from pathlib import Path

import serial
from serial.tools import list_ports

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from slider_curve import (
    LINE_RE,
    SLIDER_ADC_LEFT,
    SLIDER_ADC_RIGHT,
    Sample,
    affine_residual,
    extract_sweep,
    firmware_constants_snippet,
    resample_from_left,
)
SWEEPS = (
    ("L2R_1", "LEFT stop", "RIGHT stop"),
    ("L2R_2", "LEFT stop", "RIGHT stop"),
    ("L2R_3", "LEFT stop", "RIGHT stop"),
    ("R2L_1", "RIGHT stop", "LEFT stop"),
    ("R2L_2", "RIGHT stop", "LEFT stop"),
    ("R2L_3", "RIGHT stop", "LEFT stop"),
)
FLUSH_S = 0.35


def uart_port(default: str = "/dev/ttyUSB1") -> str:
    for p in list_ports.comports():
        loc = getattr(p, "location", "") or ""
        if "Digilent" in (p.description or "") and loc.endswith(".1"):
            return p.device
    return default


def write_csv(path: Path, rows: list[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("t,decoded\n")
        for r in rows:
            f.write(f"{r.t:.4f},{r.decoded}\n")


def record_one(ser, out_dir: Path, label: str, start: str, dest: str) -> Path:
    print(f"\n-------- {label} --------")
    print(f"Park still at {start}, then press Enter.")
    input()
    ser.reset_input_buffer()
    time.sleep(FLUSH_S)
    ser.reset_input_buffer()
    print(f"MOVE to {dest} at a comfortable even pace.")
    print("Press Enter again when you have ARRIVED and are still.", flush=True)
    buf = ""
    rows: list[Sample] = []
    t0 = time.time()
    last_print = 0.0
    while True:
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
        try:
            chunk = ser.read(256)
        except serial.SerialException as e:
            print(f"  UART glitch ({e})", flush=True)
            time.sleep(0.2)
            continue
        if not chunk:
            continue
        buf += chunk.decode("utf-8", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            m = LINE_RE.search(line)
            if not m:
                continue
            now = time.time() - t0
            if now < FLUSH_S:
                continue
            decoded = int(m.group(4))
            rows.append(Sample(now, decoded))
            if now - last_print >= 0.5:
                print(f"  t={now:4.1f}s  decoded={decoded:4d}", flush=True)
                last_print = now
    print(f"  captured {len(rows)} samples", flush=True)
    path = out_dir / f"slider_{label}.csv"
    write_csv(path, rows)
    try:
        sweep = extract_sweep(path, rows)
        grid = resample_from_left(sweep, range(0, 101, 10))
        print(
            f"  {sweep.kind}  motion {sweep.motion_s:.1f}s  "
            f"{sweep.start_decoded} → {sweep.end_decoded}  "
            f"park {sweep.left_park:.0f}–{sweep.right_park:.0f}"
        )
        print("  from_left%  decoded")
        for pct in range(0, 101, 10):
            print(f"     {pct:3d}%     {grid[pct]:7.1f}")
    except ValueError as e:
        print(f"  window warning: {e}")
    return path


def print_average(paths: list[Path]) -> None:
    from slider_curve import mean_grid

    print("\n======== files ========")
    l2r, r2l = [], []
    for path in paths:
        try:
            sweep = extract_sweep(path)
        except ValueError as e:
            print(f"  skip {path.name}: {e}")
            continue
        grid = resample_from_left(sweep, range(0, 101, 10))
        (l2r if sweep.kind == "L2R" else r2l).append(grid)
        print(
            f"  {sweep.kind}  {path.name}  {sweep.start_decoded} → {sweep.end_decoded}  "
            f"motion {sweep.motion_s:.1f}s"
        )
    if not l2r and not r2l:
        return
    both = mean_grid(l2r + r2l, range(0, 101, 10))
    print("\n======== curve (from_left 0 = LEFT stop) ========")
    print("from_left%   L2R mean   R2L mean   both mean   |L2R-R2L|")
    for pct in range(0, 101, 10):
        lv = [g[pct] for g in l2r]
        rv = [g[pct] for g in r2l]

        def fmt(xs: list[float]) -> str:
            return f"{sum(xs) / len(xs):8.1f}" if xs else "     n/a"

        gap = ""
        if lv and rv:
            gap = f"{abs(sum(lv) / len(lv) - sum(rv) / len(rv)):8.1f}"
        print(f"   {pct:3d}%    {fmt(lv)}  {fmt(rv)}  {both[pct]:8.1f}   {gap}")
    resid, resid_pct = affine_residual(both)
    span = both[100] - both[0]
    print(
        f"\nAffine residual: max |ADC − linear| = {resid:.1f} at {resid_pct}% "
        f"({100.0 * resid / span:.1f}% of span)."
    )
    print("Suggested rails (zero = electrical mid):")
    print(firmware_constants_snippet(both[0], both[100]).rstrip())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=None,
        help="UART device (default: Digilent if1, else /dev/ttyUSB1)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=HERE / "data" / date.today().isoformat(),
        help="directory for slider_*.csv",
    )
    args = parser.parse_args()
    out_dir = args.out.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    port = args.port or uart_port()
    print(f"Opening {port} at 115200")
    print(f"Writing CSVs to {out_dir}")
    print(
        f"Firmware rails: LEFT={SLIDER_ADC_LEFT:.0f}  RIGHT={SLIDER_ADC_RIGHT:.0f}  (12-bit)."
    )
    print("Six sweeps: 3 left→right, then 3 right→left.")
    print("Even pace, ~15–20 s moving.")
    ser = serial.Serial(port, 115200, timeout=0.15)
    ser.reset_input_buffer()
    paths: list[Path] = []
    try:
        for label, start, dest in SWEEPS:
            try:
                paths.append(record_one(ser, out_dir, label, start, dest))
            except KeyboardInterrupt:
                print("\nSkipping remaining sweeps.", flush=True)
                break
    finally:
        try:
            ser.close()
        except Exception:
            pass
    print_average(paths)
    print(f"\nRe-plot with:\n  python3 {HERE / 'plot_slider_curve.py'} {out_dir}")


if __name__ == "__main__":
    main()
