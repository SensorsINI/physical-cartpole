#!/usr/bin/env python3
"""Plot Pmod slider decoded ADC vs travel from the left stop.

  python3 tools/slider_pmod/plot_slider_curve.py tools/slider_pmod/data/YYYY-MM-DD

Writes curve_from_left.csv, sweep_windows.csv, slider_curve.png, and
slider_sweeps_raw.png next to the CSVs. Prints affine residual and suggested rails.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from slider_curve import (
    affine_residual,
    crossing,
    discover_sweeps,
    extract_sweep,
    firmware_constants_snippet,
    mean_grid,
    resample_from_left,
)

FINE_PCTS = list(range(0, 101))
TABLE_PCTS = list(range(0, 101, 10))


def load_session(data_dir: Path) -> tuple[list, list[dict[int, float]], list[dict[int, float]]]:
    sweeps = []
    l2r, r2l = [], []
    for path in discover_sweeps(data_dir):
        sweep = extract_sweep(path)
        fine = resample_from_left(sweep, FINE_PCTS)
        sweeps.append((sweep, fine))
        (l2r if sweep.kind == "L2R" else r2l).append(fine)
    if not sweeps:
        raise SystemExit(f"no slider_L2R_*.csv / slider_R2L_*.csv in {data_dir}")
    return sweeps, l2r, r2l


def write_tables(data_dir: Path, sweeps, l2r, r2l, both, linear) -> None:
    windows_path = data_dir / "sweep_windows.csv"
    with windows_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "file",
                "dir",
                "left_park",
                "right_park",
                "motion_start",
                "motion_end",
                "motion_s",
                "n_raw",
                "n_stripped",
                "n_motion",
            ]
        )
        for sweep, _ in sweeps:
            w.writerow(
                [
                    sweep.path.name,
                    sweep.kind,
                    f"{sweep.left_park:.1f}",
                    f"{sweep.right_park:.1f}",
                    sweep.start_decoded,
                    sweep.end_decoded,
                    f"{sweep.motion_s:.2f}",
                    len(sweep.raw),
                    len(sweep.stripped),
                    len(sweep.motion),
                ]
            )

    curve_path = data_dir / "curve_from_left.csv"
    names = [sweep.path.stem for sweep, _ in sweeps]
    with curve_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["from_left_pct"]
            + names
            + ["L2R_mean", "R2L_mean", "both_mean", "linear", "L2R_minus_R2L"]
        )
        l2r_mean = mean_grid(l2r, FINE_PCTS)
        r2l_mean = mean_grid(r2l, FINE_PCTS)
        for pct in FINE_PCTS:
            row = [pct]
            for _, fine in sweeps:
                row.append(f"{fine[pct]:.2f}")
            gap = l2r_mean[pct] - r2l_mean[pct]
            row.extend(
                [
                    f"{l2r_mean[pct]:.2f}",
                    f"{r2l_mean[pct]:.2f}",
                    f"{both[pct]:.2f}",
                    f"{linear[pct]:.2f}",
                    f"{gap:.2f}",
                ]
            )
            w.writerow(row)


def plot_curve(
    data_dir: Path,
    sweeps,
    l2r,
    r2l,
    both,
    linear,
    mid_adc: float,
    mid_pct: float | None,
) -> None:
    l2r_mean = mean_grid(l2r, FINE_PCTS)
    r2l_mean = mean_grid(r2l, FINE_PCTS)
    xs = FINE_PCTS
    resid, resid_pct = affine_residual(both)
    span = both[100] - both[0]

    fig, (ax, axr) = plt.subplots(
        2, 1, figsize=(8.2, 7.0), sharex=True, gridspec_kw={"height_ratios": [3, 1]}
    )
    for sweep, fine in sweeps:
        color = "#1f77b4" if sweep.kind == "L2R" else "#d62728"
        ys = [fine[p] for p in xs]
        ax.plot(xs, ys, color=color, alpha=0.28, lw=1.1, label="_nolegend_")
    ax.plot(xs, [l2r_mean[p] for p in xs], color="#1f77b4", lw=2.2, label="L2R mean (n=3)")
    ax.plot(xs, [r2l_mean[p] for p in xs], color="#d62728", lw=2.2, label="R2L mean (n=3)")
    ax.plot(xs, [both[p] for p in xs], color="black", lw=2.4, label="both mean")
    ax.plot(
        xs,
        [linear[p] for p in xs],
        color="0.45",
        ls="--",
        lw=1.4,
        label=f"affine {linear[0]:.0f}–{linear[100]:.0f}",
    )
    ax.axhline(mid_adc, color="#2ca02c", ls=":", lw=1.3, label=f"electrical mid {mid_adc:.0f}")
    if mid_pct is not None:
        ax.axvline(mid_pct, color="#2ca02c", ls=":", lw=1.0, alpha=0.7)
        ax.plot([mid_pct], [mid_adc], "o", color="#2ca02c", ms=5)
        ax.annotate(
            f"{mid_adc:.0f} at {mid_pct:.0f}% from left",
            xy=(mid_pct, mid_adc),
            xytext=(min(mid_pct + 8, 72), mid_adc - 0.12 * span),
            fontsize=8,
            color="#2ca02c",
            arrowprops=dict(arrowstyle="->", color="#2ca02c", lw=0.8),
        )
    ax.set_ylabel("Decoded ADC")
    ax.set_title("Pmod slider: decoded ADC vs physical travel")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 4095)
    ax.set_xticks(TABLE_PCTS)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", framealpha=0.92)

    residual = [both[p] - linear[p] for p in xs]
    axr.axhline(0.0, color="0.45", ls="--", lw=1.0)
    axr.plot(xs, residual, color="black", lw=1.6)
    axr.set_xlabel("Travel from left stop (%)")
    axr.set_ylabel("ADC − affine")
    axr.set_xticks(TABLE_PCTS)
    axr.grid(True, alpha=0.3)
    axr.set_title(
        f"max |residual| = {resid:.1f} counts at {resid_pct}% "
        f"({100.0 * resid / span:.1f}% of span)",
        fontsize=9,
    )
    fig.text(
        0.01,
        0.01,
        f"{data_dir.name}  ·  leftover UART prefix stripped",
        fontsize=7.5,
        color="0.35",
    )
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(data_dir / "slider_curve.png", dpi=160)
    plt.close(fig)


def plot_raw(data_dir: Path, sweeps) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(10.4, 6.0), sharey=True)
    order = ["L2R_1", "L2R_2", "L2R_3", "R2L_1", "R2L_2", "R2L_3"]
    by_name = {sweep.path.stem.replace("slider_", ""): sweep for sweep, _ in sweeps}
    for ax, key in zip(axes.ravel(), order):
        sweep = by_name[key]
        t = [s.t for s in sweep.raw]
        y = [s.decoded for s in sweep.raw]
        ax.plot(t, y, color="0.55", lw=0.7)
        mot = sweep.motion
        ax.plot([s.t for s in mot], [s.decoded for s in mot], color="#1f77b4" if sweep.kind == "L2R" else "#d62728", lw=1.3)
        ax.axvspan(mot[0].t, mot[-1].t, color="0.85", zorder=0)
        ax.set_title(f"{key}  {sweep.motion_s:.1f}s", fontsize=9)
        ax.set_xlabel("t (s)", fontsize=8)
        ax.grid(True, alpha=0.25)
    axes[0, 0].set_ylabel("Decoded ADC")
    axes[1, 0].set_ylabel("Decoded ADC")
    fig.suptitle("Raw captures (grey) and rail-to-rail window (color)")
    fig.tight_layout()
    fig.savefig(data_dir / "slider_sweeps_raw.png", dpi=160)
    plt.close(fig)


def print_table(
    l2r, r2l, both, linear, mid_adc: float, mid_pct: float | None
) -> None:
    l2r_mean = mean_grid(l2r, TABLE_PCTS)
    r2l_mean = mean_grid(r2l, TABLE_PCTS)
    print("from_left%   L2R mean   R2L mean   both mean   |L2R-R2L|   affine")
    for pct in TABLE_PCTS:
        gap = abs(l2r_mean[pct] - r2l_mean[pct])
        print(
            f"   {pct:3d}%    {l2r_mean[pct]:8.1f}  {r2l_mean[pct]:8.1f}  "
            f"{both[pct]:8.1f}   {gap:8.1f}  {linear[pct]:8.1f}"
        )
    resid, resid_pct = affine_residual(both)
    span = both[100] - both[0]
    print(
        f"\nAffine residual: max |ADC − linear| = {resid:.1f} at {resid_pct}% "
        f"({100.0 * resid / span:.1f}% of span)."
    )
    print(
        f"Electrical mid {mid_adc:.0f} is at "
        f"{mid_pct:.1f}% from the left stop." if mid_pct is not None else ""
    )
    print(f"Travel mid (50%) decoded = {both[50]:.0f}.")
    print("Suggested rails (zero = electrical mid):")
    print(firmware_constants_snippet(both[0], both[100]).rstrip())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "data_dir",
        type=Path,
        help="directory with slider_L2R_*.csv / slider_R2L_*.csv",
    )
    args = parser.parse_args()
    data_dir = args.data_dir.resolve()

    sweeps, l2r, r2l = load_session(data_dir)
    both = mean_grid(l2r + r2l, FINE_PCTS)
    linear = {p: both[0] + p / 100.0 * (both[100] - both[0]) for p in FINE_PCTS}
    mid_adc = 0.5 * (both[0] + both[100])
    mid_pct = crossing(both, mid_adc)
    resid, resid_pct = affine_residual(both)

    print("sweeps:")
    for sweep, _ in sweeps:
        print(
            f"  {sweep.path.name:20s}  {sweep.kind}  "
            f"park {sweep.left_park:.0f}–{sweep.right_park:.0f}  "
            f"motion {sweep.motion_s:.1f}s  "
            f"{sweep.start_decoded} → {sweep.end_decoded}"
        )
    print()
    print_table(l2r, r2l, both, linear, mid_adc, mid_pct)

    write_tables(data_dir, sweeps, l2r, r2l, both, linear)
    plot_curve(data_dir, sweeps, l2r, r2l, both, linear, mid_adc, mid_pct)
    plot_raw(data_dir, sweeps)

    summary = {
        "data_dir": str(data_dir),
        "n_sweeps": len(sweeps),
        "electrical_mid_adc": mid_adc,
        "electrical_mid_from_left_pct": mid_pct,
        "travel_mid_decoded": both[50],
        "left_park_mean": both[0],
        "right_park_mean": both[100],
        "affine_residual_adc": resid,
        "affine_residual_pct": resid_pct,
        "adc_from_left_pct": {str(p): round(both[p], 1) for p in TABLE_PCTS},
    }
    (data_dir / "curve_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"\nwrote {data_dir / 'slider_curve.png'}")
    print(f"wrote {data_dir / 'slider_sweeps_raw.png'}")
    print(f"wrote {data_dir / 'curve_from_left.csv'}")


if __name__ == "__main__":
    main()
