"""Load Slider_test sweep CSVs and resample decoded ADC vs travel from the left stop.

Raw captures can start with leftover UART samples from the previous sweep
(the board keeps printing while you park). Those are stripped before the
rail-to-rail window is taken.
"""
from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Iterable

PARK_BAND = 50
STABLE_N = 10
MIN_SWEEP_SPAN = 800
# Firmware SliderTargetHalfLength: pot rails map to ±this, not the 0.198 m track.
SLIDER_TARGET_HALF_LENGTH = 0.12
# 12-bit PmodAD1. Keep SLIDER_ADC_LEFT/RIGHT in sync with
# Firmware/Src/Zynq/external_interface.c. Target 0 is the electrical mid
# (LEFT+RIGHT)/2 — no visual-centre park.
ADC_FULL_SCALE = 4095.0
# Measured 2026-09-01: parked slider saturates the 12-bit PmodAD1.
SLIDER_ADC_LEFT = 0.0
SLIDER_ADC_RIGHT = ADC_FULL_SCALE
LINE_RE = re.compile(
    r"raw=(0x[0-9A-Fa-f]+)\s+ch0=(0x[0-9A-Fa-f]+)\s+ch1=(0x[0-9A-Fa-f]+)"
    r"\s+decoded=\s*(\d+)"
)


@dataclass(frozen=True)
class Sample:
    t: float
    decoded: int


@dataclass
class Sweep:
    path: Path
    kind: str  # "L2R" or "R2L"
    raw: list[Sample]
    stripped: list[Sample]
    i0: int
    i1: int
    left_park: float
    right_park: float

    @property
    def motion(self) -> list[Sample]:
        return self.stripped[self.i0 : self.i1 + 1]

    @property
    def motion_s(self) -> float:
        mot = self.motion
        return mot[-1].t - mot[0].t

    @property
    def start_decoded(self) -> int:
        return self.motion[0].decoded

    @property
    def end_decoded(self) -> int:
        return self.motion[-1].decoded


def parse_kind(name: str) -> str:
    upper = name.upper()
    if "R2L" in upper:
        return "R2L"
    if "L2R" in upper:
        return "L2R"
    raise ValueError(f"cannot tell L2R vs R2L from {name!r}")


def load_rows(path: Path) -> list[Sample]:
    rows: list[Sample] = []
    with Path(path).open() as f:
        for r in csv.DictReader(f):
            rows.append(Sample(float(r["t"]), int(r["decoded"])))
    return rows


def _first_stable(decoded: list[int], pred, n: int = STABLE_N) -> int | None:
    run = 0
    for i, d in enumerate(decoded):
        if pred(d):
            run += 1
            if run >= n:
                return i - n + 1
        else:
            run = 0
    return None


def strip_stale_prefix(rows: list[Sample], kind: str) -> list[Sample]:
    decoded = [s.decoded for s in rows]
    lo, hi = min(decoded), max(decoded)
    span = hi - lo
    if span < MIN_SWEEP_SPAN:
        return rows
    left_cut = lo + 0.12 * span
    right_cut = hi - 0.12 * span
    if kind == "L2R":
        i = _first_stable(decoded, lambda d: d <= left_cut)
    else:
        i = _first_stable(decoded, lambda d: d >= right_cut)
    return rows if i is None else rows[i:]


def extract_sweep(path: Path, rows: list[Sample] | None = None) -> Sweep:
    path = Path(path)
    kind = parse_kind(path.name)
    raw = rows if rows is not None else load_rows(path)
    if len(raw) < 20:
        raise ValueError(f"{path.name}: too few samples ({len(raw)})")
    stripped = strip_stale_prefix(raw, kind)
    dec = [s.decoded for s in stripped]
    k = min(12, max(4, len(stripped) // 20))
    park0 = float(median(dec[:k]))
    park1 = float(median(dec[-k:]))
    if abs(park1 - park0) < MIN_SWEEP_SPAN:
        raise ValueError(
            f"{path.name}: park span {abs(park1 - park0):.0f} < {MIN_SWEEP_SPAN}"
        )
    if kind == "L2R":
        if park1 <= park0:
            raise ValueError(
                f"{path.name}: expected L2R rising, got {park0:.0f}→{park1:.0f}"
            )
        left_park, right_park = park0, park1
    else:
        if park0 <= park1:
            raise ValueError(
                f"{path.name}: expected R2L falling, got {park0:.0f}→{park1:.0f}"
            )
        left_park, right_park = park1, park0

    park_band = max(PARK_BAND, 0.02 * abs(park1 - park0))
    i0 = None
    for i, d in enumerate(dec):
        if abs(d - park0) <= park_band and any(
            abs(x - park1) <= park_band for x in dec[i + 1 :]
        ):
            i0 = i
    if i0 is None:
        raise ValueError(f"{path.name}: never left the start rail")
    i1 = next(
        (i for i in range(i0 + 1, len(dec)) if abs(dec[i] - park1) <= park_band),
        None,
    )
    if i1 is None:
        raise ValueError(f"{path.name}: never reached the end rail")
    return Sweep(
        path=path,
        kind=kind,
        raw=raw,
        stripped=stripped,
        i0=i0,
        i1=i1,
        left_park=left_park,
        right_park=right_park,
    )


def interp_time(motion: list[Sample], frac: float) -> float:
    if not motion:
        raise ValueError("empty motion")
    frac = min(1.0, max(0.0, frac))
    t0, t1 = motion[0].t, motion[-1].t
    target = t0 + frac * (t1 - t0)
    if target <= t0:
        return float(motion[0].decoded)
    if target >= t1:
        return float(motion[-1].decoded)
    for a, b in zip(motion, motion[1:]):
        if a.t <= target <= b.t:
            if b.t == a.t:
                return float(a.decoded)
            w = (target - a.t) / (b.t - a.t)
            return a.decoded + w * (b.decoded - a.decoded)
    return float(motion[-1].decoded)


def resample_from_left(sweep: Sweep, pcts: Iterable[int]) -> dict[int, float]:
    """Map travel from the left stop (0) to the right stop (100) onto decoded ADC."""
    grid: dict[int, float] = {}
    for pct in pcts:
        if pct == 0:
            grid[pct] = sweep.left_park
        elif pct == 100:
            grid[pct] = sweep.right_park
        else:
            frac = pct / 100.0 if sweep.kind == "L2R" else 1.0 - pct / 100.0
            grid[pct] = interp_time(sweep.motion, frac)
    return grid


def mean_grid(grids: list[dict[int, float]], pcts: Iterable[int]) -> dict[int, float]:
    out: dict[int, float] = {}
    for pct in pcts:
        vals = [g[pct] for g in grids if pct in g]
        if vals:
            out[pct] = sum(vals) / len(vals)
    return out


def crossing(grid: dict[int, float], target: float) -> float | None:
    """from_left% where the curve crosses target ADC (linear between integer %)."""
    pcts = sorted(grid)
    for a, b in zip(pcts, pcts[1:]):
        ya, yb = grid[a], grid[b]
        if (ya - target) * (yb - target) <= 0:
            if yb == ya:
                return float(a)
            return a + (target - ya) / (yb - ya) * (b - a)
    return None


def adc_to_from_left(
    adc: float,
    left: float | None = None,
    right: float | None = None,
) -> float:
    """Decoded ADC → travel from the left stop in [0, 1] (affine in ADC)."""
    lo = SLIDER_ADC_LEFT if left is None else left
    hi = SLIDER_ADC_RIGHT if right is None else right
    if adc <= lo:
        return 0.0
    if adc >= hi:
        return 1.0
    return (adc - lo) / (hi - lo)


def from_left_to_adc(
    from_left: float,
    left: float | None = None,
    right: float | None = None,
) -> float:
    lo = SLIDER_ADC_LEFT if left is None else left
    hi = SLIDER_ADC_RIGHT if right is None else right
    u = min(1.0, max(0.0, from_left))
    return lo + u * (hi - lo)


def from_left_to_normed(from_left: float) -> float:
    """−1 at the left rail, 0 at the electrical mid, +1 at the right rail."""
    u = min(1.0, max(0.0, from_left))
    return 2.0 * u - 1.0


def normed_to_from_left(normed: float) -> float:
    """Invert from_left_to_normed."""
    n = max(-1.0, min(1.0, normed))
    return 0.5 * (n + 1.0)


def normed_from_adc(
    adc: float,
    left: float | None = None,
    right: float | None = None,
) -> float:
    """−1 at the left rail, 0 at the electrical mid, +1 at the right rail."""
    return from_left_to_normed(adc_to_from_left(adc, left, right))


def adc_from_normed(
    normed: float,
    left: float | None = None,
    right: float | None = None,
) -> float:
    """Invert normed_from_adc (clamped targets map to the rails)."""
    return from_left_to_adc(normed_to_from_left(normed), left, right)


def affine_residual(grid: dict[int, float]) -> tuple[float, int]:
    """Max |adc − linear(from_left)| over integer-% keys. Returns (residual, pct)."""
    if 0 not in grid or 100 not in grid:
        raise ValueError("grid needs 0% and 100%")
    lo, hi = grid[0], grid[100]
    worst = 0.0
    worst_pct = 0
    for pct, adc in grid.items():
        linear = lo + pct / 100.0 * (hi - lo)
        err = abs(adc - linear)
        if err > worst:
            worst = err
            worst_pct = pct
    return worst, worst_pct


def firmware_constants_snippet(adc_left: float, adc_right: float) -> str:
    return (
        f"#define SLIDER_ADC_LEFT {adc_left:.2f}f\n"
        f"#define SLIDER_ADC_RIGHT {adc_right:.2f}f\n"
    )


def discover_sweeps(data_dir: Path) -> list[Path]:
    paths = sorted(data_dir.glob("slider_L2R_*.csv")) + sorted(
        data_dir.glob("slider_R2L_*.csv")
    )
    return [p for p in paths if p.is_file()]
