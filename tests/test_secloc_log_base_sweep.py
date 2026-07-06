"""Sweep Secloc log_base vs skipped-update rate; plot curves for several noise levels.

Simulates PID @ 5 ms with measurement noise, records noisy states, replays them
through SeclocGate. Noise scale is increased until PID can no longer hold the pole.

Run table tests:
  pytest tests/test_secloc_log_base_sweep.py -v -s

Generate plot (saved under tests/output/):
  pytest tests/test_secloc_log_base_sweep.py -v -s -k plot
  python tests/test_secloc_log_base_sweep.py
"""
from __future__ import annotations

import contextlib
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "Driver"))
sys.path.insert(1, str(REPO_ROOT / "Driver" / "CartPoleSimulation"))
sys.path.insert(2, str(REPO_ROOT / "Driver" / "CartPoleSimulation" / "SI_Toolkit" / "src"))
os.chdir(REPO_ROOT / "Driver")

from CartPoleSimulation.CartPole import CartPole
from CartPoleSimulation.CartPole.cartpole_parameters import TrackHalfLength
from CartPoleSimulation.CartPole.state_utilities import (
    ANGLE_COS_IDX,
    ANGLE_IDX,
    ANGLE_SIN_IDX,
    POSITION_IDX,
    create_cartpole_state,
)
from Control_Toolkit_ASF.Controllers.secloc_gate import SeclocGate

POLL_DT_S = 0.005
DT_SIM_S = 0.002
WARMUP_S = 3.0
RECORD_S = 20.0

# Base measurement noise (scaled by noise_scale when exploring).
BASE_SIGMA_ANGLE = 0.0015  # rad
BASE_SIGMA_POSITION = 0.0005  # m
BASE_SIGMA_ANGLED = 0.075
BASE_SIGMA_POSITIOND = 0.005

DEFAULT_REF_PERIOD = 0.0
DEFAULT_DEAD_ANG = 0.0
DEFAULT_DEAD_POS = 0.0

# Control considered lost if true state exceeds these during the record window.
MAX_STABLE_ANGLE_RAD = 0.45
MAX_STABLE_POSITION_M = 0.85 * TrackHalfLength

LOG_BASE_SWEEP = np.round(np.arange(1.00, 1.151, 0.005), 3)
NOISE_SCALE_CANDIDATES = (
    1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0,
    25.0, 32.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0,
)

OUTPUT_DIR = REPO_ROOT / "tests" / "output"
PLOT_PATH = OUTPUT_DIR / "secloc_log_base_vs_skip_pct_noise_sweep.png"


@dataclass(frozen=True)
class TrajectorySample:
    time: float
    s: np.ndarray
    target_position: float


@dataclass(frozen=True)
class TrajectoryRecord:
    samples: list[TrajectorySample]
    noise_scale: float
    sigma_angle: float
    stable: bool
    max_angle_rad: float
    max_position_m: float


@dataclass(frozen=True)
class GateSweepResult:
    log_base: float
    skipped_pct: float
    updates: int
    total: int


@contextlib.contextmanager
def measurement_noise(sigmas: dict[str, float]):
    import CartPole.noise_adder as noise_adder

    saved = (
        noise_adder.sigma_angle,
        noise_adder.sigma_position,
        noise_adder.sigma_angleD,
        noise_adder.sigma_positionD,
        noise_adder.NOISE_MODE,
    )
    noise_adder.sigma_angle = sigmas["sigma_angle"]
    noise_adder.sigma_position = sigmas["sigma_position"]
    noise_adder.sigma_angleD = sigmas["sigma_angleD"]
    noise_adder.sigma_positionD = sigmas["sigma_positionD"]
    noise_adder.NOISE_MODE = "gaussian"
    try:
        yield
    finally:
        (
            noise_adder.sigma_angle,
            noise_adder.sigma_position,
            noise_adder.sigma_angleD,
            noise_adder.sigma_positionD,
            noise_adder.NOISE_MODE,
        ) = saved


def sigmas_for_scale(noise_scale: float) -> dict[str, float]:
    return {
        "sigma_angle": BASE_SIGMA_ANGLE * noise_scale,
        "sigma_position": BASE_SIGMA_POSITION * noise_scale,
        "sigma_angleD": BASE_SIGMA_ANGLED * noise_scale,
        "sigma_positionD": BASE_SIGMA_POSITIOND * noise_scale,
    }


def _steps_per_poll() -> int:
    return max(1, int(round(POLL_DT_S / DT_SIM_S)))


def record_stabilized_trajectory(
    *,
    noise_scale: float = 1.0,
    seed: int = 123,
    warmup_s: float = WARMUP_S,
    record_s: float = RECORD_S,
    initial_angle_rad: float = 0.12,
) -> TrajectoryRecord:
    sigmas = sigmas_for_scale(noise_scale)
    max_angle = 0.0
    max_position = 0.0
    samples: list[TrajectorySample] = []

    with measurement_noise(sigmas):
        cp = CartPole()
        cp.dt_simulation = DT_SIM_S
        cp.dt_controller = POLL_DT_S
        cp.set_controller("pid")

        cp.s = create_cartpole_state()
        cp.s[ANGLE_IDX] = initial_angle_rad
        cp.s[ANGLE_COS_IDX] = np.cos(initial_angle_rad)
        cp.s[ANGLE_SIN_IDX] = np.sin(initial_angle_rad)
        cp.target_position = 0.0
        cp.time = 0.0
        cp.NoiseAdderInstance.noise_mode = "gaussian"
        cp.NoiseAdderInstance.rng_noise_adder = np.random.default_rng(seed)

        steps_per_poll = _steps_per_poll()
        warmup_polls = int(warmup_s / POLL_DT_S)
        record_polls = int(record_s / POLL_DT_S)
        recording = False

        for poll_idx in range(warmup_polls + record_polls):
            for _ in range(steps_per_poll):
                cp.update_state()

            max_angle = max(max_angle, abs(float(cp.s[ANGLE_IDX])))
            max_position = max(max_position, abs(float(cp.s[POSITION_IDX])))

            if poll_idx >= warmup_polls:
                recording = True
                samples.append(
                    TrajectorySample(
                        time=cp.time,
                        s=np.array(cp.s_with_noise_and_latency, copy=True),
                        target_position=float(cp.target_position),
                    )
                )

    stable = (
        recording
        and max_angle <= MAX_STABLE_ANGLE_RAD
        and max_position <= MAX_STABLE_POSITION_M
        and len(samples) > 100
    )
    return TrajectoryRecord(
        samples=samples,
        noise_scale=noise_scale,
        sigma_angle=sigmas["sigma_angle"],
        stable=stable,
        max_angle_rad=max_angle,
        max_position_m=max_position,
    )


def measure_skip_rate(
    trajectory: list[TrajectorySample],
    *,
    log_base: float,
    ref_period: float = DEFAULT_REF_PERIOD,
    dead_ang: float = DEFAULT_DEAD_ANG,
    dead_pos: float = DEFAULT_DEAD_POS,
) -> GateSweepResult:
    gate = SeclocGate(
        log_base=float(log_base),
        ref_period=float(ref_period),
        dead_ang=float(dead_ang),
        dead_pos=float(dead_pos),
    )
    for sample in trajectory:
        gate.should_sample(
            sample.s,
            sample.target_position,
            time=sample.time,
        )
    return GateSweepResult(
        log_base=float(log_base),
        skipped_pct=gate.skipped_update_percentage,
        updates=gate.update_decisions,
        total=gate.total_decisions,
    )


def sweep_log_base(
    trajectory: list[TrajectorySample],
    log_bases: np.ndarray | list[float] | None = None,
    **gate_kwargs,
) -> list[GateSweepResult]:
    if log_bases is None:
        log_bases = LOG_BASE_SWEEP
    return [
        measure_skip_rate(trajectory, log_base=lb, **gate_kwargs)
        for lb in log_bases
    ]


def explore_noise_scales_until_break(
    *,
    seed: int = 123,
    scales: tuple[float, ...] = NOISE_SCALE_CANDIDATES,
) -> list[TrajectoryRecord]:
    """Increase noise scale; stop after first level where PID loses stability."""
    records: list[TrajectoryRecord] = []
    for scale in scales:
        record = record_stabilized_trajectory(noise_scale=scale, seed=seed)
        records.append(record)
        if not record.stable:
            break
    return records


def sweep_skip_curves_by_noise(
    records: list[TrajectoryRecord],
    log_bases: np.ndarray | list[float] | None = None,
) -> dict[float, list[GateSweepResult]]:
    curves: dict[float, list[GateSweepResult]] = {}
    for record in records:
        if not record.stable:
            continue
        curves[record.noise_scale] = sweep_log_base(record.samples, log_bases)
    return curves


def format_sweep_table(results: list[GateSweepResult]) -> str:
    lines = [
        "log_base  skipped%   updates   total",
        "--------  ---------  -------  ------",
    ]
    for row in results:
        lines.append(
            f"{row.log_base:8.3f}  {row.skipped_pct:8.1f}  "
            f"{row.updates:7d}  {row.total:6d}"
        )
    return "\n".join(lines)


def closest_log_base_to_target_skip(
    results: list[GateSweepResult],
    target_skip_pct: float,
) -> GateSweepResult:
    return min(results, key=lambda row: abs(row.skipped_pct - target_skip_pct))


def plot_skip_vs_log_base_by_noise(
    curves: dict[float, list[GateSweepResult]],
    records: list[TrajectoryRecord],
    output_path: Path = PLOT_PATH,
) -> Path:
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for noise_scale in sorted(curves):
        rows = curves[noise_scale]
        rec = next(r for r in records if r.noise_scale == noise_scale)
        label = (
            f"σ×{noise_scale:g} "
            f"(σ_θ={rec.sigma_angle*1e3:.2f} mrad, "
            f"max|θ|={rec.max_angle_rad:.2f} rad)"
        )
        ax.plot(
            [r.log_base for r in rows],
            [r.skipped_pct for r in rows],
            marker="o",
            markersize=3,
            linewidth=1.5,
            label=label,
        )

    broken = [r for r in records if not r.stable]
    if broken:
        r = broken[0]
        ax.annotate(
            f"PID lost control at σ×{r.noise_scale:g}\n"
            f"(max|θ|={r.max_angle_rad:.2f} rad)",
            xy=(0.02, 0.02),
            xycoords="axes fraction",
            fontsize=9,
            va="bottom",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
        )

    ax.set_xlabel("Secloc log_base")
    ax.set_ylabel("Skipped controller updates (%)")
    ax.set_title(
        "Secloc skip rate vs log_base\n"
        f"PID simulation, {POLL_DT_S*1e3:.0f} ms polls, {RECORD_S:.0f} s record window"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_xlim(LOG_BASE_SWEEP[0], LOG_BASE_SWEEP[-1])
    ax.set_ylim(0, min(100, ax.get_ylim()[1] + 5))

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


@pytest.fixture(scope="module")
def stabilized_trajectory() -> TrajectoryRecord:
    return record_stabilized_trajectory(noise_scale=1.0, seed=123)


def test_secloc_log_base_sweep_table(stabilized_trajectory, capsys):
    results = sweep_log_base(stabilized_trajectory.samples)
    table = format_sweep_table(results)
    print("\nSecloc log_base sweep (5 ms polls, PID + measurement noise):\n")
    print(table)

    by_base = {row.log_base: row for row in results}
    assert by_base[1.05].total == by_base[1.00].total
    assert by_base[1.05].total > 100
    assert by_base[1.10].updates <= by_base[1.00].updates


def test_log_base_1_triggers_more_than_1_05(stabilized_trajectory):
    low = measure_skip_rate(stabilized_trajectory.samples, log_base=1.0)
    mid = measure_skip_rate(stabilized_trajectory.samples, log_base=1.05)
    print(
        f"\nlog_base 1.00: skipped {low.skipped_pct:.1f}% "
        f"({low.updates}/{low.total} updates)"
    )
    print(
        f"log_base 1.05: skipped {mid.skipped_pct:.1f}% "
        f"({mid.updates}/{mid.total} updates)"
    )
    assert low.skipped_pct < mid.skipped_pct
    assert low.updates > mid.updates


def test_report_closest_log_base_to_10_percent_skip(stabilized_trajectory, capsys):
    log_bases = np.round(np.arange(1.00, 1.201, 0.002), 3)
    results = sweep_log_base(stabilized_trajectory.samples, log_bases)
    closest = closest_log_base_to_target_skip(results, target_skip_pct=10.0)
    print(
        f"\nClosest to 10% skipped: log_base={closest.log_base:.3f} "
        f"-> {closest.skipped_pct:.1f}% skipped "
        f"({closest.updates}/{closest.total} updates)"
    )
    assert closest.total > 0


@pytest.mark.slow
def test_plot_secloc_skip_vs_log_base_noise_sweep(capsys):
    records = explore_noise_scales_until_break(seed=123)
    stable_records = [r for r in records if r.stable]
    assert len(stable_records) >= 2, "need at least two stable noise levels for a plot"

    curves = sweep_skip_curves_by_noise(records)
    path = plot_skip_vs_log_base_by_noise(curves, records)

    print(f"\nStable noise levels: {[r.noise_scale for r in stable_records]}")
    if any(not r.stable for r in records):
        broken = next(r for r in records if not r.stable)
        print(
            f"First broken level: σ×{broken.noise_scale:g} "
            f"(max|θ|={broken.max_angle_rad:.2f} rad)"
        )
    print(f"Plot saved to: {path}")
    assert path.is_file()


if __name__ == "__main__":
    records = explore_noise_scales_until_break(seed=123)
    curves = sweep_skip_curves_by_noise(records)
    out = plot_skip_vs_log_base_by_noise(curves, records)
    print(f"Wrote {out}")
