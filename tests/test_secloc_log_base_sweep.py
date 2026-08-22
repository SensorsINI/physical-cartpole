"""Sweep Secloc log_base vs skipped-update rate; plot curves for several noise levels.

Simulates PID @ 5 ms with measurement noise, records noisy states, replays them
through SeclocGate. Noise scale is increased until PID can no longer hold the pole.

Run table tests:
  pytest tests/test_secloc_log_base_sweep.py -v -s

Generate plot (saved under tests/output/):
  pytest tests/test_secloc_log_base_sweep.py -v -s -k plot
  python tests/test_secloc_log_base_sweep.py
  python tests/test_secloc_log_base_sweep.py --quant
  python tests/test_secloc_log_base_sweep.py --plot-only      # replot float cache
  python tests/test_secloc_log_base_sweep.py --quant --plot-only
  python tests/test_secloc_log_base_sweep.py --recompute      # ignore cache

Each run writes two PNGs: *_with_no_noise.png and *_no_no_noise.png.
"""
from __future__ import annotations

import contextlib
import json
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterator, TypeVar

import numpy as np
import pytest
from tqdm import tqdm

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
QUANT_NOISE_SCALE_CANDIDATES = (0.0,) + NOISE_SCALE_CANDIDATES
FLOAT_NOISE_SCALE_CANDIDATES = QUANT_NOISE_SCALE_CANDIDATES

OUTPUT_DIR = REPO_ROOT / "tests" / "output"
CACHE_DIR = OUTPUT_DIR / "secloc_sweep_cache"
PLOT_FLOAT_WITH_NO_NOISE = (
    OUTPUT_DIR / "secloc_log_base_vs_skip_pct_noise_sweep_with_no_noise.png"
)
PLOT_FLOAT_NO_NO_NOISE = (
    OUTPUT_DIR / "secloc_log_base_vs_skip_pct_noise_sweep_no_no_noise.png"
)
PLOT_QUANT_WITH_NO_NOISE = (
    OUTPUT_DIR / "secloc_log_base_vs_skip_pct_quantized_with_no_noise.png"
)
PLOT_QUANT_NO_NO_NOISE = (
    OUTPUT_DIR / "secloc_log_base_vs_skip_pct_quantized_no_no_noise.png"
)
PLOT_FLOAT_SKIP_CHANGED_WITH_NO_NOISE = (
    OUTPUT_DIR / "secloc_log_base_vs_skip_given_changed_float_with_no_noise.png"
)
PLOT_FLOAT_SKIP_CHANGED_NO_NO_NOISE = (
    OUTPUT_DIR / "secloc_log_base_vs_skip_given_changed_float_no_no_noise.png"
)
PLOT_QUANT_SKIP_CHANGED_WITH_NO_NOISE = (
    OUTPUT_DIR / "secloc_log_base_vs_skip_given_changed_quantized_with_no_noise.png"
)
PLOT_QUANT_SKIP_CHANGED_NO_NO_NOISE = (
    OUTPUT_DIR / "secloc_log_base_vs_skip_given_changed_quantized_no_no_noise.png"
)
PLOT_COUNTERFactual_QUANT_WITH_NO_NOISE = (
    OUTPUT_DIR
    / "secloc_log_base_vs_skip_given_changed_counterfactual_quant_with_no_noise.png"
)
PLOT_COUNTERFactual_QUANT_NO_NO_NOISE = (
    OUTPUT_DIR
    / "secloc_log_base_vs_skip_given_changed_counterfactual_quant_no_no_noise.png"
)
# Backward-compatible aliases
PLOT_PATH = PLOT_FLOAT_WITH_NO_NOISE
PLOT_QUANT_PATH = PLOT_QUANT_WITH_NO_NOISE
SWEEP_CACHE_VERSION = 5
REFERENCE_CLOSED_LOOP_NOISE = 1.0
GATE_REPLAY_SWEEP_UPPER = 1.6

T = TypeVar("T")

# Finer grid near log_base=1.0; upper end set from closed-loop failure search.
LOG_BASE_SWEEP_FINE = np.round(np.arange(1.000, 1.011, 0.001), 3)
LOG_BASE_SWEEP_DEFAULT_UPPER = 1.15
CLOSED_LOOP_LOG_BASE_STEP = 0.01
CLOSED_LOOP_LOG_BASE_MAX = 5.0


def build_log_base_sweep_to(upper: float) -> np.ndarray:
    upper = float(min(max(upper, LOG_BASE_SWEEP_FINE[-1]), CLOSED_LOOP_LOG_BASE_MAX))
    parts = [LOG_BASE_SWEEP_FINE]
    if upper > 1.012:
        mid_end = min(upper, 1.25)
        parts.append(np.arange(1.012, mid_end + 0.004, 0.005))
    if upper > 1.26:
        parts.append(np.arange(1.26, min(upper, 2.0) + 0.04, 0.02))
    if upper > 2.01:
        parts.append(np.arange(2.05, upper + 0.09, 0.1))
    return np.unique(np.round(np.concatenate(parts), 3))


LOG_BASE_SWEEP_PLOT = build_log_base_sweep_to(LOG_BASE_SWEEP_DEFAULT_UPPER)


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
    plateau_pct: float | None = None


@dataclass(frozen=True)
class GateSweepResult:
    log_base: float
    skipped_pct: float
    updates: int
    total: int
    skip_given_changed_pct: float = 0.0
    skip_given_flat_pct: float = 0.0


@dataclass(frozen=True)
class GateSkipBreakdown:
    """Decompose Secloc skips by whether gate inputs changed since the previous poll."""

    log_base: float
    total: int
    skipped_pct: float
    updates: int
    flat_polls: int
    changed_polls: int
    skip_on_flat: int
    skip_on_changed: int
    update_on_flat: int
    update_on_changed: int

    @property
    def flat_poll_pct(self) -> float:
        return 100.0 * self.flat_polls / self.total if self.total else 0.0

    @property
    def skip_given_flat_pct(self) -> float:
        return 100.0 * self.skip_on_flat / self.flat_polls if self.flat_polls else 0.0

    @property
    def skip_given_changed_pct(self) -> float:
        return (
            100.0 * self.skip_on_changed / self.changed_polls if self.changed_polls else 0.0
        )

    @property
    def quant_plateau_skip_share_pct(self) -> float:
        """Share of all skips that happened while gate inputs were flat (quant artifact)."""
        skipped = self.skip_on_flat + self.skip_on_changed
        return 100.0 * self.skip_on_flat / skipped if skipped else 0.0


def gate_input_shifts(sample: TrajectorySample) -> tuple[float, float]:
    """Scalars Secloc compares across polls (|θ| and |x - target|)."""
    angle_shift = abs(float(sample.s[ANGLE_IDX]))
    position_shift = abs(float(sample.s[POSITION_IDX]) - sample.target_position)
    return angle_shift, position_shift


def replay_gate_breakdown(
    trajectory: list[TrajectorySample],
    *,
    log_base: float,
    ref_period: float = DEFAULT_REF_PERIOD,
    dead_ang: float = DEFAULT_DEAD_ANG,
    dead_pos: float = DEFAULT_DEAD_POS,
) -> GateSkipBreakdown:
    # The gate throttle is native in control loop ticks; convert the seconds
    # parameter kept for the callers' convenience.
    ref_period_ticks = (
        max(1, round(float(ref_period) / POLL_DT_S)) if float(ref_period) > 0 else 0
    )
    gate = SeclocGate(
        log_base=float(log_base),
        ref_period_ticks=ref_period_ticks,
        dead_ang=float(dead_ang),
        dead_pos=float(dead_pos),
    )
    gate.set_time_quantum(POLL_DT_S)
    flat_polls = 0
    changed_polls = 0
    skip_on_flat = 0
    skip_on_changed = 0
    update_on_flat = 0
    update_on_changed = 0
    prev_angle_shift: float | None = None
    prev_position_shift: float | None = None

    for sample in trajectory:
        angle_shift, position_shift = gate_input_shifts(sample)
        input_changed = False
        if prev_angle_shift is not None:
            input_changed = (
                angle_shift != prev_angle_shift or position_shift != prev_position_shift
            )

        did_update = gate.should_sample(
            sample.s,
            sample.target_position,
            time=sample.time,
        )

        if prev_angle_shift is not None:
            if input_changed:
                changed_polls += 1
                if did_update:
                    update_on_changed += 1
                else:
                    skip_on_changed += 1
            else:
                flat_polls += 1
                if did_update:
                    update_on_flat += 1
                else:
                    skip_on_flat += 1

        prev_angle_shift = angle_shift
        prev_position_shift = position_shift

    return GateSkipBreakdown(
        log_base=float(log_base),
        total=gate.total_decisions,
        skipped_pct=gate.skipped_update_percentage,
        updates=gate.update_decisions,
        flat_polls=flat_polls,
        changed_polls=changed_polls,
        skip_on_flat=skip_on_flat,
        skip_on_changed=skip_on_changed,
        update_on_flat=update_on_flat,
        update_on_changed=update_on_changed,
    )


def format_gate_breakdown_table(rows: list[GateSkipBreakdown]) -> str:
    lines = [
        "log_base  skip%  skip|flat  skip|chg  flat%  plateau-skip%",
        "--------  -----  --------  -------  -----  -------------",
    ]
    for row in rows:
        lines.append(
            f"{row.log_base:8.3f}  {row.skipped_pct:5.1f}  "
            f"{row.skip_given_flat_pct:8.1f}  {row.skip_given_changed_pct:7.1f}  "
            f"{row.flat_poll_pct:5.1f}  {row.quant_plateau_skip_share_pct:13.1f}"
        )
    return "\n".join(lines)


def print_quant_vs_float_breakdown(
    *,
    log_base: float = 1.05,
    noise_scale: float = 1.0,
    seed: int = 123,
) -> None:
    quant = record_stabilized_trajectory(
        noise_scale=noise_scale, seed=seed, quantize_sensors=True
    )
    float_traj = record_stabilized_trajectory(
        noise_scale=noise_scale, seed=seed, quantize_sensors=False
    )
    quant_row = replay_gate_breakdown(quant.samples, log_base=log_base)
    float_row = replay_gate_breakdown(float_traj.samples, log_base=log_base)
    counter_samples = quantize_trajectory_samples(float_traj.samples)
    counter_row = replay_gate_breakdown(counter_samples, log_base=log_base)
    print(
        f"\nSecloc skip decomposition @ log_base={log_base:g}, σ×{noise_scale:g}, seed={seed}\n"
        f"skip|changed = P(skip | |θ| or |x-x*| changed since last poll)\n"
        f"skip|flat   = P(skip | both unchanged — often quant artifact)\n"
    )
    print("Quantized recordings (hardware-like):")
    print(format_gate_breakdown_table([quant_row]))
    print("\nFloat recordings (no ADC quant):")
    print(format_gate_breakdown_table([float_row]))
    print("\nCounterfactual (float trajectory, quant only at gate):")
    print(format_gate_breakdown_table([counter_row]))

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


def plateau_angle_fraction(trajectory: list[TrajectorySample]) -> float:
    """Share of consecutive polls where quantized |θ| did not change."""
    magnitudes = [abs(float(sample.s[ANGLE_IDX])) for sample in trajectory]
    if len(magnitudes) < 2:
        return 0.0
    same = sum(1 for i in range(1, len(magnitudes)) if magnitudes[i] == magnitudes[i - 1])
    return 100.0 * same / (len(magnitudes) - 1)


def record_plateau_pct(record: TrajectoryRecord) -> float:
    if record.plateau_pct is not None:
        return record.plateau_pct
    return plateau_angle_fraction(record.samples)


def _maybe_progress(
    iterable: Iterator[T] | list[T],
    *,
    enabled: bool,
    total: int | None = None,
    desc: str,
) -> Iterator[T]:
    if not enabled:
        return iter(iterable)
    return tqdm(iterable, total=total, desc=desc, unit="step")


@dataclass(frozen=True)
class ClosedLoopSeclocResult:
    log_base: float
    stable: bool
    skip_pct: float
    max_angle_rad: float
    max_position_m: float


def run_closed_loop_quant_secloc(
    log_base: float,
    *,
    noise_scale: float = 0.0,
    seed: int = 123,
    warmup_s: float = WARMUP_S,
    record_s: float = RECORD_S,
    initial_angle_rad: float = 0.12,
    quantize_sensors: bool = True,
) -> ClosedLoopSeclocResult:
    """Run PID+Secloc in closed loop at a fixed log_base."""
    sigmas = sigmas_for_scale(noise_scale)
    with measurement_noise(sigmas):
        cp = CartPole()
        cp.dt_simulation = DT_SIM_S
        cp.dt_controller = POLL_DT_S
        cp.set_controller("pid", use_secloc=True)
        cp.controller.secloc.log_base = float(log_base)
        cp.SensorQuantizerInstance.enabled = quantize_sensors
        cp.print_controller_status_if_available = lambda: None

        cp.s = create_cartpole_state()
        cp.s[ANGLE_IDX] = initial_angle_rad
        cp.s[ANGLE_COS_IDX] = np.cos(initial_angle_rad)
        cp.s[ANGLE_SIN_IDX] = np.sin(initial_angle_rad)
        cp.target_position = 0.0
        cp.time = 0.0
        cp.NoiseAdderInstance.noise_mode = "OFF" if noise_scale == 0 else "gaussian"
        cp.NoiseAdderInstance.rng_noise_adder = np.random.default_rng(seed)

        steps_per_poll = _steps_per_poll()
        warmup_polls = int(warmup_s / POLL_DT_S)
        record_polls = int(record_s / POLL_DT_S)
        max_angle = 0.0
        max_position = 0.0

        for poll_idx in range(warmup_polls + record_polls):
            for _ in range(steps_per_poll):
                cp.update_state()
            if poll_idx >= warmup_polls:
                max_angle = max(max_angle, abs(float(cp.s[ANGLE_IDX])))
                max_position = max(max_position, abs(float(cp.s[POSITION_IDX])))

    stable = (
        max_angle <= MAX_STABLE_ANGLE_RAD
        and max_position <= MAX_STABLE_POSITION_M
    )
    return ClosedLoopSeclocResult(
        log_base=float(log_base),
        stable=stable,
        skip_pct=float(cp.controller.secloc.skipped_update_percentage),
        max_angle_rad=max_angle,
        max_position_m=max_position,
    )


def find_first_failing_log_base(
    noise_scale: float,
    *,
    seed: int = 123,
    quantize_sensors: bool = True,
    log_base_min: float = 1.0,
    log_base_max: float = CLOSED_LOOP_LOG_BASE_MAX,
) -> float | None:
    """First log_base where closed-loop PID+Secloc loses stability, or None if stable through max."""
    if not run_closed_loop_quant_secloc(
        log_base_min,
        noise_scale=noise_scale,
        seed=seed,
        quantize_sensors=quantize_sensors,
    ).stable:
        return float(log_base_min)

    lo = float(log_base_min)
    hi = lo
    while hi < log_base_max - 1e-9:
        step = max(0.03, hi * 0.08)
        candidate = min(log_base_max, round(hi + step, 3))
        if candidate <= hi + 1e-9:
            break
        if run_closed_loop_quant_secloc(
            candidate,
            noise_scale=noise_scale,
            seed=seed,
            quantize_sensors=quantize_sensors,
        ).stable:
            lo = candidate
            hi = candidate
            continue
        hi = candidate
        break
    else:
        return None

    if hi <= lo + 0.009:
        return float(hi)

    while hi - lo > 0.009:
        mid = round((lo + hi) / 2, 3)
        if run_closed_loop_quant_secloc(
            mid,
            noise_scale=noise_scale,
            seed=seed,
            quantize_sensors=quantize_sensors,
        ).stable:
            lo = mid
        else:
            hi = mid
    return float(hi)


def failure_log_base_limits(
    records: list[TrajectoryRecord],
    *,
    seed: int = 123,
    quantize_sensors: bool = True,
    show_progress: bool = False,
) -> dict[float, float | None]:
    limits: dict[float, float | None] = {}
    stable_records = [record for record in records if record.stable]
    for record in _maybe_progress(
        stable_records,
        enabled=show_progress,
        total=len(stable_records),
        desc="Failure search",
    ):
        limits[record.noise_scale] = find_first_failing_log_base(
            record.noise_scale,
            seed=seed,
            quantize_sensors=quantize_sensors,
        )
    return limits


def sweep_upper_from_failure_limits(
    failure_limits: dict[float, float | None],
    *,
    no_failure_upper: float = 2.0,
) -> float:
    upper = LOG_BASE_SWEEP_DEFAULT_UPPER
    for limit in failure_limits.values():
        if limit is not None:
            upper = max(upper, limit)
        else:
            upper = max(upper, no_failure_upper)
    return float(min(upper, CLOSED_LOOP_LOG_BASE_MAX))


def record_stabilized_trajectory(
    *,
    noise_scale: float = 1.0,
    seed: int = 123,
    warmup_s: float = WARMUP_S,
    record_s: float = RECORD_S,
    initial_angle_rad: float = 0.12,
    quantize_sensors: bool = False,
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
        cp.SensorQuantizerInstance.enabled = quantize_sensors

        cp.s = create_cartpole_state()
        cp.s[ANGLE_IDX] = initial_angle_rad
        cp.s[ANGLE_COS_IDX] = np.cos(initial_angle_rad)
        cp.s[ANGLE_SIN_IDX] = np.sin(initial_angle_rad)
        cp.target_position = 0.0
        cp.time = 0.0
        cp.NoiseAdderInstance.noise_mode = "OFF" if noise_scale == 0 else "gaussian"
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
    plateau = plateau_angle_fraction(samples) if quantize_sensors else None
    return TrajectoryRecord(
        samples=samples,
        noise_scale=noise_scale,
        sigma_angle=sigmas["sigma_angle"],
        stable=stable,
        max_angle_rad=max_angle,
        max_position_m=max_position,
        plateau_pct=plateau,
    )


def quantize_trajectory_samples(
    samples: list[TrajectorySample],
) -> list[TrajectorySample]:
    """Counterfactual: float PID trajectory, ADC/encoder applied only at gate input."""
    from CartPole.sensor_quantizer import SensorQuantizer

    quantizer = SensorQuantizer({"enabled": True, "source": "physical"})
    out: list[TrajectorySample] = []
    for sample in samples:
        out.append(
            TrajectorySample(
                time=sample.time,
                s=quantizer.quantize_measurement(sample.s, copy=True),
                target_position=sample.target_position,
            )
        )
    return out


def measure_skip_rate(
    trajectory: list[TrajectorySample],
    *,
    log_base: float,
    ref_period: float = DEFAULT_REF_PERIOD,
    dead_ang: float = DEFAULT_DEAD_ANG,
    dead_pos: float = DEFAULT_DEAD_POS,
) -> GateSweepResult:
    breakdown = replay_gate_breakdown(
        trajectory,
        log_base=log_base,
        ref_period=ref_period,
        dead_ang=dead_ang,
        dead_pos=dead_pos,
    )
    return GateSweepResult(
        log_base=breakdown.log_base,
        skipped_pct=breakdown.skipped_pct,
        updates=breakdown.updates,
        total=breakdown.total,
        skip_given_changed_pct=breakdown.skip_given_changed_pct,
        skip_given_flat_pct=breakdown.skip_given_flat_pct,
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
    quantize_sensors: bool = False,
    show_progress: bool = False,
) -> list[TrajectoryRecord]:
    """Increase noise scale; stop after first level where PID loses stability."""
    records: list[TrajectoryRecord] = []
    for scale in _maybe_progress(
        scales,
        enabled=show_progress,
        total=len(scales),
        desc="PID trajectories",
    ):
        record = record_stabilized_trajectory(
            noise_scale=scale,
            seed=seed,
            quantize_sensors=quantize_sensors,
        )
        records.append(record)
        if not record.stable:
            break
    return records


def sweep_skip_curves_by_noise(
    records: list[TrajectoryRecord],
    log_bases: np.ndarray | list[float] | None = None,
    *,
    show_progress: bool = False,
    quantize_samples: bool = False,
) -> dict[float, list[GateSweepResult]]:
    if log_bases is None:
        log_bases = LOG_BASE_SWEEP
    log_bases = list(log_bases)
    stable_records = [record for record in records if record.stable]
    total = len(stable_records) * len(log_bases)
    curves: dict[float, list[GateSweepResult]] = {}
    progress = tqdm(total=total, desc="Gate replay", unit="pt", disable=not show_progress)
    try:
        for record in stable_records:
            samples = record.samples
            if quantize_samples:
                samples = quantize_trajectory_samples(record.samples)
            rows: list[GateSweepResult] = []
            for log_base in log_bases:
                rows.append(measure_skip_rate(samples, log_base=log_base))
                progress.update(1)
            curves[record.noise_scale] = rows
    finally:
        progress.close()
    return curves


def format_sweep_table(results: list[GateSweepResult]) -> str:
    lines = [
        "log_base  skipped%  skip|chg%  updates   total",
        "--------  ---------  --------  -------  ------",
    ]
    for row in results:
        lines.append(
            f"{row.log_base:8.3f}  {row.skipped_pct:8.1f}  "
            f"{row.skip_given_changed_pct:8.1f}  "
            f"{row.updates:7d}  {row.total:6d}"
        )
    return "\n".join(lines)


def closest_log_base_to_target_skip(
    results: list[GateSweepResult],
    target_skip_pct: float,
) -> GateSweepResult:
    return min(results, key=lambda row: abs(row.skipped_pct - target_skip_pct))


@dataclass(frozen=True)
class SweepDataset:
    kind: str
    seed: int
    quantize_sensors: bool
    records: list[TrajectoryRecord]
    curves: dict[float, list[GateSweepResult]]
    failure_limits: dict[float, float | None]
    sweep_upper: float | None
    log_bases: list[float]
    reference_closed_loop_failure: float | None = None
    counterfactual_quant_curves: dict[float, list[GateSweepResult]] | None = None


def sweep_cache_path(*, quantize_sensors: bool) -> Path:
    name = "quantized" if quantize_sensors else "float"
    return CACHE_DIR / f"secloc_log_base_sweep_{name}_v{SWEEP_CACHE_VERSION}.json"


def sweep_cache_config(*, quantize_sensors: bool) -> dict[str, Any]:
    return {
        "version": SWEEP_CACHE_VERSION,
        "quantize_sensors": quantize_sensors,
        "poll_dt_s": POLL_DT_S,
        "record_s": RECORD_S,
        "warmup_s": WARMUP_S,
        "base_sigma_angle": BASE_SIGMA_ANGLE,
        "log_base_default_upper": LOG_BASE_SWEEP_DEFAULT_UPPER,
        "gate_replay_sweep_upper": GATE_REPLAY_SWEEP_UPPER,
    }


def _gate_row_to_dict(row: GateSweepResult) -> dict[str, Any]:
    return asdict(row)


def _gate_row_from_dict(data: dict[str, Any]) -> GateSweepResult:
    return GateSweepResult(
        log_base=float(data["log_base"]),
        skipped_pct=float(data["skipped_pct"]),
        updates=int(data["updates"]),
        total=int(data["total"]),
        skip_given_changed_pct=float(data.get("skip_given_changed_pct", 0.0)),
        skip_given_flat_pct=float(data.get("skip_given_flat_pct", 0.0)),
    )


def _curves_to_dict(
    curves: dict[float, list[GateSweepResult]],
) -> dict[str, list[dict[str, Any]]]:
    return {
        str(noise_scale): [_gate_row_to_dict(row) for row in rows]
        for noise_scale, rows in curves.items()
    }


def _curves_from_dict(payload: dict[str, list[dict[str, Any]]]) -> dict[float, list[GateSweepResult]]:
    return {
        float(noise_scale): [_gate_row_from_dict(row) for row in rows]
        for noise_scale, rows in payload.items()
    }


def _record_to_dict(record: TrajectoryRecord) -> dict[str, Any]:
    return {
        "noise_scale": record.noise_scale,
        "sigma_angle": record.sigma_angle,
        "stable": record.stable,
        "max_angle_rad": record.max_angle_rad,
        "max_position_m": record.max_position_m,
        "plateau_pct": record_plateau_pct(record),
    }


def _record_from_dict(data: dict[str, Any]) -> TrajectoryRecord:
    return TrajectoryRecord(
        samples=[],
        noise_scale=float(data["noise_scale"]),
        sigma_angle=float(data["sigma_angle"]),
        stable=bool(data["stable"]),
        max_angle_rad=float(data["max_angle_rad"]),
        max_position_m=float(data["max_position_m"]),
        plateau_pct=(
            None if data.get("plateau_pct") is None else float(data["plateau_pct"])
        ),
    )


def save_sweep_dataset(dataset: SweepDataset) -> Path:
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    path = sweep_cache_path(quantize_sensors=dataset.quantize_sensors)
    payload = {
        **sweep_cache_config(quantize_sensors=dataset.quantize_sensors),
        "kind": dataset.kind,
        "seed": dataset.seed,
        "sweep_upper": dataset.sweep_upper,
        "log_bases": [float(x) for x in dataset.log_bases],
        "records": [_record_to_dict(record) for record in dataset.records],
        "failure_limits": {
            str(key): value for key, value in dataset.failure_limits.items()
        },
        "reference_closed_loop_failure": dataset.reference_closed_loop_failure,
        "curves": _curves_to_dict(dataset.curves),
    }
    if dataset.counterfactual_quant_curves is not None:
        payload["counterfactual_quant_curves"] = _curves_to_dict(
            dataset.counterfactual_quant_curves
        )
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path


def load_sweep_dataset(*, quantize_sensors: bool, seed: int = 123) -> SweepDataset | None:
    path = sweep_cache_path(quantize_sensors=quantize_sensors)
    if not path.is_file():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    expected = sweep_cache_config(quantize_sensors=quantize_sensors)
    for key, value in expected.items():
        if payload.get(key) != value:
            return None
    if payload.get("seed") != seed:
        return None

    records = [_record_from_dict(row) for row in payload["records"]]
    curves = _curves_from_dict(payload["curves"])
    counterfactual = payload.get("counterfactual_quant_curves")
    counterfactual_quant_curves = (
        None if counterfactual is None else _curves_from_dict(counterfactual)
    )
    failure_limits = {
        float(key): (None if value is None else float(value))
        for key, value in payload.get("failure_limits", {}).items()
    }
    reference_closed_loop_failure = payload.get("reference_closed_loop_failure")
    if reference_closed_loop_failure is not None:
        reference_closed_loop_failure = float(reference_closed_loop_failure)
    return SweepDataset(
        kind=str(payload["kind"]),
        seed=int(payload["seed"]),
        quantize_sensors=quantize_sensors,
        records=records,
        curves=curves,
        failure_limits=failure_limits,
        sweep_upper=(
            None if payload.get("sweep_upper") is None else float(payload["sweep_upper"])
        ),
        log_bases=[float(x) for x in payload["log_bases"]],
        reference_closed_loop_failure=reference_closed_loop_failure,
        counterfactual_quant_curves=counterfactual_quant_curves,
    )


def compute_float_sweep_dataset(
    *,
    seed: int = 123,
    show_progress: bool = True,
) -> SweepDataset:
    records = explore_noise_scales_until_break(
        seed=seed,
        scales=FLOAT_NOISE_SCALE_CANDIDATES,
        quantize_sensors=False,
        show_progress=show_progress,
    )
    log_bases = [float(x) for x in build_log_base_sweep_to(GATE_REPLAY_SWEEP_UPPER)]
    curves = sweep_skip_curves_by_noise(
        records,
        log_bases,
        show_progress=show_progress,
    )
    counterfactual_quant_curves = sweep_skip_curves_by_noise(
        records,
        log_bases,
        show_progress=show_progress,
        quantize_samples=True,
    )
    return SweepDataset(
        kind="float",
        seed=seed,
        quantize_sensors=False,
        records=records,
        curves=curves,
        failure_limits={},
        sweep_upper=GATE_REPLAY_SWEEP_UPPER,
        log_bases=log_bases,
        counterfactual_quant_curves=counterfactual_quant_curves,
    )


def compute_quantized_sweep_dataset(
    *,
    seed: int = 123,
    show_progress: bool = True,
) -> SweepDataset:
    records = explore_noise_scales_until_break(
        seed=seed,
        scales=QUANT_NOISE_SCALE_CANDIDATES,
        quantize_sensors=True,
        show_progress=show_progress,
    )
    # Gate-replay skip curves use a fixed log_base range. Per-noise closed-loop
    # failure limits were misleading (same seed, different noise realizations).
    sweep_upper = GATE_REPLAY_SWEEP_UPPER
    log_bases = [float(x) for x in build_log_base_sweep_to(sweep_upper)]
    curves = sweep_skip_curves_by_noise(
        records,
        log_bases,
        show_progress=show_progress,
    )
    reference_failure = find_first_failing_log_base(
        REFERENCE_CLOSED_LOOP_NOISE,
        seed=seed,
        quantize_sensors=True,
    )
    if show_progress and reference_failure is not None:
        print(
            f"Reference closed-loop failure at σ×{REFERENCE_CLOSED_LOOP_NOISE:g}: "
            f"log_base={reference_failure:.2f}"
        )
    return SweepDataset(
        kind="quantized",
        seed=seed,
        quantize_sensors=True,
        records=records,
        curves=curves,
        failure_limits={},
        sweep_upper=sweep_upper,
        log_bases=log_bases,
        reference_closed_loop_failure=reference_failure,
    )


def get_sweep_dataset(
    *,
    quantize_sensors: bool,
    seed: int = 123,
    plot_only: bool = False,
    recompute: bool = False,
    show_progress: bool = True,
) -> SweepDataset:
    if not recompute:
        cached = load_sweep_dataset(quantize_sensors=quantize_sensors, seed=seed)
        if cached is not None:
            if show_progress:
                print(f"Loaded cached sweep data from {sweep_cache_path(quantize_sensors=quantize_sensors)}")
            return cached
        if plot_only:
            path = sweep_cache_path(quantize_sensors=quantize_sensors)
            raise FileNotFoundError(
                f"No cached sweep data at {path}. Run without --plot-only first."
            )

    if quantize_sensors:
        dataset = compute_quantized_sweep_dataset(seed=seed, show_progress=show_progress)
    else:
        dataset = compute_float_sweep_dataset(seed=seed, show_progress=show_progress)

    cache_path = save_sweep_dataset(dataset)
    if show_progress:
        print(f"Saved sweep data to {cache_path}")
    return dataset


def plot_noise_scales(
    curves: dict[float, list[GateSweepResult]],
    *,
    include_no_noise: bool = True,
) -> list[float]:
    """Every second curve to spread matplotlib default colors."""
    scales = sorted(curves)
    if not include_no_noise:
        scales = [scale for scale in scales if scale != 0.0]
    return scales[::2]


def _curve_metric(row: GateSweepResult, metric: str) -> float:
    return float(getattr(row, metric))


def plot_skip_vs_log_base_by_noise(
    curves: dict[float, list[GateSweepResult]],
    records: list[TrajectoryRecord],
    *,
    include_no_noise: bool = True,
    sweep_upper: float | None = None,
    y_metric: str = "skipped_pct",
    output_path: Path = PLOT_FLOAT_WITH_NO_NOISE,
) -> Path:
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if sweep_upper is None:
        sweep_upper = GATE_REPLAY_SWEEP_UPPER

    fig = plt.figure(figsize=(10, 5.5))
    gs = GridSpec(
        2,
        2,
        figure=fig,
        width_ratios=[1.55, 1],
        height_ratios=[3.2, 1],
        wspace=0.06,
        hspace=0.12,
        left=0.08,
        right=0.98,
        bottom=0.10,
        top=0.88,
    )
    ax = fig.add_subplot(gs[:, 0])
    ax_leg = fig.add_subplot(gs[0, 1])
    ax_note = fig.add_subplot(gs[1, 1])
    ax_leg.axis("off")
    ax_note.axis("off")

    for noise_scale in plot_noise_scales(curves, include_no_noise=include_no_noise):
        rows = curves[noise_scale]
        rec = next(r for r in records if r.noise_scale == noise_scale)
        if noise_scale == 0:
            label = f"no noise (max|θ|={rec.max_angle_rad:.2f} rad)"
        else:
            label = (
                f"σ×{noise_scale:g} "
                f"(σ_θ={rec.sigma_angle*1e3:.2f} mrad, "
                f"max|θ|={rec.max_angle_rad:.2f} rad)"
            )
        ax.plot(
            [r.log_base for r in rows],
            [_curve_metric(r, y_metric) for r in rows],
            marker="o",
            markersize=3,
            linewidth=1.5,
            label=label,
        )

    broken = [r for r in records if not r.stable]
    if y_metric == "skip_given_changed_pct":
        notes = [
            "skip|changed = P(skip | gate input changed",
            "since previous poll). Isolates Secloc vs dynamics.",
            "Float (non-quantized) PID trajectories.",
        ]
    else:
        notes = [
            "Float (non-quantized) noisy measurements.",
            "Gate replay on recorded PID trajectories.",
        ]
    if broken:
        r = broken[0]
        notes.append(
            f"PID lost control at σ×{r.noise_scale:g}\n"
            f"(max|θ|={r.max_angle_rad:.2f} rad)"
        )

    ax.set_xlabel("Secloc log_base")
    if y_metric == "skip_given_changed_pct":
        ax.set_ylabel("Skipped updates when input changed (%)")
        title_metric = "skip|changed"
    else:
        ax.set_ylabel("Skipped controller updates (%)")
        title_metric = "skip rate"
    ax.set_title(
        f"Secloc {title_metric} vs log_base (float sensors)\n"
        f"PID simulation, no ADC/encoder quant, {POLL_DT_S*1e3:.0f} ms polls, "
        f"{RECORD_S:.0f} s record window"
    )
    ax.grid(True, alpha=0.3)
    ax.set_xlim(LOG_BASE_SWEEP_FINE[0], sweep_upper)
    ax.set_ylim(0, 100)

    handles, labels = ax.get_legend_handles_labels()
    ax_leg.legend(handles, labels, loc="upper left", fontsize=7, framealpha=0.95)
    if notes:
        ax_note.text(
            0.0,
            1.0,
            "\n".join(notes),
            transform=ax_note.transAxes,
            fontsize=8,
            va="top",
            ha="left",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85, pad=0.35),
        )
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_skip_vs_log_base_quantized(
    curves: dict[float, list[GateSweepResult]],
    records: list[TrajectoryRecord],
    *,
    failure_limits: dict[float, float] | None = None,
    sweep_upper: float | None = None,
    reference_closed_loop_failure: float | None = None,
    include_no_noise: bool = True,
    output_path: Path = PLOT_QUANT_WITH_NO_NOISE,
    y_metric: str = "skipped_pct",
) -> Path:
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if sweep_upper is None:
        sweep_upper = GATE_REPLAY_SWEEP_UPPER

    fig = plt.figure(figsize=(11, 5.5))
    gs = GridSpec(
        2,
        2,
        figure=fig,
        width_ratios=[2.15, 0.78],
        height_ratios=[3.4, 1.1],
        wspace=0.05,
        hspace=0.10,
        left=0.07,
        right=0.98,
        bottom=0.10,
        top=0.88,
    )
    ax = fig.add_subplot(gs[:, 0])
    ax_leg = fig.add_subplot(gs[0, 1])
    ax_note = fig.add_subplot(gs[1, 1])
    ax_leg.axis("off")
    ax_note.axis("off")
    for noise_scale in plot_noise_scales(curves, include_no_noise=include_no_noise):
        rows = curves[noise_scale]
        if not rows:
            continue

        rec = next(r for r in records if r.noise_scale == noise_scale)
        flat_theta = record_plateau_pct(rec)
        if noise_scale == 0:
            label = (
                f"no noise "
                f"(flat |θ| {flat_theta:.0f}%, "
                f"max|θ|={rec.max_angle_rad:.2f} rad)"
            )
        else:
            label = (
                f"σ×{noise_scale:g} "
                f"(σ_θ={rec.sigma_angle*1e3:.2f} mrad, "
                f"flat |θ| {flat_theta:.0f}%, "
                f"max|θ|={rec.max_angle_rad:.2f} rad)"
            )
        ax.plot(
            [r.log_base for r in rows],
            [_curve_metric(r, y_metric) for r in rows],
            marker="o",
            markersize=3,
            linewidth=1.5,
            label=label,
        )

    broken = [r for r in records if not r.stable]
    if y_metric == "skip_given_changed_pct":
        notes = [
            "skip|changed = P(skip | gate input changed",
            "since previous poll). Isolates Secloc vs dynamics.",
            "Skip % = gate replay on open-loop",
            "PID trajectories (one recording per σ)",
            "flat |θ| = % polls with unchanged |θ|",
            "(quant steps)",
        ]
    else:
        notes = [
            "Skip % = gate replay on",
            "open-loop PID trajectories",
            "(one recording per σ)",
            "flat |θ| = % polls with",
            "unchanged |θ| (quant steps)",
        ]
    if reference_closed_loop_failure is not None:
        notes.extend(
            [
                f"σ×{REFERENCE_CLOSED_LOOP_NOISE:g} closed-loop",
                f"fails @ log_base={reference_closed_loop_failure:.2f}",
            ]
        )
    if broken:
        r = broken[0]
        notes.extend(
            [
                f"PID lost control @ σ×{r.noise_scale:g}",
                f"(max|θ|={r.max_angle_rad:.2f} rad,",
                "no secloc sweep)",
            ]
        )
    ax.set_xlabel("Secloc log_base")
    if y_metric == "skip_given_changed_pct":
        ax.set_ylabel("Skipped updates when input changed (%)")
        title_metric = "skip|changed"
    else:
        ax.set_ylabel("Skipped controller updates (%)")
        title_metric = "skip rate"
    ax.set_title(
        f"Secloc {title_metric} vs log_base (quantized sensors)\n"
        f"PID simulation, ADC/encoder quant, {POLL_DT_S*1e3:.0f} ms polls, "
        f"{RECORD_S:.0f} s record window"
    )
    ax.grid(True, alpha=0.3)
    ax.set_xlim(LOG_BASE_SWEEP_FINE[0], sweep_upper)
    ax.set_ylim(0, 100)

    handles, labels = ax.get_legend_handles_labels()
    ax_leg.legend(
        handles,
        labels,
        loc="upper left",
        fontsize=6.5,
        framealpha=0.95,
        handletextpad=0.4,
    )
    if notes:
        ax_note.text(
            0.0,
            1.0,
            "\n".join(notes),
            transform=ax_note.transAxes,
            fontsize=7.5,
            va="top",
            ha="left",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85, pad=0.3),
        )
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_counterfactual_quant_skip_changed(
    curves: dict[float, list[GateSweepResult]],
    records: list[TrajectoryRecord],
    *,
    include_no_noise: bool = True,
    sweep_upper: float | None = None,
    output_path: Path = PLOT_COUNTERFactual_QUANT_WITH_NO_NOISE,
) -> Path:
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if sweep_upper is None:
        sweep_upper = GATE_REPLAY_SWEEP_UPPER

    fig = plt.figure(figsize=(11, 5.5))
    gs = GridSpec(
        2,
        2,
        figure=fig,
        width_ratios=[2.15, 0.78],
        height_ratios=[3.4, 1.1],
        wspace=0.05,
        hspace=0.10,
        left=0.07,
        right=0.98,
        bottom=0.10,
        top=0.88,
    )
    ax = fig.add_subplot(gs[:, 0])
    ax_leg = fig.add_subplot(gs[0, 1])
    ax_note = fig.add_subplot(gs[1, 1])
    ax_leg.axis("off")
    ax_note.axis("off")

    for noise_scale in plot_noise_scales(curves, include_no_noise=include_no_noise):
        rows = curves[noise_scale]
        if not rows:
            continue
        rec = next(r for r in records if r.noise_scale == noise_scale)
        label = (
            f"σ×{noise_scale:g} "
            f"(σ_θ={rec.sigma_angle*1e3:.2f} mrad, "
            f"max|θ|={rec.max_angle_rad:.2f} rad)"
        )
        ax.plot(
            [r.log_base for r in rows],
            [r.skip_given_changed_pct for r in rows],
            marker="o",
            markersize=3,
            linewidth=1.5,
            label=label,
        )

    notes = [
        "Counterfactual: float PID trajectory,",
        "ADC/encoder quant applied only at gate.",
        "skip|changed isolates Secloc filtering",
        "from quant plateaus.",
    ]
    ax.set_xlabel("Secloc log_base")
    ax.set_ylabel("Skipped updates when input changed (%)")
    ax.set_title(
        "Secloc skip|changed vs log_base (counterfactual quant)\n"
        f"Float trajectory + quant gate input, {POLL_DT_S*1e3:.0f} ms polls, "
        f"{RECORD_S:.0f} s record window"
    )
    ax.grid(True, alpha=0.3)
    ax.set_xlim(LOG_BASE_SWEEP_FINE[0], sweep_upper)
    ax.set_ylim(0, 100)
    handles, labels = ax.get_legend_handles_labels()
    ax_leg.legend(
        handles,
        labels,
        loc="upper left",
        fontsize=6.5,
        framealpha=0.95,
        handletextpad=0.4,
    )
    ax_note.text(
        0.0,
        1.0,
        "\n".join(notes),
        transform=ax_note.transAxes,
        fontsize=7.5,
        va="top",
        ha="left",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85, pad=0.3),
    )
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def generate_quantized_sensor_plot(
    *,
    seed: int = 123,
    plot_only: bool = False,
    recompute: bool = False,
    show_progress: bool = True,
) -> list[Path]:
    dataset = get_sweep_dataset(
        quantize_sensors=True,
        seed=seed,
        plot_only=plot_only,
        recompute=recompute,
        show_progress=show_progress,
    )
    paths: list[Path] = []
    plot_specs = (
        (True, PLOT_QUANT_WITH_NO_NOISE, "skipped_pct"),
        (False, PLOT_QUANT_NO_NO_NOISE, "skipped_pct"),
        (True, PLOT_QUANT_SKIP_CHANGED_WITH_NO_NOISE, "skip_given_changed_pct"),
        (False, PLOT_QUANT_SKIP_CHANGED_NO_NO_NOISE, "skip_given_changed_pct"),
    )
    for include_no_noise, output_path, y_metric in plot_specs:
        paths.append(
            plot_skip_vs_log_base_quantized(
                dataset.curves,
                dataset.records,
                sweep_upper=dataset.sweep_upper,
                reference_closed_loop_failure=dataset.reference_closed_loop_failure,
                include_no_noise=include_no_noise,
                y_metric=y_metric,
                output_path=output_path,
            )
        )
    return paths


def generate_float_sensor_plot(
    *,
    seed: int = 123,
    plot_only: bool = False,
    recompute: bool = False,
    show_progress: bool = True,
) -> list[Path]:
    dataset = get_sweep_dataset(
        quantize_sensors=False,
        seed=seed,
        plot_only=plot_only,
        recompute=recompute,
        show_progress=show_progress,
    )
    paths: list[Path] = []
    plot_specs = (
        (True, PLOT_FLOAT_WITH_NO_NOISE, "skipped_pct"),
        (False, PLOT_FLOAT_NO_NO_NOISE, "skipped_pct"),
        (True, PLOT_FLOAT_SKIP_CHANGED_WITH_NO_NOISE, "skip_given_changed_pct"),
        (False, PLOT_FLOAT_SKIP_CHANGED_NO_NO_NOISE, "skip_given_changed_pct"),
    )
    for include_no_noise, output_path, y_metric in plot_specs:
        paths.append(
            plot_skip_vs_log_base_by_noise(
                dataset.curves,
                dataset.records,
                include_no_noise=include_no_noise,
                sweep_upper=dataset.sweep_upper,
                y_metric=y_metric,
                output_path=output_path,
            )
        )
    if dataset.counterfactual_quant_curves is not None:
        for include_no_noise, output_path in (
            (True, PLOT_COUNTERFactual_QUANT_WITH_NO_NOISE),
            (False, PLOT_COUNTERFactual_QUANT_NO_NO_NOISE),
        ):
            paths.append(
                plot_counterfactual_quant_skip_changed(
                    dataset.counterfactual_quant_curves,
                    dataset.records,
                    include_no_noise=include_no_noise,
                    sweep_upper=dataset.sweep_upper,
                    output_path=output_path,
                )
            )
    return paths


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


def test_quantized_sensors_reproduce_log_base_cliff(capsys):
    """Integer ADC/encoder counts create a step at log_base=1.0 (hardware-like)."""
    float_traj = record_stabilized_trajectory(
        noise_scale=1.0, seed=123, quantize_sensors=False
    )
    quant_traj = record_stabilized_trajectory(
        noise_scale=1.0, seed=123, quantize_sensors=True
    )
    assert float_traj.stable and quant_traj.stable

    float_at_1 = measure_skip_rate(float_traj.samples, log_base=1.0)
    float_at_105 = measure_skip_rate(float_traj.samples, log_base=1.05)
    quant_at_1 = measure_skip_rate(quant_traj.samples, log_base=1.0)
    quant_at_1002 = measure_skip_rate(quant_traj.samples, log_base=1.002)
    quant_at_105 = measure_skip_rate(quant_traj.samples, log_base=1.05)

    plateau = plateau_angle_fraction(quant_traj.samples)
    print(
        f"\nFloat noise:  log_base 1.00 -> {float_at_1.skipped_pct:.1f}% skip, "
        f"1.05 -> {float_at_105.skipped_pct:.1f}%"
    )
    print(
        f"Quant sensors: plateau |angle|={plateau:.1f}%, "
        f"1.00 -> {quant_at_1.skipped_pct:.1f}%, "
        f"1.002 -> {quant_at_1002.skipped_pct:.1f}%, "
        f"1.05 -> {quant_at_105.skipped_pct:.1f}%"
    )

    # Thresholds recalibrated for the independent-axis gate: quantized-angle
    # plateaus no longer block position from firing, so the cliff above
    # log_base 1.0 is real but much smaller than with the old angle-first gate
    # (measured: 1.00 -> 1.0%, 1.002 -> 5.7%, 1.05 -> 7.5%).
    assert float_at_105.skipped_pct < 20.0
    assert quant_at_1.skipped_pct < 2.0
    assert quant_at_1002.skipped_pct > quant_at_1.skipped_pct + 3.0
    assert quant_at_105.skipped_pct >= quant_at_1002.skipped_pct
    assert quant_at_105.skipped_pct > 4.0
    assert plateau > 25.0


def test_gate_skip_breakdown_separates_quant_plateau(capsys):
    """Quant skips split into flat-input (ADC steps) vs changed-input (true Secloc)."""
    log_base = 1.05
    quant = record_stabilized_trajectory(
        noise_scale=1.0, seed=123, quantize_sensors=True
    )
    float_traj = record_stabilized_trajectory(
        noise_scale=1.0, seed=123, quantize_sensors=False
    )
    quant_row = replay_gate_breakdown(quant.samples, log_base=log_base)
    float_row = replay_gate_breakdown(float_traj.samples, log_base=log_base)

    print(
        f"\nQuant @ {log_base}: total skip {quant_row.skipped_pct:.1f}%, "
        f"skip|flat {quant_row.skip_given_flat_pct:.1f}%, "
        f"skip|changed {quant_row.skip_given_changed_pct:.1f}%, "
        f"plateau-skip share {quant_row.quant_plateau_skip_share_pct:.1f}%"
    )
    print(
        f"Float @ {log_base}: total skip {float_row.skipped_pct:.1f}%, "
        f"skip|flat {float_row.skip_given_flat_pct:.1f}%, "
        f"skip|changed {float_row.skip_given_changed_pct:.1f}%"
    )

    # Thresholds recalibrated for the independent-axis gate (measured:
    # quant total 7.5% vs float 1.5%, skip|flat 100%, skip|changed 5.7% vs 1.5%,
    # plateau-skip share 25%).
    assert quant_row.skipped_pct > float_row.skipped_pct + 4.0
    assert quant_row.skip_given_flat_pct > 95.0
    assert quant_row.skip_given_changed_pct > float_row.skip_given_changed_pct + 2.0
    assert quant_row.quant_plateau_skip_share_pct < 40.0
    assert float_row.skip_given_changed_pct < 15.0


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
    paths = generate_float_sensor_plot(recompute=True, show_progress=False)
    for path in paths:
        print(f"\nPlot saved to: {path}")
        assert path.is_file()


@pytest.mark.slow
def test_plot_quantized_sensor_curves(capsys):
    paths = generate_quantized_sensor_plot(recompute=True, show_progress=False)
    for path in paths:
        print(f"\nQuantized sensor plot saved to: {path}")
        assert path.is_file()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--quant",
        action="store_true",
        help="Plot skip curves with quantized ADC/encoder sensors",
    )
    parser.add_argument(
        "--plot-only",
        action="store_true",
        help="Replot from cached sweep data (fast; no simulation)",
    )
    parser.add_argument(
        "--recompute",
        action="store_true",
        help="Ignore cache and rerun the full sweep",
    )
    parser.add_argument(
        "--decompose",
        action="store_true",
        help="Print skip decomposition (skip|flat vs skip|changed) for quant vs float",
    )
    cli_args = parser.parse_args()

    if cli_args.decompose:
        print_quant_vs_float_breakdown()
        raise SystemExit(0)

    if cli_args.quant:
        outs = generate_quantized_sensor_plot(
            plot_only=cli_args.plot_only,
            recompute=cli_args.recompute,
        )
    else:
        outs = generate_float_sensor_plot(
            plot_only=cli_args.plot_only,
            recompute=cli_args.recompute,
        )
    for out in outs:
        print(f"Wrote {out}")
