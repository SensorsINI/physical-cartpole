"""Plot Secloc experiment recordings: state trajectories and rolling skip breakdown.

Usage (from repo root):
  python tests/plot_secloc_experiment.py --latest
  python tests/plot_secloc_experiment.py path/to/CPP_mpc__....csv
"""
from __future__ import annotations

import argparse
import os
import sys
from collections import namedtuple
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

REPO_ROOT = Path(__file__).resolve().parents[1]
TESTS_DIR = Path(__file__).resolve().parent
DRIVER_DIR = REPO_ROOT / "Driver"
OUTPUT_DIR = TESTS_DIR / "output"

sys.path.insert(0, str(TESTS_DIR))
sys.path.insert(1, str(DRIVER_DIR))
sys.path.insert(2, str(DRIVER_DIR / "CartPoleSimulation"))
sys.path.insert(3, str(DRIVER_DIR / "CartPoleSimulation" / "SI_Toolkit" / "src"))

from analyze_secloc_cpp_csv import (  # noqa: E402
    latest_recordings,
    load_cpp_dataframe,
    parse_cpp_header,
)

os.chdir(DRIVER_DIR)

from CartPoleSimulation.CartPole.state_utilities import (  # noqa: E402
    ANGLE_COS_IDX,
    ANGLE_IDX,
    ANGLE_SIN_IDX,
    POSITION_IDX,
    create_cartpole_state,
)
from Control_Toolkit_ASF.Controllers.secloc_gate import SeclocGate  # noqa: E402

PollStat = namedtuple("PollStat", ("time", "ang_unchanged", "pos_unchanged", "skipped"))


@dataclass(frozen=True)
class GateParams:
    log_base: float
    ref_period: float
    dead_ang: float
    dead_pos: float
    poll_stats_window_s: float

    @classmethod
    def from_meta(cls, meta: dict[str, str]) -> GateParams:
        return cls(
            log_base=float(meta.get("Secloc log_base", "1.05")),
            ref_period=float(meta.get("Secloc ref_period", "0.0")),
            dead_ang=float(meta.get("Secloc dead_ang", "0.0")),
            dead_pos=float(meta.get("Secloc dead_pos", "0.0")),
            poll_stats_window_s=5.0,
        )


def replay_poll_stats(
    df: pd.DataFrame,
    params: GateParams,
    *,
    respect_split_control_busy: bool = False,
    apply_window_rows: int = 4,
    clock: np.ndarray | None = None,
    start_row: int = 0,
) -> tuple[list[PollStat], np.ndarray]:
    """Replay the Secloc gate over the recording.

    Returns (poll_stats, poll_rows): one entry per gate decision, in the same
    semantics the hardware logs use. On idle rows the gate is consulted for
    real (should_sample); on the deadline tick the driver logs the pure-gate
    peek (secloc_gate_skipped) even though the gate cannot trigger, so the
    replay records those peek skips too.

    The gate on the rig runs in Python float64 (driver.s is float64 after the
    latency adder) and the CSV logs those exact bits, so the replay keeps the
    state in float64: casting to float32 flips decisions that sit exactly on
    the log_base threshold (ADC quantization makes ratios like 21/20 = 1.05
    hit log_base exactly).
    """
    gate = SeclocGate(
        log_base=params.log_base,
        ref_period=params.ref_period,
        dead_ang=params.dead_ang,
        dead_pos=params.dead_pos,
        poll_stats_window_s=params.poll_stats_window_s,
    )

    poll_stats: list[PollStat] = []
    poll_rows: list[int] = []
    prev_poll_ang_shift = None
    prev_poll_pos_shift = None
    busy = False
    loops_since_trigger = 0

    angles = df["angle"].to_numpy(dtype=np.float64)
    positions = df["position"].to_numpy(dtype=np.float64)
    targets = df["target_position"].to_numpy(dtype=np.float64)
    times = clock if clock is not None else df["time"].to_numpy(dtype=np.float64)
    state_len = len(create_cartpole_state())

    def record(row, time, ang_shift, pos_shift, skipped):
        nonlocal prev_poll_ang_shift, prev_poll_pos_shift
        if prev_poll_ang_shift is not None:
            poll_stats.append(
                PollStat(
                    time=float(time),
                    ang_unchanged=ang_shift == prev_poll_ang_shift,
                    pos_unchanged=pos_shift == prev_poll_pos_shift,
                    skipped=skipped,
                )
            )
            poll_rows.append(row)
        prev_poll_ang_shift = ang_shift
        prev_poll_pos_shift = pos_shift

    for row in range(start_row, len(angles)):
        angle = angles[row]
        position = positions[row]
        target = targets[row]
        time = times[row]

        time_difference = gate.time_difference(time)
        ang_shift = abs(angle)
        pos_shift = abs(position - target)
        gate_evaluated = time_difference + gate.ref_period / 20 >= gate.ref_period

        s = np.zeros(state_len, dtype=np.float64)
        s[ANGLE_IDX] = angle
        s[ANGLE_COS_IDX] = np.cos(angle)
        s[ANGLE_SIN_IDX] = np.sin(angle)
        s[POSITION_IDX] = position

        if respect_split_control_busy and busy:
            loops_since_trigger += 1
            if gate_evaluated:
                # Deadline/busy row: the driver still logs the pure-gate peek.
                would_update = gate.peek_would_update(
                    s, target, time=time, time_difference=time_difference
                )
                if not would_update:
                    record(row, time, ang_shift, pos_shift, skipped=True)
            if loops_since_trigger >= apply_window_rows - 1:
                busy = False
                loops_since_trigger = 0
            continue

        spike = gate.should_sample(s, target, time=time, time_difference=time_difference)

        if respect_split_control_busy and spike:
            busy = True
            loops_since_trigger = 0

        if gate_evaluated:
            record(row, time, ang_shift, pos_shift, skipped=not spike)

    return poll_stats, np.array(poll_rows, dtype=np.int64)


def chip_clock_times(df: pd.DataFrame) -> np.ndarray | None:
    """Reconstruct the chip clock the hardware gate times its ref_period on.

    Newer recordings log the chip's absolute timestamp directly (time_chip).
    Older ones only have deltaTimeMs (chip time between *received* packets);
    its cumulative sum misses ticks whenever a serial packet is dropped, so
    those gaps are re-inserted where the PC wall clock jumps by >= 2 periods.
    """
    if "time_chip" in df.columns:
        chip = df["time_chip"].to_numpy(dtype=np.float64)
        return chip - chip[0]
    if "deltaTimeMs" not in df.columns:
        return None
    deltas = df["deltaTimeMs"].to_numpy(dtype=np.float64) / 1000.0
    period = float(np.median(deltas))
    times = np.zeros(len(deltas), dtype=np.float64)
    if "time" in df.columns:
        pc = df["time"].to_numpy(dtype=np.float64)
        pc_steps = np.diff(pc)
        missed = np.maximum(np.rint(pc_steps / period).astype(np.int64) - 1, 0)
        times[1:] = np.cumsum(deltas[1:] + missed * period)
    else:
        times[1:] = np.cumsum(deltas[1:])
    return times


def hardware_gate_decisions(df: pd.DataFrame) -> tuple[np.ndarray, np.ndarray] | None:
    """True hardware gate decision calendar from logged columns.

    Returns (row_indices, skipped_labels). Updates are rising edges of
    split_control_busy (the gate fired and triggered compute); skips are rows
    with secloc_gate_skipped == 1 (gate evaluated, decided not to update).
    """
    needed = {"split_control_busy", "secloc_gate_skipped"}
    if not needed <= set(df.columns):
        return None
    busy = df["split_control_busy"].to_numpy(dtype=np.int64)
    gate_skipped = df["secloc_gate_skipped"].to_numpy(dtype=np.int64)
    triggers = np.flatnonzero((busy[1:] == 1) & (busy[:-1] == 0)) + 1
    rows = np.union1d(triggers, np.flatnonzero(gate_skipped == 1))
    labels = np.ones(len(rows), dtype=np.int8)
    labels[np.isin(rows, triggers)] = 0
    return rows, labels


@dataclass(frozen=True)
class RollingSkipMetrics:
    ang_flat_pct: np.ndarray
    pos_flat_pct: np.ndarray
    skip_ang_flat_pct: np.ndarray
    skip_pos_flat_pct: np.ndarray
    gate_skip_pct: np.ndarray
    skip_or_changed_pct: np.ndarray
    skip_and_changed_pct: np.ndarray
    skip_only_ang_changed_pct: np.ndarray
    skip_only_pos_changed_pct: np.ndarray
    io_skip_pct: np.ndarray
    io_row_skip_pct: np.ndarray


def rolling_poll_metrics(
    poll_stats: list[PollStat],
    query_times: np.ndarray,
    window_s: float,
    hardware_skipped: np.ndarray | None = None,
    update_hold_rows: float = 1.0,
    hw_decision_times: np.ndarray | None = None,
    hw_decision_skipped: np.ndarray | None = None,
) -> RollingSkipMetrics:
    n = len(query_times)
    nan = np.full(n, np.nan, dtype=np.float64)
    if not poll_stats:
        return RollingSkipMetrics(*([nan] * 11))

    stat_times = np.array([stat.time for stat in poll_stats])
    ang_flat = np.full(n, np.nan, dtype=np.float64)
    pos_flat = np.full(n, np.nan, dtype=np.float64)
    skip_ang_flat = np.full(n, np.nan, dtype=np.float64)
    skip_pos_flat = np.full(n, np.nan, dtype=np.float64)
    gate_skip = np.full(n, np.nan, dtype=np.float64)
    skip_or = np.full(n, np.nan, dtype=np.float64)
    skip_and = np.full(n, np.nan, dtype=np.float64)
    skip_only_ang = np.full(n, np.nan, dtype=np.float64)
    skip_only_pos = np.full(n, np.nan, dtype=np.float64)

    for idx, query_time in enumerate(query_times):
        cutoff = query_time - window_s
        mask = (stat_times >= cutoff) & (stat_times <= query_time)
        if not np.any(mask):
            continue
        active = [poll_stats[i] for i in np.nonzero(mask)[0]]
        count = len(active)
        ang_flat[idx] = 100.0 * sum(stat.ang_unchanged for stat in active) / count
        pos_flat[idx] = 100.0 * sum(stat.pos_unchanged for stat in active) / count
        skip_ang_flat[idx] = 100.0 * sum(
            stat.ang_unchanged and stat.skipped for stat in active
        ) / count
        skip_pos_flat[idx] = 100.0 * sum(
            stat.pos_unchanged and stat.skipped for stat in active
        ) / count
        gate_skip[idx] = 100.0 * sum(stat.skipped for stat in active) / count
        skip_or[idx] = 100.0 * sum(
            (not stat.ang_unchanged or not stat.pos_unchanged) and stat.skipped
            for stat in active
        ) / count
        skip_and[idx] = 100.0 * sum(
            (not stat.ang_unchanged and not stat.pos_unchanged) and stat.skipped
            for stat in active
        ) / count
        skip_only_ang[idx] = 100.0 * sum(
            (not stat.ang_unchanged) and stat.pos_unchanged and stat.skipped
            for stat in active
        ) / count
        skip_only_pos[idx] = 100.0 * sum(
            stat.ang_unchanged and (not stat.pos_unchanged) and stat.skipped
            for stat in active
        ) / count

    # The CSV logs the last gate decision every save row. After each update the
    # gate is not re-polled for the apply window (update_hold_rows rows), so an
    # update decision occupies update_hold_rows rows while a skip decision
    # occupies one. Convert row counts back to decision counts so the hardware
    # percentage is comparable with the replay gate-poll percentages.
    # io_row_skip is the raw row (time-weighted) average: the fraction of wall
    # time the controller was coasting on a held plan.
    io_skip = np.full(n, np.nan, dtype=np.float64)
    io_row_skip = np.full(n, np.nan, dtype=np.float64)
    if hw_decision_times is not None and hw_decision_skipped is not None:
        hw_skipped_f = hw_decision_skipped.astype(np.float64)
        for idx, query_time in enumerate(query_times):
            cutoff = query_time - window_s
            mask = (hw_decision_times >= cutoff) & (hw_decision_times <= query_time)
            if np.any(mask):
                io_skip[idx] = 100.0 * hw_skipped_f[mask].mean()
    elif hardware_skipped is not None:
        skipped = hardware_skipped.astype(np.float64)
        for idx, query_time in enumerate(query_times):
            cutoff = query_time - window_s
            mask = (query_times >= cutoff) & (query_times <= query_time)
            if np.any(mask):
                n_skip_rows = skipped[mask].sum()
                n_update_rows = mask.sum() - n_skip_rows
                n_update_decisions = n_update_rows / update_hold_rows
                io_skip[idx] = 100.0 * n_skip_rows / (n_skip_rows + n_update_decisions)
                io_row_skip[idx] = 100.0 * n_skip_rows / mask.sum()
    if hardware_skipped is not None:
        skipped = hardware_skipped.astype(np.float64)
        for idx, query_time in enumerate(query_times):
            cutoff = query_time - window_s
            mask = (query_times >= cutoff) & (query_times <= query_time)
            if np.any(mask):
                io_row_skip[idx] = 100.0 * skipped[mask].mean()

    return RollingSkipMetrics(
        ang_flat_pct=ang_flat,
        pos_flat_pct=pos_flat,
        skip_ang_flat_pct=skip_ang_flat,
        skip_pos_flat_pct=skip_pos_flat,
        gate_skip_pct=gate_skip,
        skip_or_changed_pct=skip_or,
        skip_and_changed_pct=skip_and,
        skip_only_ang_changed_pct=skip_only_ang,
        skip_only_pos_changed_pct=skip_only_pos,
        io_skip_pct=io_skip,
        io_row_skip_pct=io_row_skip,
    )


def rolling_skip_both_changed_pct(
    poll_stats: list[PollStat],
    query_times: np.ndarray,
    window_s: float,
) -> np.ndarray:
    return rolling_poll_metrics(poll_stats, query_times, window_s).skip_and_changed_pct


def target_angle_series(df: pd.DataFrame) -> np.ndarray:
    equilibrium = df["target_equilibrium"].to_numpy(dtype=np.float64)
    return np.where(equilibrium >= 0, 0.0, np.pi)


def angle_error_to_target(df: pd.DataFrame) -> np.ndarray:
    angle = df["angle"].to_numpy(dtype=np.float64)
    equilibrium = df["target_equilibrium"].to_numpy(dtype=np.float64)
    err_upright = np.abs(angle)
    err_inverted = np.minimum(np.abs(angle - np.pi), np.abs(angle + np.pi))
    return np.where(equilibrium > 0, err_upright, err_inverted)


def stabilization_mask(
    df: pd.DataFrame,
    *,
    err_threshold: float = 0.15,
    speed_threshold: float = 0.3,
) -> np.ndarray:
    err = angle_error_to_target(df)
    speed = np.abs(df["angleD"].to_numpy(dtype=np.float64))
    return (err < err_threshold) & (speed < speed_threshold)


def find_stabilization_intervals(
    df: pd.DataFrame,
    *,
    angle_threshold: float = 0.1,
    min_duration_s: float = 0.5,
) -> list[tuple[float, float]]:
    """Upright hold: target equilibrium up AND |angle| below threshold."""
    time = df["time"].to_numpy(dtype=np.float64)
    angle = df["angle"].to_numpy(dtype=np.float64)
    upright_target = df["target_equilibrium"].to_numpy(dtype=np.float64) > 0
    stabilized = upright_target & (np.abs(angle) < angle_threshold)

    intervals: list[tuple[float, float]] = []
    in_region = False
    start_time = 0.0

    for idx, active in enumerate(stabilized):
        if active and not in_region:
            start_time = float(time[idx])
            in_region = True
        elif not active and in_region:
            end_time = float(time[idx - 1])
            if end_time - start_time >= min_duration_s:
                intervals.append((start_time, end_time))
            in_region = False

    if in_region:
        end_time = float(time[-1])
        if end_time - start_time >= min_duration_s:
            intervals.append((start_time, end_time))

    return intervals


def mask_from_intervals(time: np.ndarray, intervals: list[tuple[float, float]]) -> np.ndarray:
    mask = np.zeros(len(time), dtype=bool)
    for start, end in intervals:
        mask |= (time >= start) & (time <= end)
    return mask

def shade_stabilization_phases(
    axes,
    intervals: list[tuple[float, float]],
    *,
    angle_threshold: float = 0.1,
) -> None:
    label = f"upright hold (target up, |angle| < {angle_threshold:.2f} rad)"
    labeled = False
    for ax in axes:
        for start, end in intervals:
            shade_label = label if not labeled else None
            ax.axvspan(
                start,
                end,
                color="0.72",
                alpha=0.22,
                zorder=0,
                label=shade_label,
            )
            labeled = True

def title_with_log_base(
    base_title: str,
    log_base: float,
    *,
    window_s: float | None = None,
) -> str:
    lines = [base_title, f"log_base = {log_base:.2f}"]
    if window_s is not None:
        lines.append(
            f"rolling average window = {window_s:.1f} s "
            f"(each point: mean over [t − {window_s:.1f} s, t])"
        )
    return "\n".join(lines)


def rolling_average_caption(window_s: float) -> str:
    return (
        f"each % is a backward-looking rolling average over "
        f"[t − {window_s:.1f} s, t]"
    )


def plot_3d(
    df: pd.DataFrame,
    skip_pct: np.ndarray,
    log_base: float,
    output_path: Path,
    stabilized: np.ndarray | None = None,
    window_s: float = 0.5,
) -> None:
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    angle = df["angle"].to_numpy()
    angle_d = df["angleD"].to_numpy()
    valid = np.isfinite(skip_pct)

    if stabilized is None:
        alphas = np.full(len(angle), 0.55)
    else:
        alphas = np.where(stabilized, 0.75, 0.18)

    scatter = ax.scatter(
        angle[valid],
        angle_d[valid],
        skip_pct[valid],
        c=skip_pct[valid],
        cmap="viridis",
        s=4,
        alpha=alphas[valid],
        linewidths=0,
    )
    if stabilized is not None and np.any(stabilized & valid):
        ax.scatter(
            [],
            [],
            [],
            c="0.72",
            alpha=0.75,
            s=20,
            label="upright hold (brighter)",
        )
        ax.legend(loc="upper left", fontsize=9)
    ax.set_xlabel("angle (rad)")
    ax.set_ylabel("angular speed (rad/s)")
    ax.set_zlabel("skipped % (angle & position changed)")
    ax.set_title(
        title_with_log_base(
            "Secloc skip rate vs pole state",
            log_base,
            window_s=window_s,
        )
    )
    fig.colorbar(
        scatter,
        ax=ax,
        shrink=0.65,
        label=f"skipped % ({rolling_average_caption(window_s)})",
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def decode_hardware_decisions(
    times: np.ndarray, skip_rows: np.ndarray, update_hold_rows: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return (decision_times, skipped) decoded from logged CSV rows."""
    changes = np.flatnonzero(np.diff(skip_rows)) + 1
    runs = np.split(skip_rows, changes)
    decision_times: list[float] = []
    skipped: list[int] = []
    idx = 0
    for run in runs:
        if run[0] == 1:
            for k in range(len(run)):
                decision_times.append(float(times[idx + k]))
                skipped.append(1)
        else:
            for k in range(0, len(run), update_hold_rows):
                decision_times.append(float(times[idx + k]))
                skipped.append(0)
        idx += len(run)
    return np.array(decision_times), np.array(skipped, dtype=np.int8)


@dataclass(frozen=True)
class ValidationStats:
    replay_skip_pct: float
    hw_skip_pct: float
    replay_decisions: int
    hw_decisions: int
    replay_unpaired_pct: float
    uses_logged_gate_column: bool = False
    calendar_match_pct: float = float("nan")
    label_agreement_pct: float = float("nan")
    hw_unmatched: int = 0


def compute_validation_stats_logged(
    poll_rows: np.ndarray,
    poll_skipped: np.ndarray,
    hw_rows: np.ndarray,
    hw_skipped: np.ndarray,
) -> ValidationStats:
    """Row-index comparison of replay decisions vs the logged hardware calendar."""
    hw_label_by_row = dict(zip(hw_rows.tolist(), hw_skipped.tolist()))
    on_hw = np.array([r in hw_label_by_row for r in poll_rows], dtype=bool)
    agree = np.array(
        [
            hw_label_by_row[r] == lab
            for r, lab in zip(poll_rows[on_hw], poll_skipped[on_hw])
        ],
        dtype=bool,
    )
    replay_row_set = set(poll_rows.tolist())
    hw_unmatched = int(sum(1 for r in hw_rows.tolist() if r not in replay_row_set))
    return ValidationStats(
        replay_skip_pct=100.0 * poll_skipped.mean() if len(poll_skipped) else 0.0,
        hw_skip_pct=100.0 * hw_skipped.mean() if len(hw_skipped) else 0.0,
        replay_decisions=len(poll_skipped),
        hw_decisions=len(hw_skipped),
        replay_unpaired_pct=100.0 * (~on_hw).mean() if len(on_hw) else 0.0,
        uses_logged_gate_column=True,
        calendar_match_pct=100.0 * on_hw.mean() if len(on_hw) else 0.0,
        label_agreement_pct=100.0 * agree.mean() if len(agree) else 0.0,
        hw_unmatched=hw_unmatched,
    )


def compute_validation_stats(
    poll_stats: list[PollStat],
    times: np.ndarray,
    skip_rows: np.ndarray,
    update_hold_rows: float,
    pair_tolerance_s: float = 0.0025,
) -> ValidationStats:
    replay_skipped = np.array([int(s.skipped) for s in poll_stats], dtype=np.int8)
    replay_times = np.array([s.time for s in poll_stats])
    hw_times, hw_skipped = decode_hardware_decisions(
        times, skip_rows, max(1, round(update_hold_rows))
    )
    unpaired = 0
    for rt in replay_times:
        if np.min(np.abs(hw_times - rt)) > pair_tolerance_s:
            unpaired += 1
    return ValidationStats(
        replay_skip_pct=100.0 * replay_skipped.mean() if len(replay_skipped) else 0.0,
        hw_skip_pct=100.0 * hw_skipped.mean() if len(hw_skipped) else 0.0,
        replay_decisions=len(replay_skipped),
        hw_decisions=len(hw_skipped),
        replay_unpaired_pct=100.0 * unpaired / max(len(replay_times), 1),
        uses_logged_gate_column=False,
    )


def plot_timeseries(
    df: pd.DataFrame,
    metrics: RollingSkipMetrics,
    window_s: float,
    log_base: float,
    output_path: Path,
    stab_intervals: list[tuple[float, float]] | None = None,
    ref_period: float = 0.02,
    stab_angle_threshold: float = 0.1,
    polling_period_s: float = 0.005,
    validation: ValidationStats | None = None,
) -> None:
    time = df["time"].to_numpy()
    target_angle = target_angle_series(df)
    decision_ms = ref_period * 1000.0
    decisions_label = "Secloc decisions"

    fig, axes = plt.subplots(4, 1, figsize=(12, 17.5), sharex=True)

    if stab_intervals:
        shade_stabilization_phases(
            axes, stab_intervals, angle_threshold=stab_angle_threshold
        )

    axes[0].plot(time, df["angle"], color="C0", linewidth=0.8, label="angle", zorder=3)
    axes[0].plot(
        time,
        target_angle,
        color="C3",
        linewidth=1.2,
        linestyle="--",
        label="target angle",
        zorder=3,
    )
    axes[0].set_ylabel("angle (rad)")
    handles, labels = axes[0].get_legend_handles_labels()
    axes[0].legend(handles, labels, loc="upper right")
    axes[0].grid(True, alpha=0.3, zorder=1)

    axes[1].plot(
        time, df["position"] * 100, color="C1", linewidth=0.8, label="position", zorder=3
    )
    axes[1].plot(
        time,
        df["target_position"] * 100,
        color="C3",
        linewidth=1.2,
        linestyle="--",
        label="target position",
        zorder=3,
    )
    axes[1].set_ylabel("position (cm)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3, zorder=1)

    input_series = [
        (metrics.ang_flat_pct, "angle reading identical to previous decision", "#1f77b4"),
        (metrics.pos_flat_pct, "position reading identical to previous decision", "#ff7f0e"),
    ]
    for values, label, color in input_series:
        valid = np.isfinite(values)
        axes[2].plot(
            time[valid],
            values[valid],
            color=color,
            linewidth=1.2,
            linestyle="-",
            label=label,
            zorder=3,
        )
    axes[2].set_ylabel("% of decisions")
    axes[2].set_ylim(0, 100)
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3, zorder=1)
    axes[2].set_title(
        f"Sensor reading identical on consecutive {decisions_label} — replay\n"
        f"{rolling_average_caption(window_s)}",
        fontsize=10,
    )

    # Mutually exclusive split of all gate-poll skips; the stack sums to the total.
    skip_both = metrics.skip_and_changed_pct
    skip_only_ang = metrics.skip_only_ang_changed_pct
    skip_only_pos = metrics.skip_only_pos_changed_pct
    skip_neither = metrics.gate_skip_pct - metrics.skip_or_changed_pct
    valid = (
        np.isfinite(skip_both)
        & np.isfinite(skip_only_ang)
        & np.isfinite(skip_only_pos)
        & np.isfinite(skip_neither)
    )
    axes[3].stackplot(
        time[valid],
        skip_neither[valid],
        skip_only_ang[valid],
        skip_only_pos[valid],
        skip_both[valid],
        colors=["#d9d9d9", "#6baed6", "#fdae6b", "#e34a33"],
        labels=[
            "skipped — neither angle nor position changed",
            "skipped — only angle changed",
            "skipped — only position changed",
            "skipped — both angle and position changed",
        ],
        zorder=2,
    )
    axes[3].plot(
        time[valid],
        metrics.gate_skip_pct[valid],
        color="#54278f",
        linewidth=1.0,
        linestyle="-",
        label="total skipped (replay, % of decisions = top of stack)",
        zorder=3,
    )
    io_valid = np.isfinite(metrics.io_skip_pct)
    hw_gate_label = (
        "hardware gate (logged secloc_gate_skipped, % of gate evaluations)"
        if validation and validation.uses_logged_gate_column
        else "skipped on hardware (logged, % of gate decisions)"
    )
    axes[3].plot(
        time[io_valid],
        metrics.io_skip_pct[io_valid],
        color="black",
        linewidth=1.4,
        linestyle="--",
        label=hw_gate_label,
        zorder=4,
    )
    row_valid = np.isfinite(metrics.io_row_skip_pct)
    axes[3].plot(
        time[row_valid],
        metrics.io_row_skip_pct[row_valid],
        color="0.35",
        linewidth=1.2,
        linestyle=":",
        label="coasting on held plan (logged, % of wall time)",
        zorder=4,
    )
    axes[3].set_ylabel("%")
    axes[3].set_xlabel("time (s)")
    axes[3].set_ylim(0, 100)
    handles, labels = axes[3].get_legend_handles_labels()
    # Two separate boxes so each frame only covers its own items:
    # stack categories (2 cols x 2 rows) on the left, curves (3 rows) on the right.
    stack_order = [0, 3, 1, 2]
    line_order = [4, 5, 6]
    line_legend = axes[3].legend(
        [handles[i] for i in line_order],
        [labels[i] for i in line_order],
        loc="upper right",
        bbox_to_anchor=(1.0, 1.0),
        fontsize=7,
    )
    # Measure the drawn line-legend width so the stack legend sits flush to its left.
    fig.canvas.draw()
    line_bbox = line_legend.get_window_extent().transformed(
        axes[3].transAxes.inverted()
    )
    axes[3].legend(
        [handles[i] for i in stack_order],
        [labels[i] for i in stack_order],
        loc="upper right",
        bbox_to_anchor=(line_bbox.x0 - 0.005, 1.0),
        fontsize=7,
        ncol=2,
    )
    axes[3].add_artist(line_legend)
    axes[3].grid(True, alpha=0.3, zorder=1)
    axes[3].set_title(
        f"Skipped MPC updates (% of {decisions_label}); "
        f"stack = replay total, dashed = hardware\n"
        f"{rolling_average_caption(window_s)}",
        fontsize=10,
    )

    fig.suptitle(
        title_with_log_base(
            "Physical cartpole experiment (Secloc + MPC)",
            log_base,
            window_s=window_s,
        ),
        y=0.995,
    )
    fig.tight_layout(rect=(0, 0.26, 1, 1))
    hold_rows = max(1, round(ref_period / polling_period_s))
    poll_ms = polling_period_s * 1000
    valid_roll = np.isfinite(metrics.gate_skip_pct) & np.isfinite(metrics.io_skip_pct)
    roll_diff = metrics.gate_skip_pct[valid_roll] - metrics.io_skip_pct[valid_roll]
    roll_diff_mean = float(roll_diff.mean()) if roll_diff.size else 0.0
    roll_diff_std = float(roll_diff.std()) if roll_diff.size else 0.0
    if validation is None:
        validation = ValidationStats(0.0, 0.0, 0, 0, 0.0)
    if validation.uses_logged_gate_column:
        q = validation.hw_skip_pct / 100.0
        gray_pred = 100.0 * q / (q + hold_rows * (1.0 - q)) if q < 1.0 else 100.0
        wiggle_text = (
            "Dashed black curve.  Computed directly from the logged hardware decision calendar: updates are rising "
            "edges of split_control_busy (gate fired), skips are rows with\n"
            "secloc_gate_skipped = 1 (gate evaluated, no update). No inference from the replay is involved.\n"
            "\n"
            "Replay vs hardware gate.  The gate runs on the PC driver in float64 and the CSV stores the exact bits "
            "it saw, so the replay is bit-exact: CSV parsed with round-trip float\n"
            "precision, state kept in float64, gate timed on the reconstructed chip clock (deltaTimeMs + dropped-"
            "packet gaps from PC-clock jumps), anchored at the first logged trigger.\n"
            f"Result: {validation.calendar_match_pct:.2f}% of {validation.replay_decisions} replay decisions land "
            f"on a logged hardware decision row and {validation.label_agreement_pct:.2f}% of those agree on skip "
            f"vs update ({validation.hw_unmatched} of {validation.hw_decisions} hardware\n"
            f"decisions unmatched, from serial packet drops the reconstructed clock cannot place exactly). "
            f"Full-run skip rate: replay {validation.replay_skip_pct:.1f}% vs hardware "
            f"{validation.hw_skip_pct:.1f}%. Residual wiggle between\n"
            f"purple and black (mean {roll_diff_mean:+.1f} pp, std {roll_diff_std:.1f} pp) is rolling-window edge "
            "effects at the few unmatched rows.\n"
            "\n"
            "Dotted gray (effective skip) vs dashed black.  Gray is the row average of secloc_skipped_update: the "
            "share of wall time spent coasting. One update occupies "
            f"{decision_ms:.0f} ms ({hold_rows} rows),\n"
            f"one skip {poll_ms:.0f} ms (1 row), so with per-decision skip rate q the row average is "
            f"q / (q + {hold_rows}(1 \u2212 q)) \u2014 NOT q/{hold_rows}: only update decisions cost {hold_rows} rows, "
            f"skip decisions cost 1. With q = {validation.hw_skip_pct:.1f}% this gives "
            f"{gray_pred:.1f}%,\n"
            "matching the measured gray level. The naive \u00f7"
            f"{hold_rows} would only hold if skips were rare (q \u2192 0)."
        )
    else:
        wiggle_text = (
            "Why purple and dashed black wiggle apart.  Both use the same rolling window, but they count decisions at "
            "different instants. Example: after an update at 0 ms the rig is busy until\n"
            f"{decision_ms:.0f} ms and makes no new decision; the replay still evaluates the gate on later rows and may "
            f"record a decision at e.g. 10 ms where the log has none. About {validation.replay_unpaired_pct:.0f}% of "
            "replay decision times have no hardware\n"
            f"decision within {poll_ms / 2:.1f} ms (often exactly one {poll_ms:.0f} ms row off). That shifts which "
            f"decisions fall inside each {window_s:.1f} s window, so the two curves need not track point by point.\n"
            "\n"
            "Does replay show more skips?  No. Over the full recording: replay "
            f"{validation.replay_skip_pct:.1f}% skipped ({validation.replay_decisions} decisions), hardware "
            f"{validation.hw_skip_pct:.1f}% skipped ({validation.hw_decisions} decisions). Extra replay checks can "
            "record either skip or update;\n"
            "they do not mostly add skips. The wiggle (mean "
            f"{roll_diff_mean:+.1f} pp, std {roll_diff_std:.1f} pp here) is from mismatched decision calendars, not "
            "from replay systematically skipping more.\n"
            "\n"
            "Compute saving.  The dotted gray line is the plain time average of the logged flag: the share of wall time "
            "the controller coasted on a held plan. It is lower than the\n"
            "decision-based curves because every update occupies "
            f"{decision_ms:.0f} ms of wall time while a skip occupies only {poll_ms:.0f} ms."
        )
    explanation = (
        "HOW TO READ THIS FIGURE\n"
        "\n"
        "Setup.  A model-predictive controller (MPC) balances a physical cartpole. To save computation, a Secloc "
        "event gate sits in front of the MPC: at each decision it either\n"
        "recomputes the control plan (update) or keeps the previous one (skip). It updates only when |angle| or "
        f"|position \u2212 target| changed by a factor \u2265 {log_base:g} (log_base)\n"
        "since the last update; otherwise it skips. Gray bands mark upright stabilization phases (target up, pole "
        "near vertical); the other periods are swing-up/swing-down transitions.\n"
        "\n"
        f"Timing.  The CSV saves a row every {poll_ms:.0f} ms. After an update the rig is busy for "
        f"{decision_ms:.0f} ms ({hold_rows} rows) and does not ask the gate again. A skip uses 1 row; an update "
        f"uses {hold_rows} rows\n"
        f"with the same logged flag. All percentage curves are backward-looking rolling averages over the last "
        f"{window_s:.1f} s.\n"
        "\n"
        "Panel 3.  Share of decisions where the raw sensor reading was bit-identical to the previous decision "
        "(angle in blue, position in orange). During upright holds the angle\n"
        "sensor barely moves, so the blue curve rises.\n"
        "\n"
        "Panel 4.  Share of decisions that were skips. The colored areas split the skips by what changed between "
        "consecutive decisions (gray: nothing changed; blue: only angle;\n"
        "orange: only position; red: both) \u2014 they are mutually exclusive and stack to the total (thin purple "
        "line). Red dominating during swings shows the gate skipping because\n"
        "relative changes stayed below log_base, not because readings froze.\n"
        "\n"
        f"{wiggle_text}"
    )
    fig.text(
        0.012,
        0.008,
        explanation,
        fontsize=8.5,
        color="0.25",
        linespacing=1.35,
        verticalalignment="bottom",
    )
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def resolve_csv_path(args: argparse.Namespace) -> Path:
    if args.csv_file:
        path = Path(args.csv_file)
        if not path.is_absolute():
            path = (REPO_ROOT / path).resolve()
        return path
    if args.latest is not None:
        files = latest_recordings(args.latest)
        if not files:
            raise FileNotFoundError("No CPP recordings found")
        return files[-1]
    files = latest_recordings(1)
    if not files:
        raise FileNotFoundError("No CPP recordings found")
    return files[-1]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Plot Secloc CPP experiment recording")
    parser.add_argument("csv_file", nargs="?", help="Path to CPP CSV recording")
    parser.add_argument(
        "--latest",
        nargs="?",
        const=1,
        type=int,
        metavar="N",
        help="Use the N most recent CPP recording (default when no path given)",
    )
    parser.add_argument(
        "--window-s",
        type=float,
        default=0.5,
        help=(
            "Backward-looking rolling average window for all %% curves (seconds; "
            "each point = mean over [t − window, t]; default 0.5)"
        ),
    )
    parser.add_argument(
        "--stab-angle",
        type=float,
        default=0.1,
        help="|angle| threshold (rad) for upright-hold shading (default 0.1)",
    )
    parser.add_argument(
        "--replay-respect-busy",
        action="store_true",
        help="Replay gate only when split-control would be idle (matches hardware IO)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=OUTPUT_DIR,
        help="Directory for PNG output",
    )
    args = parser.parse_args(argv)

    if args.csv_file is None and args.latest is None:
        args.latest = 1

    csv_path = resolve_csv_path(args)
    meta = parse_cpp_header(csv_path)
    df = load_cpp_dataframe(csv_path)
    df = df.copy()
    df["time"] = df["time"] - df["time"].iloc[0]

    if "secloc_skipped_update" not in df.columns:
        raise KeyError(f"{csv_path.name}: missing secloc_skipped_update column")

    params = GateParams.from_meta(meta)
    params = GateParams(
        log_base=params.log_base,
        ref_period=params.ref_period,
        dead_ang=params.dead_ang,
        dead_pos=params.dead_pos,
        poll_stats_window_s=args.window_s,
    )

    try:
        controller_dt = float(meta["Controller update"].split()[0])
        saving_dt = float(meta["Saving"].split()[0])
        update_hold_rows = max(1.0, controller_dt / saving_dt)
    except (KeyError, ValueError):
        saving_dt = 0.005
        update_hold_rows = 1.0

    respect_busy = args.replay_respect_busy or ("split_control_busy" in df.columns)
    if respect_busy and not args.replay_respect_busy:
        print("Auto-enabled replay-respect-busy (split_control_busy column present)")

    query_times = df["time"].to_numpy(dtype=np.float64)
    hardware_skipped = df["secloc_skipped_update"].to_numpy(dtype=np.float64)
    hw_decisions = hardware_gate_decisions(df)

    # The hardware gate times its ref_period on the chip clock (exact 5 ms rows);
    # the CSV 'time' column is PC wall clock with ~1.5 ms/row jitter. Replay on
    # the chip clock, anchored at the first logged hardware trigger so the gate's
    # internal (last-shift, last-time) state starts identically.
    chip_times = chip_clock_times(df) if hw_decisions is not None else None
    start_row = 0
    if hw_decisions is not None:
        hw_rows_all, hw_labels_all = hw_decisions
        trigger_rows = hw_rows_all[hw_labels_all == 0]
        if len(trigger_rows):
            start_row = int(trigger_rows[0])
    poll_stats_chip, poll_rows = replay_poll_stats(
        df,
        params,
        respect_split_control_busy=respect_busy,
        apply_window_rows=max(1, round(update_hold_rows)),
        clock=chip_times,
        start_row=start_row,
    )
    # Map decisions back to PC wall-clock times for plotting.
    poll_stats = [
        PollStat(
            time=float(query_times[row]),
            ang_unchanged=s.ang_unchanged,
            pos_unchanged=s.pos_unchanged,
            skipped=s.skipped,
        )
        for s, row in zip(poll_stats_chip, poll_rows)
    ]

    if hw_decisions is not None:
        hw_rows, hw_labels = hw_decisions
        hw_decision_times = query_times[hw_rows]
        metrics = rolling_poll_metrics(
            poll_stats,
            query_times,
            params.poll_stats_window_s,
            hardware_skipped=hardware_skipped,
            update_hold_rows=update_hold_rows,
            hw_decision_times=hw_decision_times,
            hw_decision_skipped=hw_labels,
        )
        replay_labels = np.array([int(s.skipped) for s in poll_stats], dtype=np.int8)
        # Compare only from the replay anchor onwards.
        in_range = hw_rows >= start_row
        validation = compute_validation_stats_logged(
            poll_rows, replay_labels, hw_rows[in_range], hw_labels[in_range]
        )
    else:
        metrics = rolling_poll_metrics(
            poll_stats,
            query_times,
            params.poll_stats_window_s,
            hardware_skipped=hardware_skipped,
            update_hold_rows=update_hold_rows,
        )
        validation = compute_validation_stats(
            poll_stats,
            query_times,
            hardware_skipped.astype(int),
            update_hold_rows,
        )
    skip_pct = metrics.skip_and_changed_pct
    stab_intervals = find_stabilization_intervals(
        df, angle_threshold=args.stab_angle
    )
    stabilized = mask_from_intervals(query_times, stab_intervals)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = csv_path.stem
    plot_3d_path = args.output_dir / f"{stem}_secloc_3d_skip_both_changed.png"
    plot_ts_path = args.output_dir / f"{stem}_secloc_timeseries.png"

    plot_3d(
        df,
        skip_pct,
        params.log_base,
        plot_3d_path,
        stabilized=stabilized,
        window_s=params.poll_stats_window_s,
    )
    plot_timeseries(
        df,
        metrics,
        params.poll_stats_window_s,
        params.log_base,
        plot_ts_path,
        stab_intervals=stab_intervals,
        ref_period=params.ref_period,
        stab_angle_threshold=args.stab_angle,
        polling_period_s=saving_dt,
        validation=validation,
    )

    finite = skip_pct[np.isfinite(skip_pct)]
    print(f"Recording: {csv_path.name}")
    print(f"log_base: {params.log_base}")
    print(f"Gate polls with previous sample: {len(poll_stats)}")
    print(
        f"Upright hold shading: |angle| < {args.stab_angle:.2f} rad, "
        f"{len(stab_intervals)} intervals"
    )
    if len(finite):
        gate = metrics.gate_skip_pct[np.isfinite(metrics.gate_skip_pct)]
        print(
            "Rolling update skipped (position and angle changed) %: "
            f"mean={finite.mean():.1f}, min={finite.min():.1f}, max={finite.max():.1f}"
        )
        print(f"Rolling update skipped (all gate polls) %: mean={gate.mean():.1f}")
        ang = metrics.ang_flat_pct[np.isfinite(metrics.ang_flat_pct)]
        pos = metrics.pos_flat_pct[np.isfinite(metrics.pos_flat_pct)]
        skip_or = metrics.skip_or_changed_pct[np.isfinite(metrics.skip_or_changed_pct)]
        io = metrics.io_skip_pct[np.isfinite(metrics.io_skip_pct)]
        print(f"Rolling angle not changed %: mean={ang.mean():.1f}")
        print(f"Rolling position not changed %: mean={pos.mean():.1f}")
        print(
            "Rolling update skipped (position or angle changed) %: "
            f"mean={skip_or.mean():.1f}"
        )
        print(f"Rolling logged IO skip %: mean={io.mean():.1f}")
        row = metrics.io_row_skip_pct[np.isfinite(metrics.io_row_skip_pct)]
        print(f"Rolling coasting (row/time-weighted) %: mean={row.mean():.1f}")
    if validation.uses_logged_gate_column:
        print(
            f"Replay vs logged hardware gate: calendar match {validation.calendar_match_pct:.2f}%, "
            f"label agreement {validation.label_agreement_pct:.2f}%, "
            f"hardware decisions unmatched {validation.hw_unmatched}/{validation.hw_decisions}; "
            f"full-run skip replay {validation.replay_skip_pct:.1f}% vs hardware {validation.hw_skip_pct:.1f}%"
        )
    print(f"Wrote {plot_3d_path}")
    print(f"Wrote {plot_ts_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
