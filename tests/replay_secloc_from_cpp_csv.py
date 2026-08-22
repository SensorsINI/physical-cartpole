"""Phase 2: replay CPP recordings through SeclocGate offline.

Feed logged hardware state into the same gate logic used on the rig and compare
skip decisions with the recorded ``secloc_skipped_update`` column.

Usage (from repo root):
  python tests/replay_secloc_from_cpp_csv.py path/to/CPP_pid__....csv
  python tests/replay_secloc_from_cpp_csv.py --latest
  python tests/replay_secloc_from_cpp_csv.py --latest 3
  python tests/replay_secloc_from_cpp_csv.py --latest 3 --sim-baseline
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
TESTS_DIR = Path(__file__).resolve().parent
RECORDINGS_DIR = REPO_ROOT / "Driver" / "ExperimentRecordings"
DRIVER_DIR = REPO_ROOT / "Driver"

sys.path.insert(0, str(TESTS_DIR))
sys.path.insert(1, str(DRIVER_DIR))
sys.path.insert(2, str(DRIVER_DIR / "CartPoleSimulation"))
sys.path.insert(3, str(DRIVER_DIR / "CartPoleSimulation" / "SI_Toolkit" / "src"))

from analyze_secloc_cpp_csv import (  # noqa: E402
    latest_recordings,
    load_cpp_dataframe,
    parse_cpp_header,
    skip_rate_from_csv,
)

os.chdir(DRIVER_DIR)


def resolve_recording_path(path: Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return (REPO_ROOT / path).resolve()

from CartPoleSimulation.CartPole.state_utilities import (  # noqa: E402
    ANGLE_COS_IDX,
    ANGLE_IDX,
    ANGLE_SIN_IDX,
    POSITION_IDX,
    create_cartpole_state,
)
from Control_Toolkit_ASF.Controllers.secloc_gate import SeclocGate  # noqa: E402
from globals import ANGLE_DEVIATION, ANGLE_NORMALIZATION_FACTOR  # noqa: E402

# CSV rows sit on the chip's 5 ms polling grid; the gate's ref_period throttle
# counts integer ticks of this quantum.
POLL_DT_S = 0.005


@dataclass(frozen=True)
class GateParams:
    log_base: float
    ref_period: float  # seconds; new recordings store ticks, converted on load
    dead_ang: float
    dead_pos: float

    @classmethod
    def from_meta(cls, meta: dict[str, str]) -> GateParams:
        # New recordings log the throttle in control loop ticks
        # ("Secloc ref_period_ticks"); older ones in seconds ("Secloc ref_period").
        if "Secloc ref_period_ticks" in meta:
            ref_period = float(meta["Secloc ref_period_ticks"]) * POLL_DT_S
        else:
            ref_period = float(meta.get("Secloc ref_period", "0.0"))
        return cls(
            log_base=float(meta.get("Secloc log_base", "1.0")),
            ref_period=ref_period,
            dead_ang=float(meta.get("Secloc dead_ang", "0.0")),
            dead_pos=float(meta.get("Secloc dead_pos", "0.0")),
        )

    @property
    def ref_period_ticks(self) -> int:
        return max(1, round(self.ref_period / POLL_DT_S)) if self.ref_period > 0 else 0


@dataclass(frozen=True)
class ReplayResult:
    input_mode: str
    skip_pct: float
    apply_pct: float
    agreement_pct: float | None
    angle_spike_steps: int
    position_spike_steps: int
    update_steps: int
    total_steps: int
    angle_ratio_ge_base_pct: float | None
    angle_abs_mean: float
    angle_abs_max: float
    skip_given_changed_pct: float | None = None
    skip_given_flat_pct: float | None = None
    gate_input_flat_pct: float | None = None


def gate_params_from_args(meta: dict[str, str], args: argparse.Namespace) -> GateParams:
    params = GateParams.from_meta(meta)
    if args.log_base is not None:
        return GateParams(
            log_base=float(args.log_base),
            ref_period=params.ref_period,
            dead_ang=params.dead_ang,
            dead_pos=params.dead_pos,
        )
    return params


def select_angle_series(df: pd.DataFrame, input_mode: str) -> np.ndarray:
    if input_mode == "logged":
        return df["angle"].to_numpy(dtype=np.float64)
    if input_mode == "angle_raw":
        raw = df["angle_raw"].to_numpy(dtype=np.float64)
        return (raw + float(ANGLE_DEVIATION)) * float(ANGLE_NORMALIZATION_FACTOR)
    raise ValueError(f"Unknown input mode: {input_mode!r}")


def naive_angle_ratio_rate(angles: np.ndarray, log_base: float) -> float | None:
    if log_base <= 1.0:
        return None
    magnitude = np.abs(angles)
    positive = magnitude > 0
    if positive.sum() < 2:
        return None
    m = magnitude[positive]
    ratios = np.maximum(m[1:] / m[:-1], m[:-1] / m[1:])
    return 100.0 * float(np.mean(ratios >= log_base))


def replay_secloc(
    df: pd.DataFrame,
    *,
    params: GateParams,
    input_mode: str = "logged",
) -> tuple[np.ndarray, ReplayResult]:
    angles = select_angle_series(df, input_mode)
    positions = df["position"].to_numpy(dtype=np.float64)
    targets = df["target_position"].to_numpy(dtype=np.float64)
    if "target_equilibrium" in df.columns:
        equilibria = df["target_equilibrium"].to_numpy(dtype=np.float64)
    else:
        equilibria = np.ones(len(df), dtype=np.float64)
    times = df["time"].to_numpy(dtype=np.float64)

    gate = SeclocGate(
        log_base=params.log_base,
        ref_period_ticks=params.ref_period_ticks,
        dead_ang=params.dead_ang,
        dead_pos=params.dead_pos,
    )
    gate.set_time_quantum(POLL_DT_S)

    skipped = np.zeros(len(df), dtype=np.int8)
    angle_spikes = 0
    position_spikes = 0

    for idx, (angle, position, target, equilibrium, time) in enumerate(
        zip(angles, positions, targets, equilibria, times)
    ):
        s = create_cartpole_state()
        s[ANGLE_IDX] = angle
        s[ANGLE_COS_IDX] = np.cos(angle)
        s[ANGLE_SIN_IDX] = np.sin(angle)
        s[POSITION_IDX] = position

        ang_before = gate.ang_last_shift
        pos_before = gate.pos_last_shift
        did_update = gate.should_sample(
            s, target, time=time, target_equilibrium=equilibrium
        )
        skipped[idx] = int(not did_update)

        if did_update:
            if gate.ang_last_shift != ang_before:
                angle_spikes += 1
            elif gate.pos_last_shift != pos_before:
                position_spikes += 1

    agreement_pct = None
    if "secloc_skipped_update" in df.columns:
        hardware = df["secloc_skipped_update"].astype(int).to_numpy()
        agreement_pct = 100.0 * float(np.mean(skipped == hardware))

    total = len(skipped)
    updates = int(total - skipped.sum())
    return skipped, ReplayResult(
        input_mode=input_mode,
        skip_pct=100.0 * float(skipped.mean()),
        apply_pct=100.0 * updates / total if total else 0.0,
        agreement_pct=agreement_pct,
        angle_spike_steps=angle_spikes,
        position_spike_steps=position_spikes,
        update_steps=updates,
        total_steps=total,
        angle_ratio_ge_base_pct=naive_angle_ratio_rate(angles, params.log_base),
        angle_abs_mean=float(np.mean(np.abs(angles))),
        angle_abs_max=float(np.max(np.abs(angles))),
        **breakdown_fields_from_dataframe(df, params, input_mode),
    )


def breakdown_fields_from_dataframe(
    df: pd.DataFrame,
    params: GateParams,
    input_mode: str,
) -> dict[str, float | None]:
    try:
        from test_secloc_log_base_sweep import (  # noqa: WPS433
            TrajectorySample,
            replay_gate_breakdown,
        )
    except ImportError:
        return {
            "skip_given_changed_pct": None,
            "skip_given_flat_pct": None,
            "gate_input_flat_pct": None,
        }

    angles = select_angle_series(df, input_mode)
    positions = df["position"].to_numpy(dtype=np.float64)
    targets = df["target_position"].to_numpy(dtype=np.float64)
    times = df["time"].to_numpy(dtype=np.float64)
    samples = []
    for angle, position, target, time in zip(angles, positions, targets, times):
        s = create_cartpole_state()
        s[ANGLE_IDX] = angle
        s[ANGLE_COS_IDX] = np.cos(angle)
        s[ANGLE_SIN_IDX] = np.sin(angle)
        s[POSITION_IDX] = position
        samples.append(
            TrajectorySample(time=float(time), s=s, target_position=float(target))
        )
    row = replay_gate_breakdown(
        samples,
        log_base=params.log_base,
        ref_period=params.ref_period,
        dead_ang=params.dead_ang,
        dead_pos=params.dead_pos,
    )
    return {
        "skip_given_changed_pct": row.skip_given_changed_pct,
        "skip_given_flat_pct": row.skip_given_flat_pct,
        "gate_input_flat_pct": row.flat_poll_pct,
    }


def sweep_log_base_on_trajectory(
    df: pd.DataFrame,
    *,
    params: GateParams,
    log_bases: np.ndarray,
    input_mode: str = "logged",
) -> list[tuple[float, float]]:
    rows: list[tuple[float, float]] = []
    base = GateParams(
        log_base=params.log_base,
        ref_period=params.ref_period,
        dead_ang=params.dead_ang,
        dead_pos=params.dead_pos,
    )
    for log_base in log_bases:
        _, replay = replay_secloc(
            df,
            params=GateParams(
                log_base=float(log_base),
                ref_period=base.ref_period,
                dead_ang=base.dead_ang,
                dead_pos=base.dead_pos,
            ),
            input_mode=input_mode,
        )
        rows.append((float(log_base), replay.skip_pct))
    return rows


def closest_log_base_to_target(
    sweep: list[tuple[float, float]], target_skip_pct: float
) -> tuple[float, float]:
    return min(sweep, key=lambda row: abs(row[1] - target_skip_pct))


def format_sweep_report(
    path: Path,
    sweep: list[tuple[float, float]],
    *,
    target_skip_pct: float | None,
    recorded_log_base: float | None,
    plateau_angle_pct: float,
) -> str:
    lines = [
        f"File: {path.name}",
        "  offline log_base sweep on recorded angle/position trajectory",
        f"  plateau polls (unchanged |angle| vs previous): {plateau_angle_pct:.1f}%",
        "",
        "  log_base  skip_%",
        "  --------  ------",
    ]
    for log_base, skip_pct in sweep:
        marker = ""
        if recorded_log_base is not None and abs(log_base - recorded_log_base) < 1e-6:
            marker = "  <- recorded run"
        lines.append(f"  {log_base:8.4f}  {skip_pct:6.2f}{marker}")
    if target_skip_pct is not None:
        best = closest_log_base_to_target(sweep, target_skip_pct)
        lines.extend(
            [
                "",
                f"  closest to {target_skip_pct:.0f}% skip: "
                f"log_base={best[0]:.4f} -> {best[1]:.2f}%",
            ]
        )
    return "\n".join(lines)


def plateau_angle_fraction(df: pd.DataFrame) -> float:
    magnitude = df["angle"].abs()
    return 100.0 * float((magnitude == magnitude.shift(1)).mean())


def sim_baseline_skip_pct(log_base: float) -> float | None:
    try:
        from test_secloc_log_base_sweep import (  # noqa: WPS433
            measure_skip_rate,
            record_stabilized_trajectory,
        )
    except ImportError:
        return None

    record = record_stabilized_trajectory(noise_scale=1.0, seed=123)
    if not record.stable:
        return None
    result = measure_skip_rate(record.samples, log_base=log_base)
    return result.skipped_pct


def format_replay_report(
    path: Path,
    hardware: dict,
    replay: ReplayResult,
    params: GateParams,
    sim_skip_pct: float | None,
) -> str:
    lines = [
        f"File: {path.name}",
        f"  gate: log_base={params.log_base}, ref_period={params.ref_period}, "
        f"dead_ang={params.dead_ang}, dead_pos={params.dead_pos}",
        f"  hardware skip: {hardware['skip_pct']:.2f}%  "
        f"({hardware['rows'] - hardware['updates']} skipped / {hardware['rows']} polls)",
        f"  replay ({replay.input_mode}) skip: {replay.skip_pct:.2f}%  "
        f"({replay.total_steps - replay.update_steps} skipped / {replay.total_steps} polls)",
    ]
    if replay.agreement_pct is not None:
        lines.append(f"  replay vs CSV agreement: {replay.agreement_pct:.2f}%")
    lines.extend(
        [
            f"  spike source (replay): angle={replay.angle_spike_steps}, "
            f"position={replay.position_spike_steps} "
            f"(of {replay.update_steps} updates)",
            f"  |angle| mean={replay.angle_abs_mean:.4f} rad, "
            f"max={replay.angle_abs_max:.4f} rad",
        ]
    )
    if replay.angle_ratio_ge_base_pct is not None:
        lines.append(
            f"  naive consecutive |angle| ratio >= log_base: "
            f"{replay.angle_ratio_ge_base_pct:.1f}% of steps"
        )
    if replay.skip_given_changed_pct is not None:
        lines.extend(
            [
                f"  skip|changed (gate input changed): {replay.skip_given_changed_pct:.2f}%",
                f"  skip|flat (gate input unchanged): {replay.skip_given_flat_pct:.2f}%",
                f"  gate input flat polls: {replay.gate_input_flat_pct:.2f}%",
            ]
        )
    if sim_skip_pct is not None:
        lines.append(
            f"  sim PID+noise baseline @ log_base={params.log_base}: "
            f"{sim_skip_pct:.1f}% skipped "
            f"(replay - sim = {replay.skip_pct - sim_skip_pct:+.1f} pp)"
        )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Replay CPP CSV state through SeclocGate offline (Phase 2)"
    )
    parser.add_argument("csv_files", nargs="*", help="CPP recording path(s)")
    parser.add_argument(
        "--latest",
        nargs="?",
        const=1,
        type=int,
        metavar="N",
        help="Replay the N most recent CPP recordings",
    )
    parser.add_argument(
        "--input",
        choices=("logged", "angle_raw"),
        default="logged",
        help="Angle source for gate input (default: logged CSV angle column)",
    )
    parser.add_argument(
        "--log-base",
        type=float,
        default=None,
        help="Override log_base instead of reading CSV header",
    )
    parser.add_argument(
        "--sim-baseline",
        action="store_true",
        help="Also report sim PID+noise skip %% at the same log_base",
    )
    parser.add_argument(
        "--sweep-log-base",
        action="store_true",
        help="Sweep log_base on the recorded trajectory (offline skip curve)",
    )
    parser.add_argument(
        "--target-skip",
        type=float,
        default=10.0,
        help="Target skip %% for closest-match line in sweep output (default: 10)",
    )
    args = parser.parse_args(argv)

    paths: list[Path] = [resolve_recording_path(p) for p in args.csv_files]
    if args.latest is not None:
        if not RECORDINGS_DIR.is_dir():
            print(f"No recordings directory: {RECORDINGS_DIR}", file=sys.stderr)
            return 1
        paths.extend(latest_recordings(args.latest))

    if not paths:
        parser.print_help()
        return 1

    sim_cache: dict[float, float | None] = {}
    exit_code = 0
    for path in paths:
        if not path.is_file():
            print(f"Missing file: {path}", file=sys.stderr)
            exit_code = 1
            continue
        try:
            meta = parse_cpp_header(path)
            df = load_cpp_dataframe(path)
            params = gate_params_from_args(meta, args)
            if args.sweep_log_base:
                log_bases = np.round(np.arange(1.00, 1.151, 0.005), 3)
                sweep = sweep_log_base_on_trajectory(
                    df,
                    params=params,
                    log_bases=log_bases,
                    input_mode=args.input,
                )
                print(
                    format_sweep_report(
                        path,
                        sweep,
                        target_skip_pct=args.target_skip,
                        recorded_log_base=params.log_base,
                        plateau_angle_pct=plateau_angle_fraction(df),
                    )
                )
                print()
                continue

            hardware = skip_rate_from_csv(path)
            _, replay = replay_secloc(df, params=params, input_mode=args.input)

            sim_skip_pct = None
            if args.sim_baseline:
                if params.log_base not in sim_cache:
                    sim_cache[params.log_base] = sim_baseline_skip_pct(params.log_base)
                sim_skip_pct = sim_cache[params.log_base]

            print(format_replay_report(path, hardware, replay, params, sim_skip_pct))
            print()
        except Exception as exc:
            print(f"Error replaying {path}: {exc}", file=sys.stderr)
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
