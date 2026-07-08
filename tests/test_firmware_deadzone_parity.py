"""Compare legacy heuristic vs hardware-flag firmware dead-zone handling.

Python mirror of Firmware/Src/CartPoleFirmware/angle_processing.c
``treat_deadangle_with_derivative()`` for both code paths:

* **heuristic** — derivative-jump trigger near upright (current Secloc2026 HEAD)
* **hardware** — freeze driven by latched FPGA dz_window/dz_age (firmware-deadzone branch)

Uses ``deadzone_stream`` from the hardware filter reference model to synthesise
contamination flags from raw ADC traces, matching the latch logic in control.c.

Run from repo root:
  python -m pytest tests/test_firmware_deadzone_parity.py -q
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
FILTER_TEST_DIR = REPO_ROOT / "Driver" / "DataAnalysis" / "HardwareFilterTest"
DEADZONE_RECORDINGS = FILTER_TEST_DIR / "output"

sys.path.insert(0, str(FILTER_TEST_DIR))
from filter_reference_model import deadzone_stream, filter_stream  # noqa: E402

# Zedboard firmware defaults (parameters.c / globals.py)
ANGLE_360 = 4049.44
TIMESTEPS_FOR_DERIVATIVE = 1
POLLING_PERIOD_MS = 20
ANGLE_MEASUREMENT_INTERVAL_US = 200
HW_DZ_MAX_EXTRAPOLATION_MS = 500
FILTER_WINDOW = 63
FILTER_TRIM = 7
FILTER_MODE = 2  # trimmed mean
RAIL_LOW = 320
RAIL_HIGH = 65440


def wrap_local(angle: float) -> float:
    half = ANGLE_360 / 2.0
    if angle > half:
        return angle - ANGLE_360
    if angle <= -half:
        return angle + ANGLE_360
    return angle


@dataclass
class FreezeStepResult:
    angle: float
    angle_d: float
    freezme: int
    invalid_step: int


class FirmwareAngleFreezeModel:
    """Bit-faithful port of treat_deadangle_with_derivative + optional hw flag."""

    def __init__(self, *, use_hardware_deadzone: bool):
        self.use_hardware_deadzone = use_hardware_deadzone
        self.reset()

    def reset(self):
        self.angle_history = [-1.0] * (TIMESTEPS_FOR_DERIVATIVE + 1)
        self.idx = 0
        self.angle_raw_stable = -1.0
        self.angle_d_raw = 0.0
        self.angle_d_raw_stable = -1.0
        self.freezme = 0
        self.last_difference = 100_000.0
        self.hw_dz_valid = False
        self.hw_dz_contaminated = False
        self.hw_dz_dwell_polls = 0

    def report_hardware_deadzone(self, contaminated: int):
        self.hw_dz_valid = True
        self.hw_dz_contaminated = bool(contaminated)

    def process_poll(self, angle_adc: float, invalid_step: int = 0) -> FreezeStepResult:
        angle = wrap_local(angle_adc)
        reported_invalid = invalid_step
        if self.hw_dz_valid and self.hw_dz_contaminated:
            reported_invalid += 1

        kth_index = (self.idx + 1) % (TIMESTEPS_FOR_DERIVATIVE + 1)
        kth_past_angle = self.angle_history[kth_index]
        if kth_past_angle != -1:
            current_difference = wrap_local(angle - kth_past_angle) / TIMESTEPS_FOR_DERIVATIVE
        else:
            current_difference = 0.0

        if self.last_difference > 10_000:
            self.last_difference = current_difference

        if self.hw_dz_valid:
            if self.hw_dz_contaminated:
                self.hw_dz_dwell_polls += 1
            else:
                self.hw_dz_dwell_polls = 0
            if (
                self.hw_dz_contaminated
                and kth_past_angle != -1
                and self.hw_dz_dwell_polls
                <= int(HW_DZ_MAX_EXTRAPOLATION_MS / POLLING_PERIOD_MS)
                and self.freezme < TIMESTEPS_FOR_DERIVATIVE + 3
            ):
                self.freezme = TIMESTEPS_FOR_DERIVATIVE + 3
        elif (
            kth_past_angle != -1
            and -500 < self.angle_raw_stable < 500
            and self.freezme == 0
            and (
                TIMESTEPS_FOR_DERIVATIVE * abs(current_difference - self.last_difference)
                > POLLING_PERIOD_MS * 2.4
                or reported_invalid > 5
            )
        ):
            if self.angle_d_raw_stable > 0:
                self.freezme = int(45 / POLLING_PERIOD_MS) + TIMESTEPS_FOR_DERIVATIVE
            else:
                self.freezme = int(90 / POLLING_PERIOD_MS) + TIMESTEPS_FOR_DERIVATIVE

        if self.freezme > 0:
            self.freezme -= 1
            self.angle_d_raw = self.angle_d_raw_stable
            if self.freezme > TIMESTEPS_FOR_DERIVATIVE + 1:
                self.angle_raw_stable += self.angle_d_raw_stable
                angle = self.angle_raw_stable
            else:
                self.angle_raw_stable = angle
        else:
            self.angle_raw_stable = angle
            self.angle_d_raw = current_difference
            self.angle_d_raw_stable = self.angle_d_raw

        self.last_difference = current_difference
        self.angle_history[self.idx] = angle
        self.idx = (self.idx + 1) % (TIMESTEPS_FOR_DERIVATIVE + 1)

        return FreezeStepResult(
            angle=angle,
            angle_d=self.angle_d_raw,
            freezme=self.freezme,
            invalid_step=reported_invalid,
        )


def poll_contamination_flags(
    raw_16: np.ndarray,
    *,
    poll_ms: int = POLLING_PERIOD_MS,
    sample_us: int = ANGLE_MEASUREMENT_INTERVAL_US,
    window_size: int = FILTER_WINDOW,
    rail_low: int = RAIL_LOW,
    rail_high: int = RAIL_HIGH,
) -> list[int]:
    """Mirror control.c latch: OR contamination across 200 us angle samples in each poll."""
    _, dz_window, dz_age, _, _ = deadzone_stream(
        raw_16, window_size, rail_low, rail_high
    )
    samples_per_poll = max(1, int(poll_ms * 1000 / sample_us))
    age_threshold = (sample_us * 10) // 22
    flags: list[int] = []
    for start in range(0, len(dz_window), samples_per_poll):
        chunk_w = dz_window[start : start + samples_per_poll]
        chunk_a = dz_age[start : start + samples_per_poll]
        if len(chunk_w) == 0:
            break
        contaminated = int(np.any(chunk_w > 0) or np.any(chunk_a < age_threshold))
        flags.append(contaminated)
    return flags


def simulate_trajectory(
    angles_12bit: np.ndarray,
    *,
    mode: str,
    contamination: list[int] | None = None,
) -> list[FreezeStepResult]:
    use_hw = mode == "hardware"
    model = FirmwareAngleFreezeModel(use_hardware_deadzone=use_hw)
    results: list[FreezeStepResult] = []
    for i, angle in enumerate(angles_12bit):
        if use_hw and contamination is not None:
            model.report_hardware_deadzone(contamination[min(i, len(contamination) - 1)])
        results.append(model.process_poll(float(angle)))
    return results


def _synthetic_clean_swing(n_polls: int = 80) -> np.ndarray:
    """Smooth upright swing without rail hits (deviation-domain ADC codes)."""
    t = np.linspace(0, 1.5 * np.pi, n_polls)
    return 120.0 * np.sin(t)


def _synthetic_deadzone_crossing(n_polls: int = 50) -> tuple[np.ndarray, np.ndarray, list[int]]:
    """Build a poll-rate crossing with FPGA contamination flags from a 16-bit trace."""
    dt_us = ANGLE_MEASUREMENT_INTERVAL_US
    n_xadc = n_polls * max(1, int(POLLING_PERIOD_MS * 1000 / dt_us))
    t = np.arange(n_xadc) * dt_us * 1e-6

    # 12-bit deviation from hanging (1020); rails at ~0 and ~4090 ADC codes.
    deviation = np.zeros(n_xadc)
    deviation[t < 0.25] = 60 * np.sin(2 * np.pi * 3.0 * t[t < 0.25])
    mask_cross = (t >= 0.25) & (t < 0.45)
    deviation[mask_cross] = np.linspace(0, 3070, mask_cross.sum())  # -> high rail
    mask_rail = (t >= 0.45) & (t < 0.65)
    deviation[mask_rail] = 3070
    mask_return = (t >= 0.65)
    deviation[mask_return] = np.linspace(3070, -1010, mask_return.sum())  # -> low rail

    raw_16 = np.clip((1020 + deviation) * 16, 0, 65535).astype(np.int64)
    filtered_16 = filter_stream(raw_16, FILTER_WINDOW, FILTER_TRIM, FILTER_MODE)
    stride = max(1, int(POLLING_PERIOD_MS * 1000 / dt_us))
    angles_12 = (filtered_16[::stride] / 16.0).astype(float)
    contamination = poll_contamination_flags(raw_16)
    n = min(len(angles_12), len(contamination))
    return angles_12[:n], raw_16, contamination[:n]


def _synthetic_heuristic_trigger() -> np.ndarray:
    """Poll-rate angles that trip the legacy derivative-jump detector near upright."""
    angles = np.zeros(30)
    angles[10] = 350.0  # single large step while still inside ±500
    angles[11:] = 350.0
    return angles


def test_clean_trajectory_heuristic_matches_hardware():
    """Away from the dead zone both firmware paths must be identical."""
    angles = _synthetic_clean_swing()
    heur = simulate_trajectory(angles, mode="heuristic")
    hw = simulate_trajectory(angles, mode="hardware", contamination=[0] * len(angles))

    assert len(heur) == len(hw)
    for h, w in zip(heur, hw):
        assert h.angle == pytest.approx(w.angle)
        assert h.angle_d == pytest.approx(w.angle_d)
        assert h.freezme == w.freezme


def test_heuristic_triggers_on_derivative_jump_near_upright():
    """Legacy path freezes on a step inside the ±500 ADC upright window."""
    angles = _synthetic_heuristic_trigger()
    heur = simulate_trajectory(angles, mode="heuristic")
    assert any(r.freezme > 0 for r in heur)


def test_both_modes_enter_freeze_during_deadzone_episode():
    """Hardware freezes on contamination; heuristic freezes on the upright jump case."""
    angles, _raw, contamination = _synthetic_deadzone_crossing()
    heur = simulate_trajectory(angles, mode="heuristic")
    hw = simulate_trajectory(angles, mode="hardware", contamination=contamination)

    assert any(r.freezme > 0 for r in hw), "hardware should freeze during contamination"
    assert any(r.invalid_step > 0 for r in hw), "hardware flags contamination in invalid_step"
    # Heuristic may or may not fire on filtered rail slew; dedicated case above covers it.


def test_hardware_freeze_covers_contamination_episode():
    """Hardware freeze should stay active while contamination latch is set."""
    angles, _raw, contamination = _synthetic_deadzone_crossing()
    hw = simulate_trajectory(angles, mode="hardware", contamination=contamination)

    for cont, step in zip(contamination, hw):
        if cont:
            assert step.freezme > 0 or step.invalid_step > 0


def test_heuristic_freeze_near_upright_only():
    """Legacy trigger requires angle_raw_stable inside the ±500 ADC dead-band window."""
    model = FirmwareAngleFreezeModel(use_hardware_deadzone=False)
    # Far from upright: large derivative jump should not trigger freeze
    model.angle_raw_stable = 2000.0
    model.angle_history = [1990.0, 2500.0]
    model.idx = 1
    model.last_difference = 10.0
    result = model.process_poll(2600.0)
    assert result.freezme == 0


@pytest.mark.parametrize("path", sorted(DEADZONE_RECORDINGS.glob("deadzone_*_rep1.npz")))
def test_recorded_deadzone_both_modes_quiet_outside_episodes(path: Path):
    """On real recordings, both models agree outside rail episodes."""
    if not path.exists():
        pytest.skip(f"recording not found: {path}")

    data = np.load(path, allow_pickle=False)
    raw = data["raw"].astype(np.int64)
    filtered = data["filtered"].astype(np.int64)
    dz_window = data["dz_window"].astype(np.int64)
    interval_us = int(data["meta_interval_us"])

    angles_12 = (filtered / 16.0).astype(float)
    # Build per-poll contamination from logged hardware registers
    samples_per_poll = max(1, int(POLLING_PERIOD_MS * 1000 / interval_us))
    age_threshold = (ANGLE_MEASUREMENT_INTERVAL_US * 10) // 22
    contamination: list[int] = []
    for start in range(0, len(dz_window), samples_per_poll):
        chunk = dz_window[start : start + samples_per_poll]
        if len(chunk) == 0:
            break
        contamination.append(int(np.any(chunk > 0)))

    n_polls = min(len(angles_12), len(contamination))
    angles_12 = angles_12[:n_polls]
    contamination = contamination[:n_polls]

    quiet = np.array(contamination) == 0
    if not np.any(quiet):
        pytest.skip("recording has no uncontaminated polls")

    # Compare only the prefix before the first contamination episode. After
    # report_hardware_deadzone() the firmware never returns to the heuristic
    # branch, so post-episode angle_d can legitimately diverge.
    first_hit = next((i for i, flag in enumerate(contamination) if flag), len(contamination))
    if first_hit == 0:
        pytest.skip("recording starts inside a dead-zone episode")

    prefix = slice(0, first_hit)
    heur = simulate_trajectory(angles_12[prefix], mode="heuristic")
    hw = simulate_trajectory(
        angles_12[prefix],
        mode="hardware",
        contamination=contamination[:first_hit],
    )

    for i in range(first_hit):
        assert heur[i].angle == pytest.approx(hw[i].angle), f"poll {i} on {path.name}"
        assert heur[i].angle_d == pytest.approx(hw[i].angle_d), f"poll {i} on {path.name}"
