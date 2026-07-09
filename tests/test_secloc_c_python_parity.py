"""C vs Python parity tests for the Secloc gate.

Compiles Firmware/Src/General/secloc.c on the PC and drives it with the same
input sequences as the Python SeclocGate (Control_Toolkit_ASF), asserting the
two implementations take identical update/skip decisions and keep identical
reference state.

With ref_period_ticks = 0 every call is a pure log_base gate decision on both
sides. With ref_period_ticks > 0 both sides throttle in integer control loop
iterations (ticks of the same 5 ms time quantum) on monotonic timestamps.
"""
from __future__ import annotations

import ctypes
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_GENERAL = REPO_ROOT / "Firmware" / "Src" / "General"
RECORDINGS_DIR = REPO_ROOT / "Driver" / "ExperimentRecordings"

sys.path.insert(0, str(REPO_ROOT / "Driver"))
sys.path.insert(1, str(REPO_ROOT / "Driver" / "CartPoleSimulation"))
sys.path.insert(2, str(REPO_ROOT / "Driver" / "CartPoleSimulation" / "SI_Toolkit" / "src"))
os.chdir(REPO_ROOT / "Driver")

from CartPoleSimulation.CartPole.state_utilities import (  # noqa: E402
    ANGLE_IDX,
    POSITION_IDX,
    create_cartpole_state,
)
from Control_Toolkit_ASF.Controllers.secloc_gate import SeclocLogic  # noqa: E402

# float32-representable so the C (float) and Python (double) threshold constants
# are the same number.
LOG_BASE = float(np.float32(1.05))
DEAD_ANG = float(np.float32(0.001))
DEAD_POS = float(np.float32(0.001))
TIME_QUANTUM_S = 0.005


class CSeclocConfig(ctypes.Structure):
    _fields_ = [
        ("log_base", ctypes.c_float),
        ("ref_period_ticks", ctypes.c_int32),
        ("ang_dead_band", ctypes.c_float),
        ("pos_dead_band", ctypes.c_float),
        ("time_quantum_s", ctypes.c_float),
    ]


class CSeclocState(ctypes.Structure):
    _fields_ = [
        ("ang_last_shift", ctypes.c_float),
        ("pos_last_shift", ctypes.c_float),
        ("last_Q", ctypes.c_float),
        ("has_init", ctypes.c_uint8),
        ("time_last", ctypes.c_float),
        ("tick_last", ctypes.c_int32),
    ]


@pytest.fixture(scope="module")
def secloc_lib(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("secloc_c")
    library = tmp_path / "secloc.so"
    subprocess.run(
        [
            "gcc",
            "-shared",
            "-fPIC",
            "-std=c99",
            "-o",
            str(library),
            str(FIRMWARE_GENERAL / "secloc.c"),
            "-I",
            str(FIRMWARE_GENERAL),
            "-lm",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    lib = ctypes.CDLL(str(library))
    lib.secloc_reset.argtypes = [ctypes.POINTER(CSeclocState)]
    lib.secloc_reset.restype = None
    lib.secloc_should_sample.argtypes = [
        ctypes.POINTER(CSeclocState),
        ctypes.POINTER(CSeclocConfig),
        ctypes.c_float,  # p
        ctypes.c_float,  # pd
        ctypes.c_float,  # a
        ctypes.c_float,  # ad
        ctypes.c_float,  # tp
        ctypes.c_float,  # te
        ctypes.c_float,  # time
    ]
    lib.secloc_should_sample.restype = ctypes.c_int
    return lib


class GatePair:
    """A C gate and a Python gate stepped in lockstep."""

    def __init__(self, lib, ref_period_ticks=0):
        self.lib = lib
        self.ref_period_ticks = ref_period_ticks
        self.config = CSeclocConfig(
            log_base=LOG_BASE,
            ref_period_ticks=ref_period_ticks,
            ang_dead_band=DEAD_ANG,
            pos_dead_band=DEAD_POS,
            time_quantum_s=TIME_QUANTUM_S if ref_period_ticks > 0 else 0.0,
        )
        self.c_state = CSeclocState()
        lib.secloc_reset(ctypes.byref(self.c_state))

        self.py = SeclocLogic(
            log_base=LOG_BASE,
            ref_period_ticks=ref_period_ticks,
            dead_ang=DEAD_ANG,
            dead_pos=DEAD_POS,
        )
        if ref_period_ticks > 0:
            self.py.set_time_quantum(TIME_QUANTUM_S)
        self.py.reset()

        self.s = create_cartpole_state().astype(np.float64)
        self.time = 0.0

    def step(self, angle, position, target_position, target_equilibrium, time=None):
        if time is None:
            time = self.time
            self.time += TIME_QUANTUM_S

        # Present the exact same float32 values to both implementations.
        angle = float(np.float32(angle))
        position = float(np.float32(position))
        target_position = float(np.float32(target_position))
        target_equilibrium = float(np.float32(target_equilibrium))
        time = float(np.float32(time))

        c_decision = self.lib.secloc_should_sample(
            ctypes.byref(self.c_state),
            ctypes.byref(self.config),
            position,
            0.0,
            angle,
            0.0,
            target_position,
            target_equilibrium,
            time,
        )

        self.s[ANGLE_IDX] = angle
        self.s[POSITION_IDX] = position
        py_decision = self.py.should_sample(
            self.s,
            target_position,
            time=time,
            target_equilibrium=target_equilibrium,
        )
        return bool(c_decision), bool(py_decision)

    def assert_states_close(self):
        np.testing.assert_allclose(
            self.c_state.ang_last_shift, self.py.ang_last_shift, rtol=2e-6, atol=2e-7
        )
        np.testing.assert_allclose(
            self.c_state.pos_last_shift, self.py.pos_last_shift, rtol=2e-6, atol=2e-7
        )


def run_sequence(pair, sequence):
    for step_idx, (angle, position, target_position, target_equilibrium) in enumerate(sequence):
        c_dec, py_dec = pair.step(angle, position, target_position, target_equilibrium)
        assert c_dec == py_dec, (
            f"step {step_idx}: C={c_dec} Python={py_dec} for "
            f"angle={angle} position={position} tp={target_position} te={target_equilibrium} "
            f"(C refs ang={pair.c_state.ang_last_shift} pos={pair.c_state.pos_last_shift}, "
            f"Py refs ang={pair.py.ang_last_shift} pos={pair.py.pos_last_shift})"
        )
        pair.assert_states_close()


def test_position_check_is_independent(secloc_lib):
    """Position alone must trigger updates even with the angle far from its dead band."""
    pair = GatePair(secloc_lib)
    sequence = [
        # Angle constant at 0.5 rad (well above dead band), position ramps.
        (0.5, 0.00, 0.0, 1.0),   # first call: both refs tiny -> fires
        (0.5, 0.02, 0.0, 1.0),   # position 0.02 vs tiny ref -> fires on position
        (0.5, 0.0202, 0.0, 1.0),  # +1% -> below 5% threshold -> skip
        (0.5, 0.03, 0.0, 1.0),   # +48% -> fires on position
        (0.5, 0.0301, 0.0, 1.0),  # small -> skip
    ]
    run_sequence(pair, sequence)
    decisions = [pair.step(0.5, p, 0.0, 1.0) for p in (0.05, 0.0501)]
    assert decisions[0] == (True, True)
    assert decisions[1] == (False, False)


def test_angle_shift_tracks_down_equilibrium(secloc_lib):
    """With target down, small wiggles around hanging must fire the gate."""
    pair = GatePair(secloc_lib)
    pi = float(np.pi)
    # Pole hanging (angle ~ +/-pi), target down: shift = pi - |angle| is small,
    # so the multiplicative criterion is sensitive - like upright with target up.
    sequence = [
        (pi - 0.05, 0.0, 0.0, -1.0),  # first call fires, ref = 0.05
        (pi - 0.051, 0.0, 0.0, -1.0),  # 2% change -> skip
        (pi - 0.06, 0.0, 0.0, -1.0),  # 20% change -> fire
        (-(pi - 0.06), 0.0, 0.0, -1.0),  # wrap sign flip, same |shift| -> skip
        (pi - 0.03, 0.0, 0.0, -1.0),  # factor 2 -> fire
    ]
    run_sequence(pair, sequence)


def test_equilibrium_flip_fires_gate(secloc_lib):
    """Flipping the target equilibrium re-frames the angle shift and fires promptly."""
    pair = GatePair(secloc_lib)
    pi = float(np.pi)
    run_sequence(pair, [(pi - 0.05, 0.0, 0.0, -1.0), (pi - 0.0505, 0.0, 0.0, -1.0)])
    # Same physical angle, target flips down -> up: shift jumps 0.05 -> pi - 0.05.
    c_dec, py_dec = pair.step(pi - 0.05, 0.0, 0.0, 1.0)
    assert c_dec == py_dec == True  # noqa: E712
    pair.assert_states_close()


def test_random_trajectory_parity_with_ref_period(secloc_lib):
    """Long randomized run with the deployed throttle (4 ticks = 20 ms):
    decisions must match exactly."""
    rng = np.random.default_rng(20260709)
    pair = GatePair(secloc_lib, ref_period_ticks=4)

    n_steps = 5000
    angle = 0.0
    position = 0.0
    for step_idx in range(n_steps):
        regime = (step_idx // 500) % 4
        if regime == 0:
            angle = 0.02 * rng.standard_normal()
            te = 1.0
        elif regime == 1:
            angle = float(np.pi) * np.sin(0.01 * step_idx) + 0.1 * rng.standard_normal()
            te = 1.0
        elif regime == 2:
            sign = 1.0 if rng.random() < 0.5 else -1.0
            angle = sign * (float(np.pi) - abs(0.05 * rng.standard_normal()))
            te = -1.0
        else:
            angle = float(np.pi) * np.sin(0.013 * step_idx) + 0.1 * rng.standard_normal()
            te = -1.0
        angle = float(np.clip(angle, -np.pi, np.pi))
        position += 0.002 * rng.standard_normal()
        position = float(np.clip(position, -0.198, 0.198))
        tp = 0.0 if step_idx < n_steps // 2 else 0.05

        c_dec, py_dec = pair.step(angle, position, tp, te)
        assert c_dec == py_dec, (
            f"step {step_idx}: C={c_dec} Python={py_dec} "
            f"angle={angle} position={position} tp={tp} te={te}"
        )
    pair.assert_states_close()


def test_random_trajectory_parity(secloc_lib):
    """Long randomized run over both equilibria: decisions must match exactly."""
    rng = np.random.default_rng(20260709)
    pair = GatePair(secloc_lib)

    n_steps = 20000
    angle = 0.0
    position = 0.0
    for step_idx in range(n_steps):
        # Piecewise regimes: upright hold, hanging hold, swings.
        regime = (step_idx // 2000) % 4
        if regime == 0:      # upright wiggle, target up
            angle = 0.02 * rng.standard_normal()
            te = 1.0
        elif regime == 1:    # full swings, target up
            angle = float(np.pi) * np.sin(0.01 * step_idx) + 0.1 * rng.standard_normal()
            te = 1.0
        elif regime == 2:    # hanging wiggle, target down
            sign = 1.0 if rng.random() < 0.5 else -1.0
            angle = sign * (float(np.pi) - abs(0.05 * rng.standard_normal()))
            te = -1.0
        else:                # swings, target down
            angle = float(np.pi) * np.sin(0.013 * step_idx) + 0.1 * rng.standard_normal()
            te = -1.0
        angle = float(np.clip(angle, -np.pi, np.pi))
        position += 0.002 * rng.standard_normal()
        position = float(np.clip(position, -0.198, 0.198))
        tp = 0.0 if step_idx < n_steps // 2 else 0.05

        c_dec, py_dec = pair.step(angle, position, tp, te)
        assert c_dec == py_dec, (
            f"step {step_idx}: C={c_dec} Python={py_dec} "
            f"angle={angle} position={position} tp={tp} te={te}"
        )
    pair.assert_states_close()


def _latest_recording_with_columns() -> Path | None:
    if not RECORDINGS_DIR.is_dir():
        return None
    for path in sorted(RECORDINGS_DIR.glob("CPP_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            import pandas as pd

            header = pd.read_csv(path, comment="#", nrows=0)
        except Exception:
            continue
        if {"angle", "position", "target_position", "target_equilibrium"} <= set(header.columns):
            return path
    return None


def test_recorded_trajectory_parity(secloc_lib):
    """Replay a real hardware recording through both gates: decisions must match."""
    recording = _latest_recording_with_columns()
    if recording is None:
        pytest.skip("no experiment recording with the required columns available")

    import pandas as pd

    df = pd.read_csv(recording, comment="#")
    pair = GatePair(secloc_lib)
    mismatches = 0
    for angle, position, tp, te in zip(
        df["angle"].to_numpy(),
        df["position"].to_numpy(),
        df["target_position"].to_numpy(),
        df["target_equilibrium"].to_numpy(),
    ):
        c_dec, py_dec = pair.step(angle, position, tp, te)
        mismatches += int(c_dec != py_dec)
    assert mismatches == 0, f"{mismatches} decision mismatches over {len(df)} rows of {recording.name}"
