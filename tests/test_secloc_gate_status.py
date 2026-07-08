"""Unit tests for SeclocGate poll-level status reporting."""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "Driver"))
sys.path.insert(1, str(REPO_ROOT / "Driver" / "CartPoleSimulation"))
sys.path.insert(2, str(REPO_ROOT / "Driver" / "CartPoleSimulation" / "SI_Toolkit" / "src"))
os.chdir(REPO_ROOT / "Driver")

from CartPoleSimulation.CartPole.state_utilities import ANGLE_IDX, POSITION_IDX, create_cartpole_state
from Control_Toolkit_ASF.Controllers.secloc_gate import SeclocGate


def _run_gate_polls(gate, angle, positions, target_position=0.0, dt=0.025):
    s = create_cartpole_state()
    s[ANGLE_IDX] = angle
    time = 0.0
    for position in positions:
        s[POSITION_IDX] = position
        gate.should_sample(s, target_position, time=time, time_difference=dt)
        time += dt


def test_secloc_status_reports_poll_breakdown():
    gate = SeclocGate(
        log_base=1.5,
        ref_period=0.02,
        dead_ang=0.001,
        dead_pos=0.001,
        status_window_size=20,
    )

    # Same angle, position creeps: angle flat ~100%, position flat ~0%.
    _run_gate_polls(
        gate,
        angle=0.01,
        positions=[0.0] + [0.0005 * i for i in range(1, 25)],
    )

    assert len(gate._active_poll_stats()) == 24
    assert gate._poll_stat_percentage(lambda stat: stat.ang_unchanged) == pytest.approx(100.0, abs=0.1)
    assert gate._poll_stat_percentage(lambda stat: stat.pos_unchanged) == pytest.approx(0.0, abs=0.1)

    status = gate.get_status()
    assert "angle not changed 100.0%" in status
    assert "position not changed 0.0%" in status
    assert "skipped (angle or position changed)" in status
    assert "skipped (angle and position changed)" in status


def test_secloc_status_counts_both_changed_skips():
    gate = SeclocGate(
        log_base=1.5,
        ref_period=0.02,
        dead_ang=0.001,
        dead_pos=0.001,
        status_window_size=10,
    )

    # Small steps on both axes stay below log_base threshold -> skipped with both changed.
    angles = [0.01 + 0.0001 * i for i in range(12)]
    positions = [0.0 + 0.0001 * i for i in range(12)]
    s = create_cartpole_state()
    time = 0.0
    dt = 0.025
    for angle, position in zip(angles, positions):
        s[ANGLE_IDX] = angle
        s[POSITION_IDX] = position
        gate.should_sample(s, 0.0, time=time, time_difference=dt)
        time += dt

    skip_and_changed_pct = gate._poll_stat_percentage(
        lambda stat: (not stat.ang_unchanged and not stat.pos_unchanged) and stat.skipped
    )
    assert skip_and_changed_pct == pytest.approx(100.0, abs=0.1)


def test_secloc_poll_stats_use_time_window():
    gate = SeclocGate(
        log_base=1.5,
        ref_period=0.02,
        dead_ang=0.001,
        dead_pos=0.001,
        poll_stats_window_s=5.0,
    )

    s = create_cartpole_state()
    s[ANGLE_IDX] = 0.01
    dt = 0.025

    # First 4 s: angle flat, position changing.
    time = 0.0
    for i in range(int(4.0 / dt)):
        s[POSITION_IDX] = 0.0005 * i
        gate.should_sample(s, 0.0, time=time, time_difference=dt)
        time += dt

    assert gate._poll_stat_percentage(lambda stat: stat.ang_unchanged) == pytest.approx(100.0, abs=0.1)

    # Next 2 s: angle also changing -> angle flat drops once old 5 s window expires.
    for i in range(int(2.0 / dt)):
        s[ANGLE_IDX] = 0.01 + 0.001 * i
        s[POSITION_IDX] = 0.01 + 0.0005 * i
        gate.should_sample(s, 0.0, time=time, time_difference=dt)
        time += dt

    assert gate._poll_stat_percentage(lambda stat: stat.ang_unchanged) < 95.0
    assert len(gate._active_poll_stats()) <= int(5.0 / dt) + 2


def test_peek_should_sample_does_not_mutate_gate_state():
    gate = SeclocGate(
        log_base=1.5,
        ref_period=0.02,
        dead_ang=0.001,
        dead_pos=0.001,
    )
    s = create_cartpole_state()
    s[ANGLE_IDX] = 0.02
    s[POSITION_IDX] = 0.01

    gate.logic.time_last = 0.0
    gate.logic.ang_last_shift = 0.01
    gate.logic.pos_last_shift = 0.01
    before = (gate.logic.ang_last_shift, gate.logic.pos_last_shift, gate.logic.time_last)

    would_update = gate.peek_would_update(s, 0.0, time=0.02, time_difference=0.02)
    after_peek = (gate.logic.ang_last_shift, gate.logic.pos_last_shift, gate.logic.time_last)

    assert would_update is True
    assert after_peek == before
    assert gate.last_gate_evaluated is True
    assert gate.last_gate_would_update is True

    spike = gate.should_sample(s, 0.0, time=0.02, time_difference=0.02)
    assert spike is True
    assert gate.logic.time_last == 0.02
