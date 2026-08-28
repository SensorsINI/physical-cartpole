"""CMD_SET_SECLOC_CONFIG is not sent on a normal Development home run."""
from __future__ import annotations

import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "Driver"))
sys.path.insert(1, str(REPO_ROOT / "Driver" / "CartPoleSimulation"))
os.chdir(REPO_ROOT / "Driver")

from globals import should_push_chip_secloc_config


def test_home_run_does_not_push_chip_secloc_config():
    assert should_push_chip_secloc_config(False, False) is False


def test_pc_secloc_opt_in_pushes_chip_config():
    assert should_push_chip_secloc_config(True, False) is True


def test_chip_secloc_opt_in_pushes_chip_config():
    assert should_push_chip_secloc_config(False, True) is True


def test_default_globals_do_not_push():
    from globals import USE_CHIP_SECLOC, USE_SECLOC

    assert USE_SECLOC is False
    assert USE_CHIP_SECLOC is False
    assert should_push_chip_secloc_config() is False
