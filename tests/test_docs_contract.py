"""User-facing docs must stay aligned with firmware, driver, and each other."""
from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

DOC_PATHS = [
    REPO / "README.md",
    REPO / "Docs" / "operating.md",
    REPO / "Docs" / "pc-driver.md",
    REPO / "Docs" / "hardware.md",
    REPO / "Docs" / "calibration.md",
    REPO / "Docs" / "firmware-and-flash.md",
    REPO / "Docs" / "architecture.md",
    REPO / "Firmware" / "README.md",
    REPO / "examples" / "models" / "README.md",
    REPO / "Driver" / "DataAnalysis" / "MotorAndCartFriction" / "README.md",
    REPO / "examples" / "models" / "adaptive-quant-lstm-2025-06-01" / "README.md",
    REPO / "Driver" / "CartPoleSimulation" / "README.md",
    REPO / "Driver" / "CartPoleSimulation" / "Control_Toolkit_ASF" / "CONTROLLER_DESCRIPTION.md",
]

FORBIDDEN = ("measure.py", "cartpole_model.py", "Shift+K")


def _read(path: Path) -> str:
    assert path.is_file(), f"missing doc: {path.relative_to(REPO)}"
    return path.read_text()


def test_operating_doc_covers_show_mux_and_safety():
    text = _read(REPO / "Docs" / "operating.md")
    assert "SW0" in text and "AMP RPGD" in text
    assert "Stall cut" in text or "stall" in text.lower()
    assert "Dense-7IN-32H1-32H2-1OUT-8" in text


def test_pc_driver_doc_covers_control_and_protocols():
    text = _read(REPO / "Docs" / "pc-driver.md")
    assert "control.py" in text
    assert "`m`" in text and "`n`" in text and "`N`" in text
    assert "Run LivePlotter.py" in text


def test_readme_links_cartpole_simulation_readme():
    readme = _read(REPO / "README.md")
    assert "Driver/CartPoleSimulation/README.md" in readme


def test_no_stale_strings_in_user_facing_docs():
    for path in DOC_PATHS:
        text = _read(path)
        for bad in FORBIDDEN:
            assert bad not in text, f"{path.relative_to(REPO)} still mentions {bad!r}"


def test_secloc_doc_has_branch_banner_and_k_not_shift_k():
    text = _read(REPO / "Docs" / "SecLoc_Experiment_Platform.md")
    assert "Not the Development Zybo show default" in text
    assert "Shift+K" not in text
    assert "`K`" in text
