from types import SimpleNamespace

from Driver.DriverFunctions.main_logging_manager import MainLoggingManager
from SI_Toolkit.Functions.FunctionalDict import FunctionalDict


def test_controller_csv_schema_is_stable_when_firmware_control_toggles():
    driver = SimpleNamespace(
        firmwareControl=False,
        split_control=SimpleNamespace(last_applied_now=False, is_busy=False),
        secloc_skipped_update_chip=0,
        secloc_gate_skipped_chip=0,
        secloc_pl_used_chip=0,
        secloc_pl_fault_chip=0,
        controller=SimpleNamespace(
            controller_data_for_csv=FunctionalDict({
                "LSTM_state": lambda: [0.0],
            })
        ),
    )
    manager = MainLoggingManager.__new__(MainLoggingManager)
    manager.driver = driver
    manager.data_to_save_controller = FunctionalDict({
        "controller_update_applied": lambda: 0,
        "split_control_busy": lambda: 0,
    })

    idle_keys = set(manager._controller_data_to_save().keys())
    driver.firmwareControl = True
    firmware_keys = set(manager._controller_data_to_save().keys())

    assert firmware_keys == idle_keys
    assert "LSTM_state" in firmware_keys
    assert "secloc_skipped_update" in firmware_keys
