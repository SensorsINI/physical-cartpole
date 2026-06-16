import argparse
import os
import sys
import time

import numpy as np


DRIVER_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CARTPOLE_SIM_ROOT = os.path.join(DRIVER_ROOT, "CartPoleSimulation")

sys.path.insert(0, DRIVER_ROOT)
sys.path.insert(1, CARTPOLE_SIM_ROOT)

from DriverFunctions.interface import Interface  # noqa: E402
from CartPoleSimulation.Control_Toolkit.serial_interface_helper import get_serial_port  # noqa: E402
from globals import CHIP, CONTROL_SYNC, SERIAL_BAUD, SERIAL_PORT_NUMBER  # noqa: E402


def angle_deviation_from_hanging(angle_hanging, angle_360_deg_in_adc_units):
    if angle_hanging < angle_360_deg_in_adc_units / 2:
        return -angle_hanging - angle_360_deg_in_adc_units / 2
    return -angle_hanging + angle_360_deg_in_adc_units / 2


def measure_raw_angle(interface, samples, interval_us):
    raw = np.asarray(interface.collect_raw_angle(lenght=samples, interval_us=interval_us), dtype=float)
    return float(raw.mean()), float(raw.std())


def print_result(hanging_mean, hanging_std, upright_mean, upright_std):
    half_range = abs(upright_mean - hanging_mean)
    angle_360 = 2.0 * half_range
    angle_deviation = angle_deviation_from_hanging(hanging_mean, angle_360)

    print("\nAngle scale calibration result")
    print(f"ANGLE_HANGING: {hanging_mean:.3f} ADC reading (std {hanging_std:.3f})")
    print(f"UPRIGHT_RAW: {upright_mean:.3f} ADC reading (std {upright_std:.3f})")
    print(f"ANGLE_360_DEG_IN_ADC_UNITS: {angle_360:.3f}")
    print(f"ANGLE_DEVIATION: {angle_deviation:.3f} ADC reading")
    print("\nSuggested constants:")
    print(f"ANGLE_360_DEG_IN_ADC_UNITS = {angle_360:.2f}")
    print(f"ANGLE_HANGING_POLOLU = {hanging_mean:.3f}")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Interactive angle sensor scale calibration. Press b with the pole hanging down, "
            "then put the pole upright and press u."
        )
    )
    parser.add_argument("--port", default=None, help="Serial port, e.g. /dev/ttyUSB1. Auto-detected if omitted.")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help=f"Serial baud rate. Default: {SERIAL_BAUD}.")
    parser.add_argument("--samples", type=int, default=1000, help="Raw ADC samples per pose. Default: 1000.")
    parser.add_argument("--interval-us", type=int, default=100, help="Sampling interval in microseconds. Default: 100.")
    return parser.parse_args()


def main():
    args = parse_args()
    port = args.port or get_serial_port(chip_type=CHIP, serial_port_number=SERIAL_PORT_NUMBER)
    if port is None:
        raise RuntimeError("No serial port found. Pass --port explicitly.")

    interface = Interface()
    print(f"Opening {port} at {args.baud} baud")
    interface.open(port, args.baud)

    hanging = None

    try:
        interface.pc_control_mode(False)
        interface.control_mode(False)
        interface.set_motor(0)
        interface.stream_output(False)
        time.sleep(0.1)

        print("\nPlace the pole hanging down and press 'b'.")
        print("Then place the pole upright and press 'u'. Press 'q' to quit.")
        print(f"Samples per pose: {args.samples}, interval: {args.interval_us} us")
        print(f"CONTROL_SYNC is currently {CONTROL_SYNC}; this script only reads raw angle samples.")

        while True:
            key = input("\nCommand [b=measure down, u=measure upright, q=quit]: ").strip().lower()
            if key == "q":
                break
            if key == "b":
                print("Measuring hanging-down angle...")
                hanging = measure_raw_angle(interface, args.samples, args.interval_us)
                print(f"Down: mean={hanging[0]:.3f}, std={hanging[1]:.3f}")
            elif key == "u":
                if hanging is None:
                    print("Measure hanging-down first with 'b'.")
                    continue
                print("Measuring upright angle...")
                upright = measure_raw_angle(interface, args.samples, args.interval_us)
                print(f"Upright: mean={upright[0]:.3f}, std={upright[1]:.3f}")
                print_result(hanging[0], hanging[1], upright[0], upright[1])
            else:
                print("Unknown command.")

    finally:
        interface.set_motor(0)
        interface.pc_control_mode(False)
        interface.control_mode(False)
        interface.close()


if __name__ == "__main__":
    main()
