from datetime import datetime

from globals import CONTROL_PERIOD_MS, PATH_TO_EXPERIMENT_RECORDINGS


def create_csv_title():

    title = (f"This is a recording from physical cartpole from {datetime.now().strftime('%d.%m.%Y')}" +
             f" at time {datetime.now().strftime('%H:%M:%S')}")

    return title


def create_csv_header():

    header = [
        f"Time intervals dt:",
        f"Simulation: not applying",
        f"Controller update: {str(CONTROL_PERIOD_MS / 1000)} s",
        f"Saving: {str(CONTROL_PERIOD_MS / 1000)} s",
        f"",

        f"Units:",
        f"time: s",
        f"deltaTimeMs: ms",
        f"angle: rad",
        f"angleD: rad/s",
        f"position: m",
        f"positionD: m/s",
        f"angleTarget: rad",
        f"angleErr: rad",
        f"target_position: m",
        f"positionErr: m",
        f"Q: normed motor power",
        f""
        f"Data:"
    ]

    return header
